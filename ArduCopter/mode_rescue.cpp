#include "Copter.h"

#if MODE_RESCUE_ENABLED

// ---------------------------------------------------------------------------
// init: called when entering RESCUE mode
// ---------------------------------------------------------------------------
bool ModeRescue::init(bool ignore_checks)
{
    _current_idx     = 0;
    _phase           = RescuePhase::IDLE;
    _target_detected = false;

    // Initialises Guided's controllers (defaults to velaccel submode).
    // All Guided submodes (WP, Pos, Accel, VelAccel, PosVelAccel, Angle,
    // TakeOff) remain available via inherited ModeGuided methods.
    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: ready, awaiting START_SEARCH");
    return true;
}

// ---------------------------------------------------------------------------
// run: main loop, called at 100hz+
// ---------------------------------------------------------------------------
void ModeRescue::run()
{
    switch (_phase) {
    case RescuePhase::IDLE:
        // Hold / idle using Guided's default (velaccel, zeroed) controller
        ModeGuided::run();
        break;

    case RescuePhase::TAKEOFF:
        takeoff_run_phase();
        break;

    case RescuePhase::WP_NAV:
        wp_nav_run();
        break;

    case RescuePhase::GUIDED:
        // ModeGuided::run() dispatches automatically to whichever submode
        // (Pos / VelAccel / PosVelAccel / Accel / Angle / WP) the OBC last
        // set via the corresponding set_xxx() call (triggered by
        // SET_POSITION_TARGET_* / SET_ATTITUDE_TARGET handlers).
        ModeGuided::run();
        break;
    }
}

// ---------------------------------------------------------------------------
// Helper: RESC_NAV_ALT in metres
// ---------------------------------------------------------------------------
float ModeRescue::rescue_nav_alt_m() const
{
    return g2.resc_nav_alt;
}

// ---------------------------------------------------------------------------
// Helper: apply RESC_NAV_ALT as alt-above-home if waypoint alt is unset
// ---------------------------------------------------------------------------
void ModeRescue::apply_nav_alt(Location &loc) const
{
    if (loc.alt == 0) {
        loc.set_alt_m(rescue_nav_alt_m(), Location::AltFrame::ABOVE_HOME);
    }
}

// ---------------------------------------------------------------------------
// TAKEOFF phase: drive Guided's TakeOff submode until complete, then start
// WP navigation
// ---------------------------------------------------------------------------
void ModeRescue::takeoff_run_phase()
{
    if (_target_detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected during takeoff, switching to GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
        return;
    }

    // Drives auto_takeoff and sets takeoff_complete when done
    ModeGuided::run();

    if (is_taking_off() == false && takeoff_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: takeoff complete, starting search");
        start_wp_nav();
    }
}

// ---------------------------------------------------------------------------
// Begin / restart WP navigation at _current_idx
// ---------------------------------------------------------------------------
void ModeRescue::start_wp_nav()
{
    _phase = RescuePhase::WP_NAV;

    Location dest = _waypoints[_current_idx];
    apply_nav_alt(dest);

    // Inherited from ModeGuided: switches guided_mode to SubMode::WP and
    // configures wp_nav controller toward dest.
    set_destination(dest);

    float alt_m;
    dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m);
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: navigating to WP %u/%u (alt %.1fm)",
                    _current_idx + 1, _wp_count, (double)alt_m);
}

// ---------------------------------------------------------------------------
// WP_NAV phase
// ---------------------------------------------------------------------------
void ModeRescue::wp_nav_run()
{
    if (_target_detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected, switching to GUIDED");
        _phase = RescuePhase::GUIDED;
        // Reset Guided to velaccel submode at zero so stale WP targets don't
        // cause a jump when OBC starts sending pos/vel/accel/attitude targets.
        velaccel_control_start();
        return;
    }

    // Drives wp_nav controller (guided_mode == SubMode::WP)
    ModeGuided::run();

    if (wp_nav->reached_wp_destination()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u reached", _current_idx + 1);

        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            mavlink_msg_user_wp_reached_send(
                gcs().chan(c)->get_chan(),
                _current_idx,
                1
            );
        }

        advance_to_next_wp();
    }
}

// ---------------------------------------------------------------------------
// Advance to next waypoint, or hold position if list exhausted
// ---------------------------------------------------------------------------
void ModeRescue::advance_to_next_wp()
{
    if (_current_idx + 1 < _wp_count) {
        _current_idx++;
        start_wp_nav();
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: all WPs done, holding position");
        set_destination(copter.current_loc);
    }
}

// ---------------------------------------------------------------------------
// Receive RESCUE_WP from GCS/OBC
// ---------------------------------------------------------------------------
void ModeRescue::handle_rescue_wp(uint16_t seq, uint16_t total_count,
                                   int32_t lat_degE7, int32_t lon_degE7)
{
    if (seq >= RESCUE_WP_MAX) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RESCUE_WP: seq %u exceeds max (%u), ignoring",
                        seq, RESCUE_WP_MAX);
        return;
    }

    _expected_count = total_count;

    Location wp{};
    wp.lat          = lat_degE7;
    wp.lng          = lon_degE7;
    wp.alt          = 0;            // resolved later via RESC_NAV_ALT
    wp.relative_alt = false;
    _waypoints[seq] = wp;

    if (static_cast<uint16_t>(seq + 1) > _wp_count) {
        _wp_count = seq + 1;
    }

    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "RESCUE_WP: all %u waypoints received, awaiting START_SEARCH", _wp_count);

        for (uint8_t i = 0; i < _wp_count; i++) {
            for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
                mavlink_msg_rescue_wp_send(
                    gcs().chan(c)->get_chan(),
                    _wp_count,
                    i,
                    _waypoints[i].lat,
                    _waypoints[i].lng
                );
            }
        }
        // stays IDLE until handle_start_search() is called
    }
}

// ---------------------------------------------------------------------------
// RESCUE_START_SEARCH: takeoff (if landed) then WP nav
// ---------------------------------------------------------------------------
void ModeRescue::handle_start_search()
{
    if (copter.flightmode != this) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: not in RESCUE mode, ignoring START_SEARCH");
        return;
    }

    if (_wp_count == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no waypoints loaded, ignoring START_SEARCH");
        return;
    }

    if (_phase != RescuePhase::IDLE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: search already in progress");
        return;
    }

    _current_idx = 0;

    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: taking off to %.1fm before search",
                        (double)rescue_nav_alt_m());

        if (!do_user_takeoff_start_m(rescue_nav_alt_m())) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: takeoff failed, aborting search start");
            return;
        }

        _phase = RescuePhase::TAKEOFF;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: starting search through %u waypoints", _wp_count);
        start_wp_nav();
    }
}

// ---------------------------------------------------------------------------
// Called when OBC reports a detected target (via TARGET_PX handler)
// ---------------------------------------------------------------------------
void ModeRescue::handle_target_detected()
{
    _target_detected = true;
}

// ---------------------------------------------------------------------------
// Forward user-wp-reached status to all GCS channels
// ---------------------------------------------------------------------------
void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    if (reached) {
        gcs().send_text(MAV_SEVERITY_INFO, "Waypoint %u reached", wp_index);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Waypoint %u not reached", wp_index);
    }

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(
            gcs().chan(c)->get_chan(),
            wp_index,
            reached
        );
    }
}

#endif // MODE_RESCUE_ENABLED
