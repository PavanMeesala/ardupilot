#include "Copter.h"

#if MODE_RESCUE_ENABLED

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool ModeRescue::init(bool ignore_checks)
{
    _current_idx       = 0;
    _phase             = RescuePhase::IDLE;
    _target_detected   = false;
    _wpnav_initialised = false;

    // Guided init defaults to velaccel submode; all guided submodes remain
    // available via inherited ModeGuided methods.
    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: ready, awaiting START_SEARCH");
    return true;
}

// ---------------------------------------------------------------------------
// run — 100hz+
// ---------------------------------------------------------------------------
void ModeRescue::run()
{
    send_status(); 
    switch (_phase) {

    case RescuePhase::IDLE:
        ModeGuided::run();
        break;

    case RescuePhase::TAKEOFF:
        takeoff_phase_run();
        break;

    case RescuePhase::TAKING_OFF:
        taking_off_run();
        break;

    case RescuePhase::WP_NAV:
        wp_nav_run();
        break;

    case RescuePhase::GUIDED:
        // All guided submodes (pos/vel/posvel/accel/angle) dispatched here
        ModeGuided::run();
        break;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
float ModeRescue::rescue_nav_alt_m() const
{
    return g2.rescue.nav_alt;
}

void ModeRescue::apply_nav_alt(Location &loc) const
{
    if (loc.alt == 0) {
        loc.set_alt_m(rescue_nav_alt_m(), Location::AltFrame::ABOVE_HOME);
    }
}

// ---------------------------------------------------------------------------
// wp_nav_set_destination — mimics Auto's wp_start():
//   - initialises wp_nav ONCE per search (not on every waypoint)
//   - on first call after takeoff, passes takeoff completion position as
//     the spline origin so there is no discontinuity
//   - on subsequent calls just sets destination, no reinit
// ---------------------------------------------------------------------------
bool ModeRescue::wp_nav_set_destination(const Location &dest)
{
    if (!_wpnav_initialised) {
        Vector3p origin_neu_m;   // <-- was origin_ned_m
        bool have_origin = false;
        if (_phase == RescuePhase::TAKING_OFF || takeoff_complete) {
            have_origin = auto_takeoff.get_completion_pos_neu_m(origin_neu_m);  // <-- fixed
        }

        if (have_origin) {
            wp_nav->wp_and_spline_init_m(0, origin_neu_m);   // <-- was origin_ned_m
        } else {
            Vector3p stop_neu_m;
            wp_nav->get_wp_stopping_point_NEU_m(stop_neu_m); // check your wp_nav API — may be get_wp_stopping_point_NED_m
            wp_nav->wp_and_spline_init_m(0, stop_neu_m);
        }
        _wpnav_initialised = true;
    }

    if (!wp_nav->set_wp_destination_loc(dest)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: failed to set WP destination");
        return false;
    }

    auto_yaw.set_mode_to_default(false);
    return true;
}

// ---------------------------------------------------------------------------
// TAKEOFF phase: wait until armed, then kick off takeoff
// ---------------------------------------------------------------------------
void ModeRescue::takeoff_phase_run()
{
    if (_target_detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected, switching to GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
        return;
    }

    if (!motors->armed()) {
        if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
            // Pre-arm checks not passing yet — retry next loop
            return;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: auto-armed for search");
    }

    if (!do_user_takeoff_start_m(rescue_nav_alt_m())) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: takeoff start failed, retrying");
        return;
    }

    copter.set_auto_armed(true);

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: taking off to %.1fm",
                    (double)rescue_nav_alt_m());
    _phase = RescuePhase::TAKING_OFF;
}

// ---------------------------------------------------------------------------
// TAKING_OFF phase: drive auto_takeoff, then start WP nav
// ---------------------------------------------------------------------------
void ModeRescue::taking_off_run()
{
    if (_target_detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected during takeoff, switching to GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
        return;
    }

    // guided_mode == SubMode::TakeOff here, so ModeGuided::run() calls takeoff_run()
    ModeGuided::run();

    // is_taking_off() == (guided_mode == TakeOff && !takeoff_complete)
    // so this triggers the moment takeoff_complete goes true
    if (!is_taking_off() && takeoff_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: takeoff complete, starting search");
        _phase = RescuePhase::WP_NAV;

        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);

        if (!wp_nav_set_destination(dest)) {
            // destination failed — hold at takeoff completion point
            set_destination(copter.current_loc);
            return;
        }

        float alt_m = 0.0f;
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) {
            alt_m = dest.alt * 0.01f;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: navigating to WP %u/%u (alt %.1fm)",
                        _current_idx + 1, _wp_count, (double)alt_m);
    }
}

// ---------------------------------------------------------------------------
// WP_NAV phase — runs wp_nav controller directly (like Auto's wp_run())
// ---------------------------------------------------------------------------
void ModeRescue::wp_nav_run()
{
    if (_target_detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected, switching to GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
        return;
    }

    // ---- replicate Auto's wp_run() exactly ----

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller and record terrain failure
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wp_nav already updated alt target)
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // ---- check if destination reached ----
    if (wp_nav->reached_wp_destination()) {
        notify_wp_reached(_current_idx);
        advance_to_next_wp();
    }
}

// ---------------------------------------------------------------------------
// Advance to next WP or hold at last one
// ---------------------------------------------------------------------------
void ModeRescue::advance_to_next_wp()
{
    if (_current_idx + 1 < _wp_count) {
        _current_idx++;

        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);

        if (!wp_nav_set_destination(dest)) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Rescue: failed to set WP %u, holding", _current_idx + 1);
            set_destination(copter.current_loc);
            return;
        }

        float alt_m = 0.0f;
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) {
            alt_m = dest.alt * 0.01f;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: navigating to WP %u/%u (alt %.1fm)",
                        _current_idx + 1, _wp_count, (double)alt_m);
    } else {
        // gcs().send_text(MAV_SEVERITY_INFO, "Rescue: all %u WPs done, holding position", _wp_count);
        set_destination(copter.current_loc);
    }
}

// ---------------------------------------------------------------------------
// Send USER_WP_REACHED to all GCS channels
// ---------------------------------------------------------------------------
void ModeRescue::notify_wp_reached(uint8_t idx)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u reached", idx + 1);

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(
            gcs().chan(c)->get_chan(),
            idx,    // wp_index (0-based, matches how you sent them)
            1       // reached = true
        );
    }
}

// ---------------------------------------------------------------------------
// Receive RESCUE_WP
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
    wp.alt          = 0;        // alt applied later via RESC_NAV_ALT
    wp.relative_alt = false;
    _waypoints[seq] = wp;

    if (static_cast<uint16_t>(seq + 1) > _wp_count) {
        _wp_count = seq + 1;
    }

    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "RESCUE_WP: all %u waypoints received, awaiting START_SEARCH",
                        _wp_count);

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
    }
}

// ---------------------------------------------------------------------------
// RESCUE_START_SEARCH received
// ---------------------------------------------------------------------------
void ModeRescue::handle_start_search()
{
    if (copter.flightmode != this) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Rescue: not in RESCUE mode, ignoring START_SEARCH");
        return;
    }

    if (_wp_count == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Rescue: no waypoints loaded, ignoring START_SEARCH");
        return;
    }

    if (_phase != RescuePhase::IDLE) {
        return;
    }

    _current_idx       = 0;
    _target_detected   = false;
    _wpnav_initialised = false;
    _wp_reached        = false;

    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Rescue: arming and taking off to %.1fm",
                        (double)rescue_nav_alt_m());
        _phase = RescuePhase::TAKEOFF;
    } else {
        // Already airborne
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Rescue: starting search through %u waypoints", _wp_count);
        _phase = RescuePhase::WP_NAV;

        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);

        if (wp_nav_set_destination(dest)) {
            float alt_m = 0.0f;
            if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) {
                alt_m = dest.alt * 0.01f;
            }
            gcs().send_text(MAV_SEVERITY_INFO,
                            "Rescue: navigating to WP 1/%u (alt %.1fm)",
                            _wp_count, (double)alt_m);
        }
    }
}

// ---------------------------------------------------------------------------
// TARGET_PX received — target detected by OBC
// ---------------------------------------------------------------------------
void ModeRescue::handle_target_detected()
{
    _target_detected = true;
}

// ---------------------------------------------------------------------------
// USER_WP_REACHED — called by GCS/OBC to manually report a WP status
// ---------------------------------------------------------------------------
void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u %s",
                    wp_index + 1, reached ? "reached" : "not reached");

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(
            gcs().chan(c)->get_chan(),
            wp_index,
            reached
        );
    }
}
void ModeRescue::send_status()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_status_send_ms < STATUS_SEND_INTERVAL_MS) {
        return;
    }
    _last_status_send_ms = now;

    const uint8_t phase      = static_cast<uint8_t>(_phase);
    const uint8_t wp_total   = _wp_count;
    const uint8_t wp_current = _current_idx;
    const uint8_t wps_loaded = rescue_wps_complete() ? 1 : 0;

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_rescue_status_send(
            gcs().chan(c)->get_chan(),
            phase,
            wp_total,
            wp_current,
            wps_loaded
        );
    }
}

#endif // MODE_RESCUE_ENABLED
