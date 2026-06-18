#include "Copter.h"

#if MODE_RESCUE_ENABLED

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool ModeRescue::init(bool ignore_checks)
{
    _current_idx       = 0;
    _phase             = RescuePhase::IDLE;
    _target_px_fresh   = false;
    _target_px_last_ms = 0;
    _smooth_vx         = 0.0f;
    _smooth_vy         = 0.0f;
    _lifebuoy_deployed = false;
    _wpnav_initialised = false;
    _has_inserted_wp   = false;
    _mission_start_ms  = 0;
    _wps_from_generate = false;

    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: ready, awaiting WP upload or GENERATE_WPS");
    return true;
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------
void ModeRescue::run()
{
    send_status();

    switch (_phase) {
    case RescuePhase::IDLE:          ModeGuided::run();     break;
    case RescuePhase::WPS_GENERATED: ModeGuided::run();     break;  // hold, wait for START_SEARCH
    case RescuePhase::TAKEOFF:       takeoff_pending_run(); break;
    case RescuePhase::TAKING_OFF:    taking_off_run();      break;
    case RescuePhase::WP_NAV:        wp_nav_run();          break;
    case RescuePhase::INSERT_NAV:    insert_nav_run();      break;
    case RescuePhase::CENTERING:     centering_run();       break;
    case RescuePhase::DEPLOYING:     deploying_run();       break;
    case RescuePhase::GUIDED:        ModeGuided::run();     break;
    }
}

// ---------------------------------------------------------------------------
// apply_nav_alt — set RSC_NAV_ALT as ABOVE_HOME if location has no altitude
// nav_alt is AP_Float
// ---------------------------------------------------------------------------
void ModeRescue::apply_nav_alt(Location &loc) const
{
    if (loc.alt == 0) {
        loc.set_alt_m((float)g2.rescue.nav_alt, Location::AltFrame::ABOVE_HOME);
    }
}

// ---------------------------------------------------------------------------
// generate_lawn_pattern
// Now takes total_dist_m as a parameter (from GENERATE_WPS.length or default).
// Uses: RSC_NAV_ALT (AP_Float), RSC_GMB_HFOV (AP_Float)
// Mirrors OBC's generate_lawn_waypoints_lat_long()
// ---------------------------------------------------------------------------
bool ModeRescue::generate_lawn_pattern(float total_dist_m)
{
    const float alt_m    = (float)g2.rescue.nav_alt;       // AP_Float
    const float hfov_rad = radians((float)g2.rescue.gmb_hfov); // AP_Float
    const float overlap  = 0.2f;
    const float strip_w  = 2.0f * alt_m * tanf(hfov_rad * 0.5f) * (1.0f - overlap);

    if (strip_w < 0.5f) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: strip width %.2fm too small, check RSC_NAV_ALT/RSC_GMB_HFOV",
            (double)strip_w);
        return false;
    }

    if (total_dist_m < 10.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: search length %.0fm too small", (double)total_dist_m);
        return false;
    }

    const float start_lat = copter.current_loc.lat * 1e-7f;
    const float start_lon = copter.current_loc.lng * 1e-7f;
    const float hdg_rad   = radians(copter.ahrs.yaw_sensor * 0.01f);

    const float fwd_n = cosf(hdg_rad);
    const float fwd_e = sinf(hdg_rad);
    const float rgt_n = cosf(hdg_rad + M_PI_2);
    const float rgt_e = sinf(hdg_rad + M_PI_2);

    const float R        = 6378137.0f;
    const float lat0_rad = radians(start_lat);
    const int   n_strips = (int)(total_dist_m / strip_w);

    _wp_count       = 0;
    _expected_count = 0;

    auto add_wp = [&](float north_m, float east_m) -> bool {
        if (_wp_count >= RESCUE_WP_MAX) return false;
        Location wp{};
        wp.lat          = (int32_t)((start_lat + degrees(north_m / R)) * 1e7f);
        wp.lng          = (int32_t)((start_lon +
                           degrees(east_m / (R * cosf(lat0_rad)))) * 1e7f);
        wp.alt          = 0;
        wp.relative_alt = false;
        _waypoints[_wp_count++] = wp;
        return true;
    };

    for (int s = 0; s < n_strips && _wp_count < (int)RESCUE_WP_MAX - 1; s++) {
        const float side = (s + 0.5f) * strip_w;
        const float dir  = (s % 2 == 0) ? 1.0f : -1.0f;
        if (!add_wp(rgt_n * side, rgt_e * side)) break;
        if (!add_wp(rgt_n * side + fwd_n * total_dist_m * dir,
                    rgt_e * side + fwd_e * total_dist_m * dir)) break;
    }

    _expected_count = _wp_count;

    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: generated %u lawn WPs (length=%.0fm strip=%.1fm)",
        _wp_count, (double)total_dist_m, (double)strip_w);

    return _wp_count > 0;
}

// ---------------------------------------------------------------------------
// echo_wps_to_gcs
// Sends all generated/loaded waypoints back to all GCS channels via RESCUE_WP.
// QGC receives these, displays them, then sends START_SEARCH to confirm.
// ---------------------------------------------------------------------------
void ModeRescue::echo_wps_to_gcs()
{
    for (uint8_t i = 0; i < _wp_count; i++) {
        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            mavlink_msg_rescue_wp_send(
                gcs().chan(c)->get_chan(),
                _wp_count,       // total_count
                i,               // seq
                _waypoints[i].lat,
                _waypoints[i].lng
            );
        }
        // Small delay between burst sends to avoid flooding the link
        // (runs in the main loop context so we can't sleep — just send all at once;
        // the mavlink library buffers them)
    }

    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: sent %u WPs to GCS — send START_SEARCH to begin mission", _wp_count);
}

// ---------------------------------------------------------------------------
// wp_nav_set_destination — init wp_nav ONCE (Auto-style), then just set dest
// ---------------------------------------------------------------------------
bool ModeRescue::wp_nav_set_destination(const Location &dest)
{
    if (!_wpnav_initialised) {
        Vector3p origin_neu_m;
        bool have_origin = false;
        if (takeoff_complete) {
            have_origin = auto_takeoff.get_completion_pos_neu_m(origin_neu_m);
        }
        if (have_origin) {
            wp_nav->wp_and_spline_init_m(0, origin_neu_m);
        } else {
            Vector3p stop_neu_m;
            wp_nav->get_wp_stopping_point_NEU_m(stop_neu_m);
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

bool ModeRescue::wp_nav_set_destination_insert(const Location &dest)
{
    if (!wp_nav->set_wp_destination_loc(dest)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: failed to set INSERT WP destination");
        return false;
    }
    auto_yaw.set_mode_to_default(false);
    return true;
}

// ---------------------------------------------------------------------------
// TAKEOFF
// ---------------------------------------------------------------------------
void ModeRescue::takeoff_pending_run()
{
    if (!motors->armed()) {
        return;
    }

    if (!do_user_takeoff_start_m((float)g2.rescue.nav_alt)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: takeoff start failed, retrying");
        return;
    }

    copter.set_auto_armed(true);
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: taking off to %.1fm",
                    (double)(float)g2.rescue.nav_alt);
    _phase = RescuePhase::TAKING_OFF;
}

// ---------------------------------------------------------------------------
// TAKING_OFF
// ---------------------------------------------------------------------------
void ModeRescue::taking_off_run()
{
    ModeGuided::run();

    if (!is_taking_off() && takeoff_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: takeoff complete, starting search");
        _phase = RescuePhase::WP_NAV;

        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);

        if (!wp_nav_set_destination(dest)) {
            set_destination(copter.current_loc);
            return;
        }

        float alt_m = 0.0f;
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) {
            alt_m = dest.alt * 0.01f;
        }
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: WP 1/%u (%.1fm)", _wp_count, (double)alt_m);
    }
}

// ---------------------------------------------------------------------------
// WP_NAV
// ---------------------------------------------------------------------------
void ModeRescue::wp_nav_run()
{
    if (_target_px_fresh) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target detected → CENTERING");
        _phase = RescuePhase::CENTERING;
        _smooth_vx = 0.0f;
        _smooth_vy = 0.0f;
        velaccel_control_start();
        return;
    }

    if (_has_inserted_wp) {
        _has_inserted_wp = false;
        Location dest = _inserted_wp;
        apply_nav_alt(dest);
        if (wp_nav_set_destination_insert(dest)) {
            gcs().send_text(MAV_SEVERITY_INFO,
                "Rescue: navigating to inserted WP (%.7f, %.7f)",
                (double)(dest.lat * 1e-7f), (double)(dest.lng * 1e-7f));
            _phase = RescuePhase::INSERT_NAV;
        }
        return;
    }

    // miss_timeout is AP_Int32
    if (_mission_start_ms > 0 &&
        AP_HAL::millis() - _mission_start_ms >
        (uint32_t)((int32_t)g2.rescue.miss_timeout * 1000)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: mission timeout → dynamic landing");
        switch_to_dynamic_landing();
        return;
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(), auto_yaw.get_heading());

    if (wp_nav->reached_wp_destination()) {
        notify_wp_reached(_current_idx);
        advance_to_next_wp();
    }
}

// ---------------------------------------------------------------------------
// INSERT_NAV
// ---------------------------------------------------------------------------
void ModeRescue::insert_nav_run()
{
    if (_target_px_fresh) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target during insert nav → CENTERING");
        _phase = RescuePhase::CENTERING;
        _smooth_vx = 0.0f;
        _smooth_vy = 0.0f;
        velaccel_control_start();
        return;
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(), auto_yaw.get_heading());

    if (wp_nav->reached_wp_destination()) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: inserted WP reached, resuming pattern at WP %u/%u",
            _current_idx + 1, _wp_count);
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination_insert(dest);
    }
}

// ---------------------------------------------------------------------------
// CENTERING
// Uses: RSC_LIFE_ALT (life_dep_alt AP_Int16)
// ---------------------------------------------------------------------------
void ModeRescue::centering_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now_ms = AP_HAL::millis();
    const bool px_valid   = _target_px_fresh &&
                            (now_ms - _target_px_last_ms < 3000);

    float vx = 0.0f;
    float vy = 0.0f;

    if (px_valid) {
        compute_centering_velocity(vx, vy);
        _target_px_fresh = false;
    } else {
        _smooth_vx *= 0.95f;
        _smooth_vy *= 0.95f;
        vx = constrain_float(_smooth_vx, -1.5f, 1.5f);
        vy = constrain_float(_smooth_vy, -1.5f, 1.5f);
    }

    const float vz_down = 0.3f;
    const float hdg_rad = radians(copter.ahrs.yaw_sensor * 0.01f);
    const Vector3f vel_neu{
         vx * cosf(hdg_rad) - vy * sinf(hdg_rad),
         vx * sinf(hdg_rad) + vy * cosf(hdg_rad),
        -vz_down
    };

    set_vel_NEU_ms(vel_neu);
    ModeGuided::run();

    float check_alt;
    if (copter.rangefinder_alt_ok()) {
        check_alt = copter.rangefinder_state.alt_m_filt.get();
    } else {
        check_alt = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;
    }

    // life_dep_alt is AP_Int16
    if (check_alt <= (float)(int16_t)g2.rescue.life_dep_alt && !_lifebuoy_deployed) {
        fire_lifebuoy_servos();
        _lifebuoy_deployed = true;
        _deploy_time_ms    = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: lifebuoy deployed at %.1fm", (double)check_alt);
        _phase = RescuePhase::DEPLOYING;
    }
}

// ---------------------------------------------------------------------------
// compute_centering_velocity
// ---------------------------------------------------------------------------
void ModeRescue::compute_centering_velocity(float &vx_out, float &vy_out)
{
    float alt_m;
    if (copter.rangefinder_alt_ok()) {
        alt_m = copter.rangefinder_state.alt_m_filt.get();
    } else {
        alt_m = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;
    }
    alt_m = MAX(alt_m, 3.0f);

    const float alt_scale = alt_m / 30.0f;
    const float raw_vx    = -CENTER_PX_SCALE * (float)_target_dy * alt_scale;
    const float raw_vy    =  CENTER_PX_SCALE * (float)_target_dx * alt_scale;

    _smooth_vx = CENTER_ALPHA * raw_vx + (1.0f - CENTER_ALPHA) * _smooth_vx;
    _smooth_vy = CENTER_ALPHA * raw_vy + (1.0f - CENTER_ALPHA) * _smooth_vy;

    vx_out = constrain_float(_smooth_vx, -1.5f, 1.5f);
    vy_out = constrain_float(_smooth_vy, -1.5f, 1.5f);
}

// ---------------------------------------------------------------------------
// DEPLOYING
// ---------------------------------------------------------------------------
void ModeRescue::deploying_run()
{
    ModeGuided::run();

    if (AP_HAL::millis() - _deploy_time_ms > 5000) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: deploy hold done → dynamic landing");
        switch_to_dynamic_landing();
    }
}

// ---------------------------------------------------------------------------
// switch_to_dynamic_landing
// ---------------------------------------------------------------------------
void ModeRescue::switch_to_dynamic_landing()
{
    if (!copter.set_mode(Mode::Number::DYNAMIC_LANDING, ModeReason::MISSION_END)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: failed to switch to DYNAMIC_LANDING, entering GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
    }
}

// ---------------------------------------------------------------------------
// fire_lifebuoy_servos
// life_pwm_ch1/2/3 are AP_Int16 → (uint8_t)(int16_t) for channel number
// life_pwm_val1/2/3 are AP_Int16 → (uint16_t)(int16_t) for PWM value
// ---------------------------------------------------------------------------
void ModeRescue::fire_lifebuoy_servos()
{
    const uint8_t ch[3] = {
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch1,
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch2,
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch3
    };
    const uint16_t val[3] = {
        (uint16_t)(int16_t)g2.rescue.life_pwm_val1,
        (uint16_t)(int16_t)g2.rescue.life_pwm_val2,
        (uint16_t)(int16_t)g2.rescue.life_pwm_val3
    };

    for (uint8_t i = 0; i < 3; i++) {
        SRV_Channels::set_output_pwm_chan_timeout(ch[i] - 1, val[i], 10000);
    }

    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: servos ch%u=%u ch%u=%u ch%u=%u",
        ch[0], val[0], ch[1], val[1], ch[2], val[2]);
}

// ---------------------------------------------------------------------------
// compute_hold_point
// Uses: RSC_STOP_ACC (AP_Float), RSC_MAX_STOP_DIS (AP_Int16)
// Mirrors OBC's go_to_hold_point()
// ---------------------------------------------------------------------------
void ModeRescue::compute_hold_point(const Location &drone_loc,
                                     float v_north, float v_east,
                                     Location &hold_loc) const
{
    const float v = sqrtf(v_north * v_north + v_east * v_east);

    if (v < 0.3f) {
        hold_loc = drone_loc;
        return;
    }

    // stop_acc is AP_Float, max_stop_dis is AP_Int16
    float d = (v * v) / (2.0f * (float)g2.rescue.stop_acc) + v * 0.2f;
    d = MIN(d, (float)(int16_t)g2.rescue.max_stop_dis);

    const float dir_n    = v_north / v;
    const float dir_e    = v_east  / v;
    const float R        = 6378137.0f;
    const float lat0_rad = radians(drone_loc.lat * 1e-7f);

    hold_loc.lat = drone_loc.lat +
                   (int32_t)(degrees(dir_n * d / R) * 1e7f);
    hold_loc.lng = drone_loc.lng +
                   (int32_t)(degrees(dir_e * d / (R * cosf(lat0_rad))) * 1e7f);
    hold_loc.set_alt_m((float)g2.rescue.nav_alt, Location::AltFrame::ABOVE_HOME);
}

// ---------------------------------------------------------------------------
// advance_to_next_wp
// ---------------------------------------------------------------------------
void ModeRescue::advance_to_next_wp()
{
    if (_current_idx + 1 < _wp_count) {
        _current_idx++;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);

        if (!wp_nav_set_destination_insert(dest)) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                "Rescue: failed WP %u, holding", _current_idx + 1);
            set_destination(copter.current_loc);
            return;
        }

        float alt_m = 0.0f;
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) {
            alt_m = dest.alt * 0.01f;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u/%u (%.1fm)",
                        _current_idx + 1, _wp_count, (double)alt_m);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: all %u WPs done, no target → dynamic landing", _wp_count);
        switch_to_dynamic_landing();
    }
}

void ModeRescue::notify_wp_reached(uint8_t idx)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u reached", idx + 1);
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(gcs().chan(c)->get_chan(), idx, 1);
    }
}

// ---------------------------------------------------------------------------
// MAVLink handlers
// ---------------------------------------------------------------------------

void ModeRescue::handle_rescue_wp(uint16_t seq, uint16_t total_count,
                                   int32_t lat_degE7, int32_t lon_degE7)
{
    // If WPs were generated internally, ignore external uploads
    // unless we're back in IDLE
    if (_wps_from_generate && _phase != RescuePhase::IDLE) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: RESCUE_WP ignored — using internally generated WPs");
        return;
    }

    if (seq >= RESCUE_WP_MAX) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "RESCUE_WP: seq %u exceeds max %u", seq, RESCUE_WP_MAX);
        return;
    }

    _wps_from_generate = false;
    _expected_count    = total_count;

    Location wp{};
    wp.lat          = lat_degE7;
    wp.lng          = lon_degE7;
    wp.alt          = 0;
    wp.relative_alt = false;
    _waypoints[seq] = wp;

    if (static_cast<uint16_t>(seq + 1) > _wp_count) {
        _wp_count = seq + 1;
    }

    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "RESCUE_WP: all %u WPs received — send START_SEARCH to begin",
            _wp_count);
        // Echo back to GCS to confirm receipt
        echo_wps_to_gcs();
        _phase = RescuePhase::WPS_GENERATED;
    }
}

void ModeRescue::handle_generate_wps(uint16_t length_m)
{
    if (copter.flightmode != this) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: GENERATE_WPS ignored — not in RESCUE mode");
        return;
    }

    if (_phase != RescuePhase::IDLE && _phase != RescuePhase::WPS_GENERATED) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: GENERATE_WPS ignored — mission already in progress (phase %u)",
            (uint8_t)_phase);
        return;
    }

    if (copter.current_loc.lat == 0 && copter.current_loc.lng == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: GENERATE_WPS rejected — no GPS fix");
        return;
    }

    _wp_count          = 0;
    _expected_count    = 0;
    _wps_from_generate = true;

    const float dist_m = (float)length_m;
    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: generating lawn pattern (length=%.0fm)...", (double)dist_m);

    if (!generate_lawn_pattern(dist_m)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: pattern generation failed");
        _phase = RescuePhase::IDLE;
        _wps_from_generate = false;
        return;
    }

    echo_wps_to_gcs();

    _phase = RescuePhase::WPS_GENERATED;

    // Force immediate status send so QGC sees phase = WPS_GENERATED
    _last_status_ms = 0;
    send_status();
}

void ModeRescue::handle_target_detected(int16_t dx, int16_t dy)
{
    _target_dx         = dx;
    _target_dy         = dy;
    _target_px_fresh   = true;
    _target_px_last_ms = AP_HAL::millis();
}

void ModeRescue::handle_insert_wp(int32_t lat_degE7, int32_t lon_degE7)
{
    if (_phase != RescuePhase::WP_NAV && _phase != RescuePhase::INSERT_NAV) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: INSERT_WP ignored — not in WP_NAV phase (current: %u)",
            (uint8_t)_phase);
        return;
    }

    _inserted_wp.lat          = lat_degE7;
    _inserted_wp.lng          = lon_degE7;
    _inserted_wp.alt          = 0;
    _inserted_wp.relative_alt = false;
    _has_inserted_wp          = true;

    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: WP inserted (%.7f, %.7f), going there next",
        (double)(lat_degE7 * 1e-7f), (double)(lon_degE7 * 1e-7f));
}

void ModeRescue::handle_start_search()
{
    if (copter.flightmode != this) return;

    // Must be in IDLE or WPS_GENERATED to start
    if (_phase != RescuePhase::IDLE && _phase != RescuePhase::WPS_GENERATED) {
        // Silently absorb GCS retries during active mission
        return;
    }

    // Must have waypoints ready
    if (_wp_count == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: START_SEARCH rejected — no WPs loaded. "
            "Send GENERATE_WPS or upload via RESCUE_WP first.");
        return;
    }

    // Beacon pre-check
    if (!_beacon_valid ||
        (AP_HAL::millis() - _beacon_last_ms > BEACON_TIMEOUT_MS)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: START_SEARCH rejected — HOME_BEACON_GPS not received. "
            "OBC must forward beacon before mission start.");
        return;
    }

    _current_idx       = 0;
    _target_px_fresh   = false;
    _lifebuoy_deployed = false;
    _wpnav_initialised = false;
    _has_inserted_wp   = false;
    _mission_start_ms  = AP_HAL::millis();

    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: confirmed — taking off to %.1fm when armed (%u WPs ready)",
            (double)(float)g2.rescue.nav_alt, _wp_count);
        _phase = RescuePhase::TAKEOFF;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: confirmed — starting search through %u WPs", _wp_count);
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination(dest);
    }
}

void ModeRescue::handle_deploy_lifebuoy()
{
    if (_lifebuoy_deployed) return;
    fire_lifebuoy_servos();
    _lifebuoy_deployed = true;
    _deploy_time_ms    = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: lifebuoy deployed (manual)");
    _phase = RescuePhase::DEPLOYING;
}

void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u %s",
                    wp_index + 1, reached ? "reached" : "not reached");
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(
            gcs().chan(c)->get_chan(), wp_index, reached);
    }
}

void ModeRescue::handle_home_beacon_gps(int32_t lat, int32_t lon,
                                          float heading,
                                          float v_north, float v_east)
{
    _beacon_lat     = lat * 1e-7f;
    _beacon_lon     = lon * 1e-7f;
    _beacon_last_ms = AP_HAL::millis();
    _beacon_valid   = true;
}

// ---------------------------------------------------------------------------
// send_status — RESCUE_STATUS (55004) at 1Hz
// phase values:
//   0=IDLE 1=TAKEOFF 2=TAKING_OFF 3=WP_NAV 4=INSERT_NAV
//   5=CENTERING 6=DEPLOYING 7=GUIDED 8=WPS_GENERATED
// ---------------------------------------------------------------------------
void ModeRescue::send_status()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_status_ms < STATUS_INTERVAL_MS) return;
    _last_status_ms = now;

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_rescue_status_send(
            gcs().chan(c)->get_chan(),
            static_cast<uint8_t>(_phase),
            _wp_count,
            _current_idx,
            (_wp_count > 0) ? 1 : 0   // wps_loaded
        );
    }
}

#endif // MODE_RESCUE_ENABLED
