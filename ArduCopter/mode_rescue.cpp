#include "Copter.h"

#if MODE_RESCUE_ENABLED

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool ModeRescue::init(bool ignore_checks)
{
    _current_idx                = 0;
    _phase                      = RescuePhase::IDLE;
    _wpnav_initialised          = false;
    _wps_from_generate          = false;
    _has_inserted_wp            = false;
    _mission_start_ms           = 0;
    _target_dx                  = 0;
    _target_dy                  = 0;
    _target_px_valid            = false;
    _target_px_last_ms          = 0;
    _detection_window_start_ms  = 0;
    _detection_count            = 0;
    _tracking_active            = false;
    _first_wp_reached           = false;
    _hold_point_start_ms        = 0;
    // _target_gps_valid           = false;
    _target_approach_start_ms   = 0;
    _smooth_vx                  = 0.0f;
    _smooth_vy                  = 0.0f;
    _lifebuoy_deployed          = false;
    _deploy_time_ms             = 0;

    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: ready");
    return true;
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------
void ModeRescue::run()
{
    send_status();

    switch (_phase) {
    case RescuePhase::IDLE:            ModeGuided::run();       break;
    case RescuePhase::WPS_GENERATED:   ModeGuided::run();       break;
    case RescuePhase::TAKEOFF:         takeoff_pending_run();   break;
    case RescuePhase::TAKING_OFF:      taking_off_run();        break;
    case RescuePhase::WP_NAV:          wp_nav_run();            break;
    case RescuePhase::INSERT_NAV:      insert_nav_run();        break;
    case RescuePhase::HOLD_POINT:      hold_point_run();        break;
    case RescuePhase::TARGET_APPROACH: target_approach_run();   break;
    case RescuePhase::CENTERING:       centering_run();         break;
    case RescuePhase::DEPLOYING:       deploying_run();         break;
    case RescuePhase::GUIDED:          ModeGuided::run();       break;
    }
}

void ModeRescue::update_detection_window()
{
    const uint32_t now = AP_HAL::millis();

    if (now - _detection_window_start_ms > DETECTION_WINDOW_MS) {
        _detection_count           = 0;
        _detection_window_start_ms = now;
    }

    _detection_count++;

    const bool enough = (_detection_count >= MIN_DETECTIONS_FOR_TRACK);
    const bool recent = (now - _target_px_last_ms < 3000);
    _tracking_active  = enough && recent;
}

// ---------------------------------------------------------------------------
// handle_target_detected
// ---------------------------------------------------------------------------
void ModeRescue::handle_target_detected(int16_t dx, int16_t dy)
{
    _target_dx         = dx;
    _target_dy         = dy;
    _target_px_valid   = true;
    _target_px_last_ms = AP_HAL::millis();

    update_detection_window();
}
// ---------------------------------------------------------------------------
// compute_hold_point — port of navigation.py go_to_hold_point()
// ---------------------------------------------------------------------------
void ModeRescue::compute_hold_point(Vector3p &hold_loc) 
{
    float yaw_rad = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_rads = 0.0f; 
    bool yaw_ignore = true;
    bool yaw_rate_ignore = true;
    Vector3f vel; 
    if (!copter.ahrs.get_velocity_NED(vel)) { 
        // Velocity unavailable 
        return; } 
    const float v_north = vel.x; 
    const float v_east = vel.y;
    const float v       = sqrtf(v_north * v_north + v_east * v_east);

    if (v < 0.3f) {
        hold_loc = Vector3p(0.0f, 0.0f, 0.0f);
        copter.mode_guided.set_pos_NEU_m(hold_loc, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative, false);
        return;
    }

    float d = (v * v) / (2.0f * (float)g2.rescue.stop_acc) + v * 0.2f;
    d = MIN(d, (float)(int16_t)g2.rescue.max_stop_dis);

    const float dir_n    = v_north / v;
    const float dir_e    = v_east  / v;

    hold_loc = Vector3p(dir_n * d, dir_e * d, 0.0f);
    if (!AP::ahrs().get_relative_position_NED_origin(hold_loc)) {
        // need position estimate to calculate target position
        copter.mode_guided.init(true);
        return;
    }
    hold_loc.xy() += hold_loc.xy();
    hold_loc.z -= hold_loc.z;
    
    copter.mode_guided.set_pos_NEU_m(hold_loc, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative, false);
}

// ---------------------------------------------------------------------------
// apply_nav_alt
// ---------------------------------------------------------------------------
void ModeRescue::apply_nav_alt(Location &loc) const
{
    if (loc.alt == 0) {
        loc.set_alt_m((float)g2.rescue.nav_alt, Location::AltFrame::ABOVE_HOME);
    }
}

// ---------------------------------------------------------------------------
// generate_lawn_pattern — expanding conical pattern aligned to heading
// ---------------------------------------------------------------------------
bool ModeRescue::generate_lawn_pattern(float total_dist_m)
{
    const float alt_m    = (float)g2.rescue.nav_alt;
    const float hfov_rad = radians((float)g2.rescue.gmb_hfov);
    const float overlap  = 0.2f;
    const float strip_w  = 2.0f * alt_m * tanf(hfov_rad * 0.5f) * (1.0f - overlap);

    if (strip_w < 0.5f || total_dist_m < 10.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: pattern params invalid");
        return false;
    }

    const int   n_pairs   = MAX(1, (int)(total_dist_m / strip_w));
    const float start_lat = copter.current_loc.lat * 1e-7f;
    const float start_lon = copter.current_loc.lng * 1e-7f;
    const float hdg_rad   = radians(copter.ahrs.yaw_sensor * 0.01f);
    const float fwd_n     = cosf(hdg_rad);
    const float fwd_e     = sinf(hdg_rad);
    const float lshift_n  = fwd_e;
    const float lshift_e  = -fwd_n;
    const float max_hw    = total_dist_m * 0.5f;
    const float R         = 6378137.0f;
    const float lat0_rad  = radians(start_lat);

    auto ne_to_loc = [&](float north_m, float east_m) -> Location {
        Location loc{};
        loc.lat = (int32_t)((start_lat + degrees(north_m / R)) * 1e7f);
        loc.lng = (int32_t)((start_lon +
                   degrees(east_m / (R * cosf(lat0_rad)))) * 1e7f);
        loc.alt = 0;
        loc.relative_alt = false;
        return loc;
    };

    _wp_count = _expected_count = 0;

    for (int k = 0; k < n_pairs && _wp_count < (int)RESCUE_WP_MAX - 1; k++) {
        const float frac     = (n_pairs > 1) ? (float)k / (float)(n_pairs - 1) : 0.5f;
        const float fwd_dist = total_dist_m * frac;
        const float half_w   = max_hw * frac;
        const float ctr_n    = fwd_dist * fwd_n;
        const float ctr_e    = fwd_dist * fwd_e;
        Location left_wp  = ne_to_loc(ctr_n + half_w * lshift_n, ctr_e + half_w * lshift_e);
        Location right_wp = ne_to_loc(ctr_n - half_w * lshift_n, ctr_e - half_w * lshift_e);
        if (k % 2 == 0) {
            _waypoints[_wp_count++] = right_wp;
            if (_wp_count >= RESCUE_WP_MAX) break;
            _waypoints[_wp_count++] = left_wp;
        } else {
            _waypoints[_wp_count++] = left_wp;
            if (_wp_count >= RESCUE_WP_MAX) break;
            _waypoints[_wp_count++] = right_wp;
        }
    }

    _expected_count = _wp_count;
    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: %u WPs, hdg=%.0f deg, len=%.0fm", _wp_count,
        (double)(copter.ahrs.yaw_sensor * 0.01f), (double)total_dist_m);
    return _wp_count > 0;
}

void ModeRescue::echo_wps_to_gcs()
{
    for (uint8_t i = 0; i < _wp_count; i++) {
        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            mavlink_msg_rescue_wp_send(gcs().chan(c)->get_chan(),
                _wp_count, i, _waypoints[i].lat, _waypoints[i].lng);
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: sent %u WPs to GCS, awaiting START_SEARCH", _wp_count);
}

// void ModeRescue::send_current_route_to_gcs()
// {
//     const uint8_t remaining = (_current_idx < _wp_count) ? (_wp_count - _current_idx) : 0;
//     const uint8_t inserted  = _has_inserted_wp ? 1 : 0;
//     const uint8_t total     = inserted + remaining;
//     if (total == 0) return;

//     uint8_t seq = 0;
//     if (_has_inserted_wp) {
//         for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
//             mavlink_msg_rescue_wp_send(gcs().chan(c)->get_chan(),
//                 total, seq, _inserted_wp.lat, _inserted_wp.lng);
//         }
//         seq++;
//     }
//     for (uint8_t i = _current_idx; i < _wp_count; i++, seq++) {
//         for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
//             mavlink_msg_rescue_wp_send(gcs().chan(c)->get_chan(),
//                 total, seq, _waypoints[i].lat, _waypoints[i].lng);
//         }
//     }
// }

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
    if (!motors->armed()) return;
    if (!do_user_takeoff_start_m((float)g2.rescue.nav_alt)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: takeoff start failed");
        return;
    }
    copter.set_auto_armed(true);
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: taking off to %.1fm",
                    (double)(float)g2.rescue.nav_alt);
    _phase = RescuePhase::TAKING_OFF;
}

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
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) alt_m = dest.alt * 0.01f;
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP 1/%u (%.1fm)", _wp_count, (double)alt_m);
    }
}

// ---------------------------------------------------------------------------
// WP_NAV
// ---------------------------------------------------------------------------
void ModeRescue::wp_nav_run()
{
    if (_tracking_active &&
        AP_HAL::millis() - _target_px_last_ms >
        (uint32_t)(g2.rescue.track_timeout * 1000.0f)) {
        _tracking_active = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target lost, resuming WP nav");
    }

    if (_first_wp_reached && _tracking_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target locked, going to hold point");
        compute_hold_point(_hold_point_loc);
        _hold_point_start_ms = AP_HAL::millis();
        _phase = RescuePhase::HOLD_POINT;
        return;
    }

    if (_has_inserted_wp) {
        _has_inserted_wp = false;
        Location dest = _inserted_wp;
        apply_nav_alt(dest);
        if (wp_nav_set_destination_insert(dest)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Rescue: navigating to inserted WP");
            _phase = RescuePhase::INSERT_NAV;
            // send_current_route_to_gcs();
        }
        return;
    }

    if (_mission_start_ms > 0 &&
        AP_HAL::millis() - _mission_start_ms >
        (uint32_t)((int32_t)g2.rescue.miss_timeout * 1000)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: mission timeout");
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
        notify_wp_reached(_current_idx + _insert_nav_number);
        if (!_first_wp_reached) {
            _first_wp_reached = true;
            gcs().send_text(MAV_SEVERITY_INFO,
                "Rescue: first WP reached, detection/tracking enabled");
        }
        advance_to_next_wp();
    }
}

// ---------------------------------------------------------------------------
// INSERT_NAV
// ---------------------------------------------------------------------------
void ModeRescue::insert_nav_run()
{
    if (_tracking_active &&
        AP_HAL::millis() - _target_px_last_ms >
        (uint32_t)(g2.rescue.track_timeout * 1000.0f)) {
        _tracking_active = false;
    }

    if (_first_wp_reached && _tracking_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target during insert nav, hold point");
        compute_hold_point(_hold_point_loc);
        _hold_point_start_ms = AP_HAL::millis();
        _phase = RescuePhase::HOLD_POINT;
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
            "Rescue: inserted WP reached, resuming at WP %u/%u",
            _current_idx + 1, _wp_count);
        notify_wp_reached(_current_idx + _insert_nav_number);
        _insert_nav_number++;
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination_insert(dest);
        // send_current_route_to_gcs();
    }
}

// ---------------------------------------------------------------------------
// HOLD_POINT
// ---------------------------------------------------------------------------
void ModeRescue::hold_point_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(
        pos_control->get_thrust_vector(), auto_yaw.get_heading());

    const uint32_t now    = AP_HAL::millis();
    const uint32_t waited = now - _hold_point_start_ms;

    const bool target_locked = _target_px_valid && ((sq((float)_target_dx) + sq((float)_target_dy)) < 900.0f);

    if (target_locked) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: target locked (dx=%d dy=%d), target approach",
            (int)_target_dx, (int)_target_dy);
        _phase = RescuePhase::TARGET_APPROACH;
        _target_approach_start_ms = now;

        // if (_target_gps_valid) {
        //     Location dest = _target_gps_loc;
        //     apply_nav_alt(dest);
        //     wp_nav_set_destination_insert(dest);
        //     gcs().send_text(MAV_SEVERITY_INFO, "Rescue: flying to target GPS position");
        // }
        return;
    }

    if (waited >= HOLD_POINT_WAIT_MS) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: hold timeout, resuming pattern at WP %u/%u",
            _current_idx + 1, _wp_count);
        _tracking_active = false;
        _detection_count = 0;
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination_insert(dest);
    }
}

// ---------------------------------------------------------------------------
// TARGET_APPROACH
// ---------------------------------------------------------------------------
void ModeRescue::target_approach_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now     = AP_HAL::millis();
    const uint32_t elapsed = now - _target_approach_start_ms;

    // if (_target_gps_valid) {
    //     copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    // }
    // pos_control->update_U_controller();
    // attitude_control->input_thrust_vector_heading(
    //     pos_control->get_thrust_vector(), auto_yaw.get_heading());

    if (elapsed >= TARGET_APPROACH_WAIT_MS) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: approach wait done, centering");
        _smooth_vx = 0.0f;
        _smooth_vy = 0.0f;
        velaccel_control_start();
        _phase = RescuePhase::CENTERING;
    }
}

// ---------------------------------------------------------------------------
// CENTERING
// ---------------------------------------------------------------------------
void ModeRescue::centering_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now = AP_HAL::millis();
    float vx = 0.0f;
    float vy = 0.0f;

    if (_target_px_valid) {
        compute_centering_velocity(vx, vy);
    } else {
        vx = 0.0f;
        vy = 0.0f;
    }

    const float vz_down = 0.3f;
    Vector3f vel_body{vx, vy, -vz_down};
    vel_body.xy() = copter.ahrs.body_to_earth2D(vel_body.xy());
    Vector3f accel_neu_mss;
    float yaw_rad = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_rads = 0.0f; 
    bool yaw_ignore = true;
    bool yaw_rate_ignore = true;
    set_vel_accel_NEU_m(vel_body, accel_neu_mss, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads, yaw_relative);
    ModeGuided::run();

    const float rel_alt = (copter.current_loc.alt -
                            copter.ahrs.get_home().alt) * 0.01f;

    if (rel_alt < (float)(int16_t)g2.rescue.life_dep_alt && !_lifebuoy_deployed) {
        fire_lifebuoy_servos(1);
        _lifebuoy_deployed = true;
        _deploy_time_ms    = now;
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: lifebuoy deployed at rel_alt=%.1fm", (double)rel_alt);
        _phase = RescuePhase::DEPLOYING;
    }
}

// ---------------------------------------------------------------------------
// compute_centering_velocity
// ---------------------------------------------------------------------------
void ModeRescue::compute_centering_velocity(float &vx_out, float &vy_out)
{
    const float rel_alt = (copter.current_loc.alt -
                            copter.ahrs.get_home().alt) * 0.01f;
    const float altitude  = MAX(rel_alt, 3.0f);
    const float alt_scale = altitude / 30.0f;

    const float raw_vx = -CENTER_PX_SCALE * (float)_target_dy * alt_scale;
    const float raw_vy =  CENTER_PX_SCALE * (float)_target_dx * alt_scale;

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
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: deploy done, dynamic landing");
        switch_to_dynamic_landing();
    }
}

void ModeRescue::switch_to_dynamic_landing()
{
    if (!copter.set_mode(Mode::Number::DYNAMIC_LANDING, ModeReason::MISSION_END)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "Rescue: failed to switch mode, entering GUIDED");
        _phase = RescuePhase::GUIDED;
        velaccel_control_start();
    }
}

void ModeRescue::fire_lifebuoy_servos(bool deploy)
{
    const uint8_t ch[3] = {
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch1,
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch2,
        (uint8_t)(int16_t)g2.rescue.life_pwm_ch3
    };
    int pwm = 0;
    if (deploy) {
        pwm = g2.rescue.life_deploy_pwm;
    } else {
        pwm = g2.rescue.life_retract_pwm;
    }
    for (uint8_t i = 0; i < 3; i++) {
        SRV_Channels::set_output_pwm_chan_timeout(ch[i] - 1, pwm, 10000);
    }
    gcs().send_text(MAV_SEVERITY_INFO,
        "Rescue: servos ch%u=%u ch%u=%u ch%u=%u",
        ch[0], pwm, ch[1], pwm, ch[2], pwm);
}

void ModeRescue::advance_to_next_wp()
{
    if (_current_idx + 1 < _wp_count) {
        _current_idx++;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        if (!wp_nav_set_destination_insert(dest)) {
            set_destination(copter.current_loc);
            return;
        }
        float alt_m = 0.0f;
        if (!dest.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_m)) alt_m = dest.alt * 0.01f;
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u/%u (%.1fm)",
                        _current_idx + 1, _wp_count, (double)alt_m);
        // send_current_route_to_gcs();
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: all %u WPs done, dynamic landing", _wp_count);
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
    if (_wps_from_generate && _phase != RescuePhase::IDLE) return;
    if (seq >= RESCUE_WP_MAX) return;
    _wps_from_generate  = false;
    _expected_count     = total_count;
    Location wp{};
    wp.lat = lat_degE7; wp.lng = lon_degE7; wp.alt = 0; wp.relative_alt = false;
    _waypoints[seq] = wp;
    if (static_cast<uint16_t>(seq + 1) > _wp_count) _wp_count = seq + 1;
    if (rescue_wps_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO, "RESCUE_WP: all %u WPs received", _wp_count);
        echo_wps_to_gcs();
        _phase = RescuePhase::WPS_GENERATED;
    }
}

void ModeRescue::handle_generate_wps(uint16_t length_m)
{
    if (copter.flightmode != this) return;
    if (_phase != RescuePhase::IDLE && _phase != RescuePhase::WPS_GENERATED) return;
    if (copter.current_loc.lat == 0 && copter.current_loc.lng == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no GPS fix");
        return;
    }
    _wp_count = _expected_count = 0;
    _wps_from_generate = true;
    if (!generate_lawn_pattern((float)length_m)) {
        _phase = RescuePhase::IDLE;
        _wps_from_generate = false;
        return;
    }
    echo_wps_to_gcs();
    _phase = RescuePhase::WPS_GENERATED;
    _last_status_ms = 0;
    send_status();
}
void ModeRescue::handle_lifebuoy(bool deploy)
{
    if (_lifebuoy_deployed) return;
    fire_lifebuoy_servos(deploy);
    _lifebuoy_deployed = true;
    _deploy_time_ms    = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: lifebuoy deployed (manual)");
    _phase = RescuePhase::DEPLOYING;
}
void ModeRescue::handle_insert_wp(int32_t lat_degE7, int32_t lon_degE7)
{
    bool accepted = true;
    uint8_t reason = 0;

    if (_phase != RescuePhase::WP_NAV && _phase != RescuePhase::INSERT_NAV) {
        accepted = false;
        reason = 1;
    } else if (lat_degE7 == 0 && lon_degE7 == 0) {
        accepted = false;
        reason = 2;
    }

    if (!accepted) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: INSERT_WP rejected (reason=%u)", reason);
        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            mavlink_msg_rescue_insert_wp_ack_send(gcs().chan(c)->get_chan(),
                0, 0, 0, reason, _current_idx + _insert_nav_number);
        }
        return;
    }

    _inserted_wp.lat = lat_degE7; _inserted_wp.lng = lon_degE7;
    _inserted_wp.alt = 0; _inserted_wp.relative_alt = false;
    _has_inserted_wp = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP inserted (%.7f, %.7f)",
        (double)(lat_degE7*1e-7f), (double)(lon_degE7*1e-7f));

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_rescue_insert_wp_ack_send(gcs().chan(c)->get_chan(),
            lat_degE7, lon_degE7, 1, 0, _current_idx + _insert_nav_number);
    }

    // send_current_route_to_gcs();
}

void ModeRescue::handle_start_search()
{
    if (copter.flightmode != this) return;
    if (_phase != RescuePhase::IDLE && _phase != RescuePhase::WPS_GENERATED) return;
    if (_wp_count == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no WPs loaded");
        return;
    }
    if (!_beacon_valid ||
        (AP_HAL::millis() - _beacon_last_ms > BEACON_TIMEOUT_MS)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no beacon GPS, cannot start");
        return;
    }

    _current_idx                = 0;
    _tracking_active            = false;
    _target_px_valid            = false;
    _target_px_last_ms          = 0;
    _detection_count            = 0;
    _detection_window_start_ms  = 0;
    _first_wp_reached           = false;
    _smooth_vx                  = 0.0f;
    _smooth_vy                  = 0.0f;
    _lifebuoy_deployed          = false;
    _wpnav_initialised          = false;
    _has_inserted_wp             = false;
    _mission_start_ms           = AP_HAL::millis();

    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: queued, taking off to %.1fm when armed",
            (double)(float)g2.rescue.nav_alt);
        _phase = RescuePhase::TAKEOFF;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: starting search through %u WPs", _wp_count);
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination(dest);
    }
}

void ModeRescue::handle_user_wp_reached(uint16_t wp_index, uint8_t reached)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u %s",
                    wp_index+1, reached ? "reached" : "not reached");
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(gcs().chan(c)->get_chan(), wp_index, reached);
    }
}

void ModeRescue::handle_home_beacon_gps(int32_t lat, int32_t lon,
                                          float heading, float v_north, float v_east)
{
    _beacon_lat     = lat * 1e-7f;
    _beacon_lon     = lon * 1e-7f;
    _beacon_last_ms = AP_HAL::millis();
    _beacon_valid   = true;
}

void ModeRescue::send_status()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_status_ms < STATUS_INTERVAL_MS) return;
    _last_status_ms = now;
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_rescue_status_send(gcs().chan(c)->get_chan(),
            static_cast<uint8_t>(_phase), _wp_count, _current_idx,
            (_wp_count > 0) ? 1 : 0);
    }
}

#endif // MODE_RESCUE_ENABLED
