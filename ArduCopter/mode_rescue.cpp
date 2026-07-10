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
    _detection_window_start_ms  = 0;
    _tracking_active            = false;
    _first_wp_reached           = false;
    _hold_point_start_ms        = 0;
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

void ModeRescue::waypoint_control_start()
{
    // Initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_m();

    // Initialise wpnav to stopping point
    Vector3p stopping_point_neu_m;
    wp_nav->get_wp_stopping_point_NEU_m(stopping_point_neu_m);
    if (!wp_nav->set_wp_destination_NEU_m(stopping_point_neu_m, false)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // Initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void ModeRescue::posvelaccel_control_start()
{
    // Initialise horizontal speed, acceleration limits inside pos_control
    pos_control->set_max_speed_accel_NE_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->set_correction_speed_accel_NE_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // Initialize vertical speeds and acceleration limits
    pos_control->set_max_speed_accel_U_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_U_mss());
    pos_control->set_correction_speed_accel_U_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_U_mss());

    // Initialise core underlying feedback loops
    pos_control->init_U_controller();
    pos_control->init_NE_controller();

    // Initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// ---------------------------------------------------------------------------
// handle_target_detected
// ---------------------------------------------------------------------------
void ModeRescue::handle_target_detected(int16_t dx, int16_t dy)
{
    _target_dx         = dx;
    _target_dy         = dy;
    _target_px_valid   = true;
    _tracking_active   = true;
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
// generate_lawn_pattern
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
        loc.lng = (int32_t)((start_lon + degrees(east_m / (R * cosf(lat0_rad)))) * 1e7f);
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
    if (_first_wp_reached && _tracking_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target locked, going to hold point");
        set_hold_point();
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
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

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
    if (_first_wp_reached && _tracking_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: target during insert nav, hold point");
        set_hold_point();
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
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

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
    }
}

// ---------------------------------------------------------------------------
// set_hold_point
// ---------------------------------------------------------------------------
void ModeRescue::set_hold_point() 
{
    _hold_point_neu.zero();
    _target_vel_neu.zero();

    // Determine current groundspeed vector sizes via EKF
    if (!copter.ahrs.get_velocity_NED(_target_vel_neu)) { 
        return; 
    } 
    
    const float v_north = _target_vel_neu.x; 
    const float v_east  = _target_vel_neu.y;
    const float v       = sqrtf(v_north * v_north + v_east * v_east);

    // If already stationary, pull current local frame tracking point and lock it
    if (v < 0.3f) {
        Vector3p pos_ned_m;
        if (copter.ahrs.get_relative_position_NED_origin(pos_ned_m)) {
            _hold_point_neu.xy() = pos_ned_m.xy();
            _hold_point_neu.z = -pos_ned_m.z;
        }
        set_pos_NEU_m(_hold_point_neu, false, 0.0f, false, 0.0f, false, false);
        return;
    }

    // Calculate forward deceleration path scaling
    float d = (v * v) / (2.0f * (float)g2.rescue.stop_acc) + v * 0.2f;
    d = MIN(d, (float)(int16_t)g2.rescue.max_stop_dis);

    const float dir_n = v_north / v;
    const float dir_e = v_east  / v;
    
    _hold_point_neu = Vector3p{d * dir_n, d * dir_e, 0.0f};

    // Shift spatial layout from local NED origin offset vectors onto local NEU coordinate frames
    Vector3p pos_ned_m;
    if (!copter.ahrs.get_relative_position_NED_origin(pos_ned_m)) {
        ModeGuided::init(true);
        return;
    }
    _hold_point_neu.xy() += pos_ned_m.xy();
    _hold_point_neu.z -= pos_ned_m.z;

    // Use ModeGuided structural tracking updates to clean submode states properly
    set_pos_NEU_m(_hold_point_neu, false, 0.0f, false, 0.0f, false, false);
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
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    const uint32_t now    = AP_HAL::millis();
    const uint32_t waited = now - _hold_point_start_ms;

    const bool target_locked = _target_px_valid && ((sq((float)_target_dx) + sq((float)_target_dy)) < 900.0f);

    if (target_locked) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: target locked (dx=%d dy=%d), target approach",
            (int)_target_dx, (int)_target_dy);
        if (calculate_target_location(_target_gps_loc)) {
            waypoint_control_start();
            wp_nav->set_wp_destination_loc(_target_gps_loc);
        }
        _phase = RescuePhase::TARGET_APPROACH;
        _target_approach_start_ms = now;
        return;
    }

    if (waited >= HOLD_POINT_WAIT_MS) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Rescue: hold timeout, resuming pattern at WP %u/%u",
            _current_idx + 1, _wp_count);
        _tracking_active = false;
        _phase = RescuePhase::WP_NAV;
        Location dest = _waypoints[_current_idx];
        apply_nav_alt(dest);
        wp_nav_set_destination_insert(dest);
    }
}

void ModeRescue::get_gimbal_angles()
{
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        mount->get_attitude_euler(0, gimbal_roll_rad, gimbal_pitch_rad, gimbal_yaw_rad);
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: gimbal pitch rad=%.3f", (double)gimbal_pitch_rad);
    }
}

// ---------------------------------------------------------------------------
// TARGET_Location
// ---------------------------------------------------------------------------
bool ModeRescue::calculate_target_location(Location &target_loc)
{
    float z = 0.0f;
    if (!copter.ahrs.get_location(target_loc)) {
        return false;
    }
    if (copter.current_loc.relative_alt) {
        z = ((copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f);
    }
    else {
        z = (copter.current_loc.alt * 0.01f);
    }

    if (copter.rangefinder_state.enabled &&
        copter.rangefinder_state.alt_healthy &&
        fabsf(copter.rangefinder_state.alt_m - g2.rescue.nav_alt) < 10.0f) {
        z = copter.rangefinder_state.alt_m;
    }
    
    get_gimbal_angles();

    if (fabsf(gimbal_pitch_rad) < 0.0174f) {
        gimbal_pitch_rad = (gimbal_pitch_rad >= 0.0f) ? 0.0174f : -0.0174f;
    }

    float distance = -z / tanf(gimbal_pitch_rad);

    Vector2p target_body;
    target_body.x = distance * cosf(gimbal_yaw_rad); 
    target_body.y = distance * sinf(gimbal_yaw_rad); 

    target_body = copter.ahrs.body_to_earth2D_p(target_body);
    target_loc.offset(target_body.x, target_body.y); 
    target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);

    return true;
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
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_U_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    const uint32_t now     = AP_HAL::millis();
    const uint32_t elapsed = now - _target_approach_start_ms;
    
    if (wp_nav->reached_wp_destination() && elapsed > 1500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: Target approach reached, waiting for centering");
    }

    if ((wp_nav->reached_wp_destination() && elapsed > 1500) || (elapsed >= TARGET_APPROACH_WAIT_MS)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: approach wait done, centering");
        _smooth_vx = 0.0f;
        _smooth_vy = 0.0f;
        posvelaccel_control_start();
        _hold_point_neu.zero();
        _target_vel_neu.zero();
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

    const uint32_t now = AP_HAL::millis();
    float vx = 0.0f;
    float vy = 0.0f;

    // 1. Calculate tracking velocities relative to image offsets
    if (_target_px_valid) {
        compute_centering_velocity(vx, vy);
    }

    // Direct vertical approach speed rate configuration
    const float vz = 0.3f;

    // 2. Map body frame targets to earth-frame NEU tracking vectors matching native packet handling
    _target_vel_neu = Vector3f{vx, vy, vz};
    _target_vel_neu.xy() = copter.ahrs.body_to_earth2D(_target_vel_neu.xy());

    // 3. Match ongoing position references to anchor the combined loop logic smoothly
    Vector3p current_pos_target = pos_control->get_pos_desired_NEU_m();
    _hold_point_neu.xy() = current_pos_target.xy();
    _hold_point_neu.z = current_pos_target.z;

    // 4. Update the controller parameters through the standard public API. 
    // This transitions the flight mode into SubMode::PosVelAccel cleanly.
    set_pos_vel_accel_NEU_m(_hold_point_neu, _target_vel_neu, Vector3f{0.0f, 0.0f, 0.0f}, false, 0.0f, false, 0.0f, false);

    // 5. Invoke ModeGuided's execution engine to update underlying PID loops and vector outputs natively
    ModeGuided::run();

    // 6. Handle payload/lifebuoy hardware triggers based on calculated relative height limits
    float rel_alt = 0.0f;
    if (copter.current_loc.relative_alt) {
        rel_alt = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;
    }
    else {
        rel_alt = copter.current_loc.alt * 0.01f;
    }

    if (rel_alt < g2.rescue.life_dep_alt && !_lifebuoy_deployed) {
        fire_lifebuoy_servos(1);
        _lifebuoy_deployed = true;
        _deploy_time_ms    = now;
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: lifebuoy deployed at rel_alt=%.1fm", (double)rel_alt);
        _phase = RescuePhase::DEPLOYING;
    }
}

// ---------------------------------------------------------------------------
// compute_centering_velocity
// ---------------------------------------------------------------------------
void ModeRescue::compute_centering_velocity(float &vx_out, float &vy_out)
{
    float rel_alt = 0.0f;
    if (copter.current_loc.relative_alt) {
        rel_alt = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;
    }
    else {
        rel_alt = copter.current_loc.alt * 0.01f;
    }
    const float altitude  = MAX(rel_alt, 3.0f);
    const float alt_scale = altitude / 30.0f;

    // const float raw_vx = -CENTER_PX_SCALE * (float)_target_dy * alt_scale;
    // const float raw_vy =  CENTER_PX_SCALE * (float)_target_dx * alt_scale;
    // Change signs to invert the tracking command vector
    const float raw_vx =  CENTER_PX_SCALE * (float)_target_dy * alt_scale; // Removed the negative sign
    const float raw_vy = -CENTER_PX_SCALE * (float)_target_dx * alt_scale; // Added a negative sign

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
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: failed to switch mode, entering GUIDED");
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
    int pwm = deploy ? g2.rescue.life_deploy_pwm : g2.rescue.life_retract_pwm;
    
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
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u/%u (%.1fm)", _current_idx + 1, _wp_count, (double)alt_m);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: all %u WPs done, dynamic landing", _wp_count);
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
void ModeRescue::handle_rescue_wp(uint16_t seq, uint16_t total_count, int32_t lat_degE7, int32_t lon_degE7)
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
}

void ModeRescue::handle_start_search()
{
    if (copter.flightmode != this) return;
    if (_phase != RescuePhase::IDLE && _phase != RescuePhase::WPS_GENERATED) return;
    if (_wp_count == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no WPs loaded");
        return;
    }
    if (!_beacon_valid || (AP_HAL::millis() - _beacon_last_ms > BEACON_TIMEOUT_MS)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rescue: no beacon GPS, cannot start");
        return;
    }

    _current_idx                = 0;
    _tracking_active            = false;
    _target_px_valid            = false;
    _detection_window_start_ms  = 0;
    _first_wp_reached           = false;
    _smooth_vx                  = 0.0f;
    _smooth_vy                  = 0.0f;
    _lifebuoy_deployed          = false;
    _wpnav_initialised          = false;
    _has_inserted_wp            = false;
    _mission_start_ms           = AP_HAL::millis();

    if (copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rescue: queued, taking off to %.1fm when armed", (double)(float)g2.rescue.nav_alt);
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
    gcs().send_text(MAV_SEVERITY_INFO, "Rescue: WP %u %s", wp_index+1, reached ? "reached" : "not reached");
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_user_wp_reached_send(gcs().chan(c)->get_chan(), wp_index, reached);
    }
}

void ModeRescue::handle_home_beacon_gps(int32_t lat, int32_t lon, float heading, float v_north, float v_east)
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
