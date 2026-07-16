#include "Copter.h"

#if MODE_DYNAMIC_LANDING_ENABLED

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool ModeDynamicLanding::init(bool ignore_checks)
{
    _phase            = LandPhase::GPS_FOLLOW;
    _beacon.valid     = false;
    _marker.detected  = false;

    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "DynLand: GPS Foll on");
    return true;
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------
void ModeDynamicLanding::run()
{
    send_status();

    switch (_phase) {
    case LandPhase::GPS_FOLLOW: gps_follow_run();        break;
    case LandPhase::PRECISION:  precision_landing_run(); break;
    case LandPhase::LANDED:     ModeGuided::run();       break;
    }
}

// ---------------------------------------------------------------------------
// update_beacon_lookahead 
// ---------------------------------------------------------------------------
void ModeDynamicLanding::update_beacon_lookahead()
{
    const float spd = sqrtf(_beacon.v_north * _beacon.v_north +
                             _beacon.v_east  * _beacon.v_east);
    
    float T = (float)g2.rescue.flw_lkahd_fac * spd + 1.0f;
    T = constrain_float(T, 1.0f, 4.0f); 

    const float R        = 6378137.0f; 
    const float lat0_rad = radians(_beacon.lat);

    _beacon.future_lat = _beacon.lat + (_beacon.v_north * T / R) * RAD_TO_DEG; 
    _beacon.future_lon = _beacon.lon + (_beacon.v_east * T / (R * cosf(lat0_rad))) * RAD_TO_DEG; 
}

// ---------------------------------------------------------------------------
// GPS_FOLLOW
// ---------------------------------------------------------------------------
void ModeDynamicLanding::gps_follow_run()
{
    if (!_beacon.valid) {
        ModeGuided::run();
        return;
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Location future_dest{};
    future_dest.lat = (int32_t)(_beacon.future_lat * 1e7f);
    future_dest.lng = (int32_t)(_beacon.future_lon * 1e7f);
    future_dest.set_alt_m((float)(int16_t)g2.rescue.flw_tar_alt, Location::AltFrame::ABOVE_HOME); 

    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f; 
    bool yaw_ignore = true;
    bool yaw_rate_ignore = true;
    set_destination(future_dest, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
    ModeGuided::run();

    Location beacon_cur{};
    beacon_cur.lat = (int32_t)(_beacon.lat * 1e7f);
    beacon_cur.lng = (int32_t)(_beacon.lon * 1e7f);
    beacon_cur.alt = 0;
    const float dist = copter.current_loc.get_distance(beacon_cur);

    if (dist < (float)(int16_t)g2.rescue.dyn_tar_thr && _marker.detected) {
        gcs().send_text(MAV_SEVERITY_INFO, "DynLand: target reached, Vis Foll on"); 
        precision_landing_enter(); // Initialize state filters[cite: 7]
        _last_update_ms = AP_HAL::millis();
        _phase = LandPhase::PRECISION;
    }
}

// ---------------------------------------------------------------------------
// precision_landing_enter
// ---------------------------------------------------------------------------
void ModeDynamicLanding::precision_landing_enter()
{
    _smooth_marker_x = 0.0f;
    _smooth_marker_y = 0.0f;
    _smooth_marker_z = 0.0f;
    _prev_smooth_marker_x = 0.0f;
    _prev_smooth_marker_y = 0.0f;
    _prev_smooth_marker_z = 0.0f;
    _smooth_home_vx  = 0.0f;
    _smooth_home_vy  = 0.0f;
    _marker_last_detected_ms = AP_HAL::millis();
    _detection_window_head  = 0;
    _detection_window_count = 0;
    _last_vz_command = 0.0f;
}

// ---------------------------------------------------------------------------
// Sliding Time Window Trackers
// ---------------------------------------------------------------------------
void ModeDynamicLanding::push_detection_timestamp(uint32_t now_ms)
{
    _detection_window[_detection_window_head] = now_ms;
    _detection_window_head = (_detection_window_head + 1) % DETECTION_WINDOW_MAX;
    if (_detection_window_count < DETECTION_WINDOW_MAX) {
        _detection_window_count++;
    }
}

void ModeDynamicLanding::prune_detection_window(uint32_t now_ms)
{
    const uint32_t window_ms = (uint32_t)(DETECTION_WINDOW_S * 1000.0f); // 1.0 second duration 

    while (_detection_window_count > 0) {
        uint8_t oldest_idx = (_detection_window_head + DETECTION_WINDOW_MAX - _detection_window_count) % DETECTION_WINDOW_MAX;
        uint32_t oldest_ts = _detection_window[oldest_idx];

        if (now_ms - oldest_ts > window_ms) {
            _detection_window_count--;
        } else {
            break;
        }
    }
}

float ModeDynamicLanding::compute_detections_per_second(uint32_t now_ms)
{
    prune_detection_window(now_ms);
    return (float)_detection_window_count / DETECTION_WINDOW_S;
}

void ModeDynamicLanding::get_vel(float dt)
{
    const uint32_t now = AP_HAL::millis();
    const float detections_per_second = compute_detections_per_second(now);
    const float since_last_detect_s = (now - _marker_last_detected_ms) * 0.001f; 

    float vz_ned = 0.0f; 

    if (since_last_detect_s > (float)(int16_t)g2.rescue.mar_time_thr) {
        vz_ned = -0.2f; 
    } else if (_marker.detected) {
        const float distance_to_marker = sqrtf(_marker.x * _marker.x + _marker.y * _marker.y); 
        

        if (distance_to_marker <= _marker.z * 0.5f * (float)g2.rescue.alt_rest_cone_factor) {
            vz_ned = 0.3f; 
        } else if (distance_to_marker > _marker.z * 0.53f) {
            vz_ned = -0.2f; 
        } else {
            vz_ned = 0.0f; 
        }
        _last_vz_command = vz_ned; 
    } else if (detections_per_second > (float)(int16_t)g2.rescue.mar_det_thr) {
        vz_ned = _last_vz_command; 
    } else {
        vz_ned = 0.0f;
    }

    Vector2f home_vel = Vector2f{_beacon.v_north, _beacon.v_east};
    home_vel = copter.ahrs.earth_to_body2D(home_vel);

    _smooth_home_vx = 0.7f * home_vel.x + 0.3f * _smooth_home_vx; 
    _smooth_home_vy = 0.7f * home_vel.y + 0.3f * _smooth_home_vy; 

    float vx, vy;

    if (_marker.detected) {
        _prev_smooth_marker_x = _smooth_marker_x;
        _prev_smooth_marker_y = _smooth_marker_y;
        _prev_smooth_marker_z = _smooth_marker_z;

        _smooth_marker_x = 0.8f * _marker.x + 0.2f * _smooth_marker_x; 
        _smooth_marker_y = 0.8f * _marker.y + 0.2f * _smooth_marker_y; 
        _smooth_marker_z = 0.8f * _marker.z + 0.2f * _smooth_marker_z; 

        const float marker_vx = 0.6f * _smooth_marker_x + 0.04f * (_smooth_marker_x - _prev_smooth_marker_x) / dt; 
        const float marker_vy = 0.6f * _smooth_marker_y + 0.04f * (_smooth_marker_y - _prev_smooth_marker_y) / dt; 

        const float clipped_marker_vx = constrain_float(marker_vx, -5.0f, 5.0f); 
        const float clipped_marker_vy = constrain_float(marker_vy, -5.0f, 5.0f); 

        vx = _smooth_home_vx + clipped_marker_vx; 
        vy = _smooth_home_vy + clipped_marker_vy; 
        // gcs().send_text(MAV_SEVERITY_INFO,"Marker Vel x=%.2f y=%.2f",(double)clipped_marker_vx,(double)clipped_marker_vy);
    } else {
        vx = _smooth_home_vx;
        vy = _smooth_home_vy;
    }

    // Pack into Front-Right-Up (Python uses body x, body y, vz command)
    vel_neu = Vector3f{vx, vy, -vz_ned}; 
    
    // Rotate 2D horizontal velocities back to Earth Frame for position controllers
    vel_neu.xy() = copter.ahrs.body_to_earth2D(vel_neu.xy()); 
}
// ---------------------------------------------------------------------------
// precision_landing_run — Single unified runtime tracking state engine
// ---------------------------------------------------------------------------
void ModeDynamicLanding::precision_landing_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        _phase = LandPhase::LANDED;
        return;
    }

    Location beacon_cur{};
    beacon_cur.lat = (int32_t)(_beacon.lat * 1e7f);
    beacon_cur.lng = (int32_t)(_beacon.lon * 1e7f);
    beacon_cur.alt = 0;
    const float distance_to_target = copter.current_loc.get_distance(beacon_cur);

    if (distance_to_target > (float)(int16_t)g2.rescue.dyn_tar_thr + 5.0f) {
        gcs().send_text(MAV_SEVERITY_INFO, "DynLand: drifted too far (%.1fm), GPS Foll on", (double)distance_to_target); 
        _phase = LandPhase::GPS_FOLLOW;
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    float rel_alt = 0.0f;
    if (copter.current_loc.relative_alt) {
        rel_alt = copter.current_loc.alt * 0.01f;
    } else {
        rel_alt = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;
    }
    if ((_marker.detected && _marker.z < 0.5f) || rel_alt < 1.2f) {
        gcs().send_text(MAV_SEVERITY_INFO, "DynLand: SITL touchdown (marker_z=%.2f)", (double)_marker.z); 
        _phase = LandPhase::LANDED;
        copter.arming.disarm(AP_Arming::Method::LANDED);
        return;
    }
#endif

    
    bool lidar_threshold_reached = false;
    if (copter.rangefinder_alt_ok()) {
        lidar_threshold_reached = copter.rangefinder_state.alt_m_filt.get() < (float)g2.rescue.mot_cutoff_thr; 
    }

    if (lidar_threshold_reached && _marker.detected && _marker.z < 0.7f) {
        gcs().send_text(MAV_SEVERITY_INFO, "DynLand: lidar threshold reached"); 
        _phase = LandPhase::LANDED;
        copter.arming.disarm(AP_Arming::Method::LANDED); 
        return;
    }

    const uint32_t now = AP_HAL::millis();
    uint32_t delta_ms = now - _last_update_ms;


    if (delta_ms >= 33) {
        _last_update_ms = now;
        float dt = (float)delta_ms * 0.001f;

        get_vel(dt);
    }
    _accel_cmd.zero();
    
    set_vel_accel_NEU_m(vel_neu, _accel_cmd, true, 0.0f, false, 0.0f, true, false);
    ModeGuided::run();
    
}

// ---------------------------------------------------------------------------
// MAVLink telemetry handlers
// ---------------------------------------------------------------------------
void ModeDynamicLanding::handle_home_beacon_gps(int32_t lat, int32_t lon,
                                                  float heading,
                                                  float v_north, float v_east)
{
    _beacon.lat     = lat * 1e-7f;
    _beacon.lon     = lon * 1e-7f;
    _beacon.heading = heading;
    _beacon.v_north = v_north;
    _beacon.v_east  = v_east;
    _beacon.last_ms = AP_HAL::millis();
    _beacon.valid   = true;
    update_beacon_lookahead();
}

void ModeDynamicLanding::handle_aruco_marker(float x_m, float y_m,
                                              float z_m, uint8_t detected)
{
    const uint32_t now = AP_HAL::millis();
    
    if (detected == 1) {
        _marker.x        = x_m;
        _marker.y        = y_m;
        _marker.z        = z_m;
        _marker.detected = true;
        _marker_last_detected_ms = now;
        // gcs().send_text(MAV_SEVERITY_INFO," Marker Position x=%.2f y=%.2f z=%.2f",(double)_marker.x,(double)_marker.y,(double)_marker.z);
        
        push_detection_timestamp(now);
    } else {
        const float since_last_detect_s = (now - _marker_last_detected_ms) * 0.001f;
        if (since_last_detect_s > (float)(int16_t)g2.rescue.mar_time_thr) {
            _marker.detected = false; 
        }
    }
}

void ModeDynamicLanding::send_status()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_status_ms < STATUS_INTERVAL_MS) return;
    _last_status_ms = now;

    Location beacon_cur{};
    beacon_cur.lat = (int32_t)(_beacon.lat * 1e7f);
    beacon_cur.lng = (int32_t)(_beacon.lon * 1e7f);
    const float dist = _beacon.valid ? copter.current_loc.get_distance(beacon_cur) : 0.0f;
    const float rel_alt = (copter.current_loc.alt - copter.ahrs.get_home().alt) * 0.01f;

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_dynamic_landing_status_send(gcs().chan(c)->get_chan(),
            static_cast<uint8_t>(_phase), rel_alt, dist,
            _marker.detected ? 1 : 0); 
    }
}

#endif // MODE_DYNAMIC_LANDING_ENABLED
