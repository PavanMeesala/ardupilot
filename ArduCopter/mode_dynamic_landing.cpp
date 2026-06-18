#include "Copter.h"

#if MODE_DYNAMIC_LANDING_ENABLED

bool ModeDynamicLanding::init(bool ignore_checks)
{
    _phase                 = LandPhase::GPS_FOLLOW;
    _beacon.valid          = false;
    _marker.detected       = false;
    _smooth_home_vx        = 0.0f;
    _smooth_home_vy        = 0.0f;
    _marker.smooth_vx      = 0.0f;
    _marker.smooth_vy      = 0.0f;
    _marker.prev_smooth_vx = 0.0f;
    _marker.prev_smooth_vy = 0.0f;

    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "DynLand: init, following beacon GPS");
    return true;
}

void ModeDynamicLanding::run()
{
    send_status();

    switch (_phase) {
    case LandPhase::GPS_FOLLOW: gps_follow_run(); break;
    case LandPhase::CENTERING:  centering_run();  break;
    case LandPhase::DESCENDING: descending_run(); break;
    case LandPhase::LANDED:     ModeGuided::run(); break;
    }
}

// ---------------------------------------------------------------------------
// update_beacon_lookahead
// Uses: RSC_FLW_LKAHD (flw_lkahd_fac AP_Float)
// T = flw_lkahd_fac * speed + 1.0, clamped [1, 4]
// ---------------------------------------------------------------------------
void ModeDynamicLanding::update_beacon_lookahead()
{
    const float spd = sqrtf(_beacon.v_north * _beacon.v_north +
                             _beacon.v_east  * _beacon.v_east);

    // flw_lkahd_fac is AP_Float
    float T = (float)g2.rescue.flw_lkahd_fac * spd + 1.0f;
    T = constrain_float(T, 1.0f, 4.0f);

    const float R        = 6378137.0f;
    const float lat0_rad = radians(_beacon.lat);

    _beacon.future_lat = _beacon.lat +
                         (_beacon.v_north * T / R) * RAD_TO_DEG;
    _beacon.future_lon = _beacon.lon +
                         (_beacon.v_east * T / (R * cosf(lat0_rad))) * RAD_TO_DEG;
}

// ---------------------------------------------------------------------------
// GPS_FOLLOW
// Uses: RSC_FLW_ALT (flw_tar_alt AP_Int16), RSC_DYN_TAR_THR (dyn_tar_thr AP_Int16)
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
    // flw_tar_alt is AP_Int16 — cast to float
    future_dest.set_alt_m((float)(int16_t)g2.rescue.flw_tar_alt,
                           Location::AltFrame::ABOVE_HOME);

    set_destination(future_dest);
    ModeGuided::run();

    Location beacon_cur{};
    beacon_cur.lat = (int32_t)(_beacon.lat * 1e7f);
    beacon_cur.lng = (int32_t)(_beacon.lon * 1e7f);
    beacon_cur.alt = 0;
    const float dist = copter.current_loc.get_distance(beacon_cur);

    // dyn_tar_thr is AP_Int16 — cast to float
    if (dist < (float)(int16_t)g2.rescue.dyn_tar_thr && _marker.detected) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "DynLand: %.1fm from beacon, marker visible → CENTERING", (double)dist);
        _phase = LandPhase::CENTERING;
        velaccel_control_start();
    }
}

// ---------------------------------------------------------------------------
// compute_marker_velocity
// Uses: RSC_GMB_WID (gmb_cam_wid AP_Int16), RSC_GMB_HFOV (gmb_hfov AP_Float)
//       RSC_VEL_MSG_RT (vel_msg_rate AP_Float)
// PD: kp=0.60 kd=0.04, LPF alpha=0.8 — all from OBC code
// ---------------------------------------------------------------------------
void ModeDynamicLanding::compute_marker_velocity(float &vx_out, float &vy_out)
{
    // gmb_cam_wid is AP_Int16 — cast to float
    const float cam_w  = (float)(int16_t)g2.rescue.gmb_cam_wid;
    // gmb_hfov is AP_Float
    const float hfov_r = radians((float)g2.rescue.gmb_hfov);

    const float px_to_m = (cam_w > 0.0f && _marker.z > 0.0f)
                          ? (2.0f * _marker.z * tanf(hfov_r * 0.5f)) / cam_w
                          : 0.01f;

    const float mk_x_raw = -(float)_marker.dy * px_to_m;
    const float mk_y_raw =  (float)_marker.dx * px_to_m;

    _marker.prev_smooth_vx = _marker.smooth_vx;
    _marker.prev_smooth_vy = _marker.smooth_vy;

    _marker.smooth_vx = MARKER_ALPHA * mk_x_raw + (1.0f - MARKER_ALPHA) * _marker.smooth_vx;
    _marker.smooth_vy = MARKER_ALPHA * mk_y_raw + (1.0f - MARKER_ALPHA) * _marker.smooth_vy;

    // vel_msg_rate is AP_Float
    const float dt = MAX((float)g2.rescue.vel_msg_rate, 0.001f);

    const float mk_vx = 0.60f * _marker.smooth_vx +
                        0.04f * (_marker.smooth_vx - _marker.prev_smooth_vx) / dt;
    const float mk_vy = 0.60f * _marker.smooth_vy +
                        0.04f * (_marker.smooth_vy - _marker.prev_smooth_vy) / dt;

    vx_out = constrain_float(mk_vx, -5.0f, 5.0f);
    vy_out = constrain_float(mk_vy, -5.0f, 5.0f);
}

// ---------------------------------------------------------------------------
// CENTERING
// Uses: RSC_MAR_TIME (mar_time_thr AP_Int16)
// ---------------------------------------------------------------------------
void ModeDynamicLanding::centering_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        _phase = LandPhase::LANDED;
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now_ms = AP_HAL::millis();

    // mar_time_thr is AP_Int16 — multiply by 1000 for ms
    if (!_marker.detected &&
        (now_ms - _marker.last_detected_ms) >
        (uint32_t)((int16_t)g2.rescue.mar_time_thr * 1000)) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "DynLand: marker lost >%ds → GPS_FOLLOW",
            (int)(int16_t)g2.rescue.mar_time_thr);
        _phase = LandPhase::GPS_FOLLOW;
        return;
    }

    const float hdg_rad = radians(copter.ahrs.yaw_sensor * 0.01f);
    const float base_vx =  _beacon.v_north * cosf(hdg_rad) + _beacon.v_east * sinf(hdg_rad);
    const float base_vy = -_beacon.v_north * sinf(hdg_rad) + _beacon.v_east * cosf(hdg_rad);

    _smooth_home_vx = 0.5f * base_vx + 0.5f * _smooth_home_vx;
    _smooth_home_vy = 0.5f * base_vy + 0.5f * _smooth_home_vy;

    float vx = _smooth_home_vx;
    float vy = _smooth_home_vy;

    if (_marker.detected) {
        float mk_vx = 0.0f;
        float mk_vy = 0.0f;
        compute_marker_velocity(mk_vx, mk_vy);
        vx += mk_vx;
        vy += mk_vy;
    }

    const Vector3f vel_neu{
         vx * cosf(hdg_rad) - vy * sinf(hdg_rad),
         vx * sinf(hdg_rad) + vy * cosf(hdg_rad),
        -0.3f
    };
    set_vel_NEU_ms(vel_neu);
    ModeGuided::run();

    if (_marker.detected) {
        const float dist_px = sqrtf((float)(_marker.dx * _marker.dx +
                                             _marker.dy * _marker.dy));
        if (dist_px < 30.0f) {
            gcs().send_text(MAV_SEVERITY_INFO,
                "DynLand: centred (%.0fpx) → DESCENDING", (double)dist_px);
            _phase = LandPhase::DESCENDING;
        }
    }
}

// ---------------------------------------------------------------------------
// DESCENDING
// Uses: RSC_MAR_TIME (mar_time_thr AP_Int16), RSC_MOT_CUT (mot_cutoff_thr AP_Float)
// Rangefinder: copter.rangefinder_alt_ok() + rangefinder_state.alt_m_filt.get()
// ---------------------------------------------------------------------------
void ModeDynamicLanding::descending_run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        _phase = LandPhase::LANDED;
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now_ms = AP_HAL::millis();

    // mar_time_thr is AP_Int16
    if (!_marker.detected &&
        (now_ms - _marker.last_detected_ms) >
        (uint32_t)((int16_t)g2.rescue.mar_time_thr * 1000)) {
        gcs().send_text(MAV_SEVERITY_INFO, "DynLand: marker lost → CENTERING");
        _phase = LandPhase::CENTERING;
        return;
    }

    const float hdg_rad = radians(copter.ahrs.yaw_sensor * 0.01f);
    float vx = _smooth_home_vx;
    float vy = _smooth_home_vy;

    if (_marker.detected) {
        float mk_vx = 0.0f;
        float mk_vy = 0.0f;
        compute_marker_velocity(mk_vx, mk_vy);
        vx += mk_vx;
        vy += mk_vy;
    }

    const Vector3f vel_neu{
         vx * cosf(hdg_rad) - vy * sinf(hdg_rad),
         vx * sinf(hdg_rad) + vy * cosf(hdg_rad),
        -0.3f
    };
    set_vel_NEU_ms(vel_neu);
    ModeGuided::run();

    // Touchdown: use copter.rangefinder_alt_ok() — NOT rangefinder_state.healthy
    // mot_cutoff_thr is AP_Float
    if (copter.rangefinder_alt_ok() &&
        copter.rangefinder_state.alt_m_filt.get() < (float)g2.rescue.mot_cutoff_thr &&
        _marker.z < 0.7f) {
        gcs().send_text(MAV_SEVERITY_INFO,
            "DynLand: touchdown (lidar=%.2fm, mk_z=%.2fm) → disarming",
            (double)copter.rangefinder_state.alt_m_filt.get(),
            (double)_marker.z);
        _phase = LandPhase::LANDED;
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }
}

// ---------------------------------------------------------------------------
// MAVLink handlers
// ---------------------------------------------------------------------------
void ModeDynamicLanding::handle_home_beacon_gps(int32_t lat, int32_t lon,
                                                  float heading,
                                                  float v_north, float v_east)
{
    _beacon.lat     = lat     * 1e-7f;
    _beacon.lon     = lon     * 1e-7f;
    _beacon.heading = heading;
    _beacon.v_north = v_north;
    _beacon.v_east  = v_east;
    _beacon.last_ms = AP_HAL::millis();
    _beacon.valid   = true;
    update_beacon_lookahead();
}

void ModeDynamicLanding::handle_aruco_marker(int16_t dx, int16_t dy,
                                              float z, uint8_t detected)
{
    _marker.dx       = dx;
    _marker.dy       = dy;
    _marker.z        = z;
    _marker.detected = (detected == 1);
    if (_marker.detected) {
        _marker.last_detected_ms = AP_HAL::millis();
    }
}

// ---------------------------------------------------------------------------
// send_status — DYNAMIC_LANDING_STATUS (55009) at 1Hz
// ---------------------------------------------------------------------------
void ModeDynamicLanding::send_status()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_status_ms < STATUS_INTERVAL_MS) return;
    _last_status_ms = now;

    Location beacon_cur{};
    beacon_cur.lat = (int32_t)(_beacon.lat * 1e7f);
    beacon_cur.lng = (int32_t)(_beacon.lon * 1e7f);
    beacon_cur.alt = 0;
    const float dist    = _beacon.valid
                          ? copter.current_loc.get_distance(beacon_cur)
                          : 0.0f;
    const float rel_alt = (copter.current_loc.alt -
                            copter.ahrs.get_home().alt) * 0.01f;

    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        mavlink_msg_dynamic_landing_status_send(
            gcs().chan(c)->get_chan(),
            static_cast<uint8_t>(_phase),
            rel_alt,
            dist,
            _marker.detected ? 1 : 0
        );
    }
}

#endif // MODE_DYNAMIC_LANDING_ENABLED
