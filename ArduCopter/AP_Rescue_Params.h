#pragma once

#include <AP_Param/AP_Param.h>

class AP_Rescue_Params
{
public:

    AP_Rescue_Params();

    static const struct AP_Param::GroupInfo var_info[];

    // Navigation
    AP_Int16 nav_alt;
    AP_Int16 dyn_tar_thr;
    AP_Float stop_acc;
    AP_Int16 max_stop_dis;
    AP_Int16 miss_timeout;

    // Round About
    AP_Int8 round_abt_or;
    AP_Int16 round_abt_orbit_rad;
    AP_Int16 round_abt_ang_spd_deg;
    AP_Int16 round_abt_init_thr;

    // Follow
    AP_Float flw_lkahd_fac;
    AP_Int16 flw_tar_alt;

    // Landing
    AP_Float alt_rest_cone_factor;
    AP_Int16 mar_time_thr;
    AP_Int16 mar_det_thr;
    AP_Float mot_cutoff_thr;

    // Gimbal
    AP_Float gmb_msg_rate;
    AP_Float gmb_hfov;
    AP_Float gmb_vfov;

    AP_Int16 gmb_cam_wid;
    AP_Int16 gmb_cam_hgt;

    AP_Int16 gmb_pit_poi;
    AP_Int16 gmb_yaw_poi;

    AP_Int16 gmb_pit_cor;
    AP_Int16 gmb_yaw_cor;

    // Detection
    AP_Int16 track_timeout;

    // Sensor
    AP_Int16 rad_msg_id;
    AP_Int16 lid_msg_id;

    AP_Int16 life_pwm_ch1;
    AP_Int16 life_pwm_ch2;
    AP_Int16 life_pwm_ch3;

    AP_Int16 life_deploy_pwm;
    AP_Int16 life_retract_pwm;

    AP_Float life_dep_alt;
    AP_Float overlap;
    AP_Int8 beacon_avl;
    AP_Float wind_mps;
    AP_Float act_dist_m;
    AP_Int8  wp_skip;
    AP_Float max_path_dist;
    
    };
