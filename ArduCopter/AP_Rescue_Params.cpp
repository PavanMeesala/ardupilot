#include "AP_Rescue_Params.h"
AP_Rescue_Params::AP_Rescue_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
    
}
const AP_Param::GroupInfo AP_Rescue_Params::var_info[] = {

    // @Param: WP_THR
    // @DisplayName: Waypoint Arrival Threshold
    // @Description: Distance from a waypoint at which the vehicle considers the waypoint reached during normal navigation.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WP_THR",1,AP_Rescue_Params,wp_thr,3),

    // @Param: WP_THR_END
    // @DisplayName: Endurance Waypoint Threshold
    // @Description: Distance from a waypoint at which the vehicle considers the waypoint reached during endurance operations.
    // @Units: m
    // @Range: 1 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WP_THR_END",2,AP_Rescue_Params,wp_thr_end,50),

    // @Param: DYN_TAR_THR
    // @DisplayName: Dynamic Target Threshold
    // @Description: Distance from the moving target at which the vehicle considers the target reached.
    // @Units: m
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DYN_TAR_THR",3,AP_Rescue_Params,dyn_tar_thr,4),

    // @Param: POS_MSG_RT
    // @DisplayName: Position Message Rate
    // @Description: Interval between position messages transmitted to the vehicle.
    // @Units: s
    // @Range: 0.01 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS_MSG_RT",4,AP_Rescue_Params,pos_msg_rate,0.1f),

    // @Param: VEL_MSG_RT
    // @DisplayName: Velocity Message Rate
    // @Description: Interval between velocity messages transmitted to the vehicle.
    // @Units: s
    // @Range: 0.01 5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("VEL_MSG_RT",5,AP_Rescue_Params,vel_msg_rate,0.033f),

    // @Param: STOP_ACC
    // @DisplayName: Stopping Acceleration
    // @Description: Deceleration applied while bringing the vehicle to a controlled stop.
    // @Units: m/s/s
    // @Range: 0.1 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("STOP_ACC",6,AP_Rescue_Params,stop_acc,1.2f),

    // @Param: MAX_STOP_DIS
    // @DisplayName: Maximum Stop Distance
    // @Description: Maximum distance ahead of the vehicle at which a temporary hold point may be created when the target is detected.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX_STOP_DIS",7,AP_Rescue_Params,max_stop_dis,15),

    // @Param: MISS_TIMEOUT
    // @DisplayName: Mission Timeout
    // @Description: Maximum allowed mission duration before the rescue operation is aborted.
    // @Units: s
    // @Range: 60 7200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MISS_TIMEOUT",8,AP_Rescue_Params,miss_timeout,1200),

    // @Param: RND_OR
    // @DisplayName: Round About Direction
    // @Description: Initial orbit direction used during the round-about maneuver.
    // @Values: 0:CW,1:CCW
    // @User: Advanced
    AP_GROUPINFO("RND_OR",9,AP_Rescue_Params,round_abt_or,0),

    // @Param: RND_RAD
    // @DisplayName: Round About Radius
    // @Description: Radius of the orbit used during the round-about maneuver.
    // @Units: m
    // @Range: 5 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_RAD",10,AP_Rescue_Params,round_abt_orbit_rad,30),

    // @Param: RND_SPD
    // @DisplayName: Round About Angular Speed
    // @Description: Angular speed of the round-about orbit.
    // @Units: deg/s
    // @Range: 1 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_SPD",11,AP_Rescue_Params,round_abt_ang_spd_deg,18),

    // @Param: RND_THR
    // @DisplayName: Round About Initiation Threshold
    // @Description: Distance from the target at which the vehicle initiates the round-about maneuver.
    // @Units: m
    // @Range: 1 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_THR",12,AP_Rescue_Params,round_abt_init_thr,100),

    // @Param: FLW_LKAHD
    // @DisplayName: Follow Lookahead Factor
    // @Description: Look-ahead factor used to predict target motion while following.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("FLW_LKAHD",13,AP_Rescue_Params,flw_lkahd_fac,0.28f),

    // @Param: FLW_ALT
    // @DisplayName: Follow Altitude
    // @Description: Altitude maintained above the target during follow operations.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLW_ALT",14,AP_Rescue_Params,flw_tar_alt,9),

    // @Param: ALT_CONE
    // @DisplayName: Altitude Restriction Cone Factor
    // @Description: Scaling factor applied to the landing altitude restriction cone.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ALT_CONE",15,AP_Rescue_Params,alt_rest_cone_factor,0.89f),

    // @Param: MAR_TIME
    // @DisplayName: Marker Timeout
    // @Description: Time that the marker may be lost before reverting to GPS-based following.
    // @Units: s
    // @Range: 0 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAR_TIME",16,AP_Rescue_Params,mar_time_thr,5),

    // @Param: MAR_DET
    // @DisplayName: Marker Detection Threshold
    // @Description: Minimum marker detections required per second to maintain marker tracking.
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAR_DET",17,AP_Rescue_Params,mar_det_thr,7),

    // @Param: MOT_CUT
    // @DisplayName: Motor Cutoff Threshold
    // @Description: Relative landing threshold used to determine when motors may be shut down after touchdown.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOT_CUT",18,AP_Rescue_Params,mot_cutoff_thr,0.5f),

    // @Param: LOG_INT
    // @DisplayName: Logging Interval
    // @Description: Time interval between rescue-mode log entries.
    // @Units: s
    // @Range: 0.01 10
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("LOG_INT",19,AP_Rescue_Params,log_int,0.05f),

    // @Param: GMB_RATE
    // @DisplayName: Gimbal Message Rate
    // @Description: Interval between gimbal control messages transmitted to the vehicle.
    // @Units: s
    // @Range: 0.01 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("GMB_RATE",20,AP_Rescue_Params,gmb_msg_rate,0.05f),

    // @Param: GMB_HFOV
    // @DisplayName: Gimbal Horizontal FOV
    // @Description: Horizontal field of view of the tracking camera used for target localization and gimbal control calculations.
    // @Units: deg
    // @Range: 1 180
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GMB_HFOV",21,AP_Rescue_Params,gmb_hfov,84.5f),

    // @Param: GMB_VFOV
    // @DisplayName: Gimbal Vertical FOV
    // @Description: Vertical field of view of the tracking camera used for target localization and gimbal control calculations.
    // @Units: deg
    // @Range: 1 180
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GMB_VFOV",22,AP_Rescue_Params,gmb_vfov,54.0f),

    // @Param: GMB_WID
    // @DisplayName: Camera Width
    // @Description: Width of the tracking camera image in noneels.
    // @Range: 1 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_WID",23,AP_Rescue_Params,gmb_cam_wid,1280),

    // @Param: GMB_HGT
    // @DisplayName: Camera Height
    // @Description: Height of the tracking camera image in noneels.
    // @Range: 1 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_HGT",24,AP_Rescue_Params,gmb_cam_hgt,720),

    // @Param: GMB_SMTH
    // @DisplayName: Gimbal Smoothing Factor
    // @Description: Smoothing factor applied to gimbal control commands. Higher values result in slower and smoother gimbal movement.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("GMB_SMTH",25,AP_Rescue_Params,gmb_cntr_smth,0.1f),

    // @Param: GMB_PIT_POI
    // @DisplayName: Gimbal Pitch Pointing Angle
    // @Description: Desired gimbal pitch angle used while tracking the target.
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_PIT_POI",26,AP_Rescue_Params,gmb_pit_poi,-70),

    // @Param: GMB_YAW_POI
    // @DisplayName: Gimbal Yaw Pointing Angle
    // @Description: Desired gimbal yaw angle used while tracking the target.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_YAW_POI",27,AP_Rescue_Params,gmb_yaw_poi,0),

    // @Param: GMB_PIT_COR
    // @DisplayName: Gimbal Pitch Correction
    // @Description: Pitch correction angle used to compensate for gimbal mounting offsets and calibration errors.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_PIT_COR",28,AP_Rescue_Params,gmb_pit_cor,-60),

    // @Param: GMB_YAW_COR
    // @DisplayName: Gimbal Yaw Correction
    // @Description: Yaw correction angle used to compensate for gimbal mounting offsets and calibration errors.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_YAW_COR",29,AP_Rescue_Params,gmb_yaw_cor,30),

    // @Param: DET_CNF
    // @DisplayName: Detection Confidence Threshold
    // @Description: Minimum confidence score required for an object detection to be considered valid.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DET_CNF",30,AP_Rescue_Params,det_cnf_thr,0.9f),

    // @Param: DET_WIN
    // @DisplayName: Detection Window Duration
    // @Description: Time window used to accumulate detections before making tracking decisions.
    // @Units: s
    // @Range: 1 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DET_WIN",31,AP_Rescue_Params,det_win_dur,3),

    // @Param: MIN_TRACK
    // @DisplayName: Minimum Tracking Detections
    // @Description: Minimum number of detections required within the detection window to maintain an active target track.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN_TRACK",32,AP_Rescue_Params,min_det_track,10),

    // @Param: MIN_TRACKI
    // @DisplayName: Minimum Tracking Initiation Detections
    // @Description: Minimum number of detections required within the detection window to initiate target tracking.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN_TRACKI",33,AP_Rescue_Params,min_det_track_int,5),

    // @Param: TRACK_TO
    // @DisplayName: Tracking Timeout
    // @Description: Maximum time allowed without receiving a valid detection before the target is considered lost.
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRACK_TO",34,AP_Rescue_Params,track_timeout,5),

    // @Param: RAD_ID
    // @DisplayName: Radar Message ID
    // @Description: MAVLink message identifier used for radar distance measurements.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAD_ID",35,AP_Rescue_Params,rad_msg_id,1),

    // @Param: LID_ID
    // @DisplayName: Lidar Message ID
    // @Description: MAVLink message identifier used for lidar distance measurements.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LID_ID",36,AP_Rescue_Params,lid_msg_id,0),

    // @Param: PWM_CH1
    // @DisplayName: Lifebuoy PWM Channel 1
    // @Description: First servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH1",37,AP_Rescue_Params,life_pwm_ch1,9),

    // @Param: PWM_CH2
    // @DisplayName: Lifebuoy PWM Channel 2
    // @Description: Second servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH2",38,AP_Rescue_Params,life_pwm_ch2,10),

    // @Param: PWM_CH3
    // @DisplayName: Lifebuoy PWM Channel 3
    // @Description: Third servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH3",39,AP_Rescue_Params,life_pwm_ch3,11),

    // @Param: PWM_VAL1
    // @DisplayName: Lifebuoy PWM Value 1
    // @Description: PWM value commanded on lifebuoy deployment channel 1 during deployment.
    // @Units: us
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_VAL1",40,AP_Rescue_Params,life_pwm_val1,1800),

    // @Param: PWM_VAL2
    // @DisplayName: Lifebuoy PWM Value 2
    // @Description: PWM value commanded on lifebuoy deployment channel 2 during deployment.
    // @Units: us
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_VAL2",41,AP_Rescue_Params,life_pwm_val2,1800),

    // @Param: PWM_VAL3
    // @DisplayName: Lifebuoy PWM Value 3
    // @Description: PWM value commanded on lifebuoy deployment channel 3 during deployment.
    // @Units: us
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_VAL3",42,AP_Rescue_Params,life_pwm_val3,1700),

    // @Param: LIFE_ALT
    // @DisplayName: Lifebuoy Deployment Altitude
    // @Description: Altitude above the target at which the lifebuoy deployment sequence is initiated.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LIFE_ALT",43,AP_Rescue_Params,life_dep_alt,15),

    AP_GROUPEND
};
