#include "AP_Rescue_Params.h"
AP_Rescue_Params::AP_Rescue_Params()
{
    AP_Param::setup_object_defaults(this, var_info);
    
}
const AP_Param::GroupInfo AP_Rescue_Params::var_info[] = {

    // @Param: DYN_TAR_THR
    // @DisplayName: Dynamic Target Threshold
    // @Description: Distance from the moving target at which the vehicle considers the target reached.
    // @Units: m
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DYN_TAR_THR",1,AP_Rescue_Params,dyn_tar_thr,4),

    // @Param: STOP_ACC
    // @DisplayName: Stopping Acceleration
    // @Description: Deceleration applied while bringing the vehicle to a controlled stop.
    // @Units: m/s/s
    // @Range: 0.1 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("STOP_ACC",2,AP_Rescue_Params,stop_acc,1.2f),

    // @Param: MAX_STOP_DIS
    // @DisplayName: Maximum Stop Distance
    // @Description: Maximum distance ahead of the vehicle at which a temporary hold point may be created when the target is detected.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX_STOP_DIS",3,AP_Rescue_Params,max_stop_dis,15),

    // @Param: MISS_TIMEOUT
    // @DisplayName: Mission Timeout
    // @Description: Maximum allowed mission duration before the rescue operation is aborted.
    // @Units: s
    // @Range: 60 7200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MISS_TIMEOUT",4,AP_Rescue_Params,miss_timeout,1200),

    // @Param: RND_OR
    // @DisplayName: Round About Direction
    // @Description: Initial orbit direction used during the round-about maneuver.
    // @Values: 0:CW,1:CCW
    // @User: Advanced
    AP_GROUPINFO("RND_OR",5,AP_Rescue_Params,round_abt_or,0),

    // @Param: RND_RAD
    // @DisplayName: Round About Radius
    // @Description: Radius of the orbit used during the round-about maneuver.
    // @Units: m
    // @Range: 5 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_RAD",6,AP_Rescue_Params,round_abt_orbit_rad,30),

    // @Param: RND_SPD
    // @DisplayName: Round About Angular Speed
    // @Description: Angular speed of the round-about orbit.
    // @Units: deg/s
    // @Range: 1 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_SPD",7,AP_Rescue_Params,round_abt_ang_spd_deg,18),

    // @Param: RND_THR
    // @DisplayName: Round About Initiation Threshold
    // @Description: Distance from the target at which the vehicle initiates the round-about maneuver.
    // @Units: m
    // @Range: 1 500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RND_THR",8,AP_Rescue_Params,round_abt_init_thr,100),

    // @Param: FLW_LKAHD
    // @DisplayName: Follow Lookahead Factor
    // @Description: Look-ahead factor used to predict target motion while following.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("FLW_LKAHD",9,AP_Rescue_Params,flw_lkahd_fac,0.28f),

    // @Param: FLW_ALT
    // @DisplayName: Follow Altitude
    // @Description: Altitude maintained above the target during follow operations.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLW_ALT",10,AP_Rescue_Params,flw_tar_alt,9),

    // @Param: ALT_CONE
    // @DisplayName: Altitude Restriction Cone Factor
    // @Description: Scaling factor applied to the landing altitude restriction cone.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ALT_CONE",11,AP_Rescue_Params,alt_rest_cone_factor,0.89f),

    // @Param: MAR_TIME
    // @DisplayName: Marker Timeout
    // @Description: Time that the marker may be lost before reverting to GPS-based following.
    // @Units: s
    // @Range: 0 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAR_TIME",12,AP_Rescue_Params,mar_time_thr,5),

    // @Param: MAR_DET
    // @DisplayName: Marker Detection Threshold
    // @Description: Minimum marker detections required per second to maintain marker tracking.
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAR_DET",13,AP_Rescue_Params,mar_det_thr,7),

    // @Param: MOT_CUT
    // @DisplayName: Motor Cutoff Threshold
    // @Description: Relative landing threshold used to determine when motors may be shut down after touchdown.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MOT_CUT",14,AP_Rescue_Params,mot_cutoff_thr,0.5f),

    // @Param: GMB_RATE
    // @DisplayName: Gimbal Message Rate
    // @Description: Interval between gimbal control messages transmitted to the vehicle.
    // @Units: s
    // @Range: 0.01 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("GMB_RATE",15,AP_Rescue_Params,gmb_msg_rate,0.05f),

    // @Param: GMB_HFOV
    // @DisplayName: Gimbal Horizontal FOV
    // @Description: Horizontal field of view of the tracking camera used for target localization and gimbal control calculations.
    // @Units: deg
    // @Range: 1 180
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GMB_HFOV",16,AP_Rescue_Params,gmb_hfov,84.5f),

    // @Param: GMB_VFOV
    // @DisplayName: Gimbal Vertical FOV
    // @Description: Vertical field of view of the tracking camera used for target localization and gimbal control calculations.
    // @Units: deg
    // @Range: 1 180
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GMB_VFOV",17,AP_Rescue_Params,gmb_vfov,54.0f),

    // @Param: GMB_WID
    // @DisplayName: Camera Width
    // @Description: Width of the tracking camera image in noneels.
    // @Range: 1 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_WID",18,AP_Rescue_Params,gmb_cam_wid,640),

    // @Param: GMB_HGT
    // @DisplayName: Camera Height
    // @Description: Height of the tracking camera image in noneels.
    // @Range: 1 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_HGT",19,AP_Rescue_Params,gmb_cam_hgt,480),


    // @Param: GMB_PIT_POI
    // @DisplayName: Gimbal Pitch Pointing Angle
    // @Description: Desired gimbal pitch angle used while tracking the target.
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_PIT_POI",20,AP_Rescue_Params,gmb_pit_poi,-70),

    // @Param: GMB_YAW_POI
    // @DisplayName: Gimbal Yaw Pointing Angle
    // @Description: Desired gimbal yaw angle used while tracking the target.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_YAW_POI",21,AP_Rescue_Params,gmb_yaw_poi,0),

    // @Param: GMB_PIT_COR
    // @DisplayName: Gimbal Pitch Correction
    // @Description: Pitch correction angle used to compensate for gimbal mounting offsets and calibration errors.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_PIT_COR",22,AP_Rescue_Params,gmb_pit_cor,-60),

    // @Param: GMB_YAW_COR
    // @DisplayName: Gimbal Yaw Correction
    // @Description: Yaw correction angle used to compensate for gimbal mounting offsets and calibration errors.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_YAW_COR",23,AP_Rescue_Params,gmb_yaw_cor,30),


    // @Param: TRACK_TO
    // @DisplayName: Tracking Timeout
    // @Description: Maximum time allowed without receiving a valid detection before the target is considered lost.
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TRACK_TO",24,AP_Rescue_Params,track_timeout,5),

    // @Param: RAD_ID
    // @DisplayName: Radar Message ID
    // @Description: MAVLink message identifier used for radar distance measurements.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAD_ID",25,AP_Rescue_Params,rad_msg_id,1),

    // @Param: LID_ID
    // @DisplayName: Lidar Message ID
    // @Description: MAVLink message identifier used for lidar distance measurements.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LID_ID",26,AP_Rescue_Params,lid_msg_id,0),

    // @Param: PWM_CH1
    // @DisplayName: Lifebuoy PWM Channel 1
    // @Description: First servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH1",27,AP_Rescue_Params,life_pwm_ch1,9),

    // @Param: PWM_CH2
    // @DisplayName: Lifebuoy PWM Channel 2
    // @Description: Second servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH2",28,AP_Rescue_Params,life_pwm_ch2,10),

    // @Param: PWM_CH3
    // @DisplayName: Lifebuoy PWM Channel 3
    // @Description: Third servo output channel used for lifebuoy deployment control.
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PWM_CH3",29,AP_Rescue_Params,life_pwm_ch3,11),

    // @Param: DEP_PWM
    // @DisplayName: PWM value to deploy lifebuoy servos
    // @Description: PWM value commanded on lifebuoy deployment during deployment
    // @Units: us
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DEP_PWM",30,AP_Rescue_Params,life_deploy_pwm,1800),

    // @Param: RET_PWM
    // @DisplayName: PWM value to retract lifebuoy servos
    // @Description: PWM value commanded on lifebuoy deployment during retraction
    // @Units: us
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RET_PWM",31,AP_Rescue_Params,life_retract_pwm,1100),

    // @Param: LIFE_ALT
    // @DisplayName: Lifebuoy Deployment Altitude
    // @Description: Altitude above the target at which the lifebuoy deployment sequence is initiated.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LIFE_ALT",32,AP_Rescue_Params,life_dep_alt,15.0f),

    // @Param: NAV_ALT
    // @DisplayName: Navigation Altitude
    // @Description: Altitude above the home to which the drone will ascend during navigation.
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("NAV_ALT",33,AP_Rescue_Params,nav_alt,20.0f),

    // @Param: OVLP
    // @DisplayName: Path overlap factor
    // @Description: Overlap between adjacent search strips. 0=no overlap, 0.5=50% overlap
    // @Range: 0.0 0.9
    // @User: Standard
    AP_GROUPINFO("OVLP", 34, AP_Rescue_Params, overlap, 0.2f),

    // @Param: BEACON_AVL
    // @DisplayName: Beacon Available
    // @Description: Enable beacon GPS availability check before starting the rescue mission.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("BEACON_AVL", 35, AP_Rescue_Params, beacon_avl, 0),

    // @Param: WIND_MPS
    // @DisplayName: Rescue search pattern wind/drift factor
    // @Description: Controls how fast the lawnmower pattern half-width grows with distance from start. half_width_max = (total_dist/100) * 0.6 * WIND_MPS
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("WIND_MPS", 36, AP_Rescue_Params, wind_mps, 16.0f),

    // @Param: ACT_DIST
    // @DisplayName: Rescue pattern activation distance
    // @Description: Minimum distance to generate waypoints
    // @Units: m
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("ACT_DIST", 37, AP_Rescue_Params, act_dist_m, 0.0f),

    // @Param: WP_SKIP
    // @DisplayName: Number of Waypoints to be skipped
    // @Description: Waypoints within this distance of the first generated waypoint are dropped.
    // @Units: m
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("WP_SKIP", 38, AP_Rescue_Params, wp_skip, 0),

    // @Param: MAX_DIST
    // @DisplayName: Maximum path distance
    // @Description: Home history is logged to this distance with interval 5m.
    // @Units: m
    // @Range: 100 500
    // @User: Advanced
    AP_GROUPINFO("MAX_DIST", 39, AP_Rescue_Params, max_path_dist, 500.0f),
    
    AP_GROUPEND
};
