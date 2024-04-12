#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BLHeli/AP_BLHeli.h>

#include "AC_INDI_Control.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AC_INDI_Control::var_info[] = {
// @Param: _POSXY_P
    // @DisplayName: Position (horizontal) controller error P gain
    // @Description: Position (horizontal) controller error P gain. Converts the position error into acceleration command which is used in specific thrust increment
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_xy,      "_POSXY_",      1,  AC_INDI_Control, AC_P),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) controller error P gain
    // @Description: Velocity (horizontal) controller error P gain. Converts the velocity error into the linear acceleration command which is used in specific thrust increment
    // @Range: 1.000 3.000
    // @User: Standard    
    AP_SUBGROUPINFO(_p_vel_xy,      "_VELXY_",      2,  AC_INDI_Control, AC_P),    

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller error P gain
    // @Description: Position (vertical) controller error P gain. Converts the altitude error into the linear  acceleration command which is used in specific thrust increment
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z,       "_POSZ_",       3,  AC_INDI_Control, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller error P gain
    // @Description: Velocity (vertical) controller error P gain. Converts the climb rate error into the linear acceleration command which is used in specific thrust increment
    // @Range: 1.000 3.000
    // @User: Standard    
    AP_SUBGROUPINFO(_p_vel_z,       "_VELZ_",       4,  AC_INDI_Control, AC_P),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the roll angle error into the angular acceleration command which is used in torque increment 
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_x,      "_ANG_RLL_",   5,  AC_INDI_Control, AC_P),

    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the pitch angle error into the angular acceleration command which is used in torque increment
    // @Range: 3.000 12.000
    // @User: Standard    
    AP_SUBGROUPINFO(_p_angle_y,      "_ANG_PIT_",   6,  AC_INDI_Control, AC_P),

    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the yaw angle error into the angular acceleration command which is used in torque increment
    // @Range: 3.000 12.000
    // @User: Standard        
    AP_SUBGROUPINFO(_p_angle_z,      "_ANG_YAW_",   7,  AC_INDI_Control, AC_P),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the body x axis angular velocity error into the angular acceleration command which is used in torque increment
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard
    AP_SUBGROUPINFO(_p_ang_rate_x,   "_RAT_RLL_",   8,  AC_INDI_Control, AC_P),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the body y axis angular velocity error into the angular acceleration command which is used in torque increment
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard    
    AP_SUBGROUPINFO(_p_ang_rate_y,   "_RAT_PIT_",   9, AC_INDI_Control, AC_P),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the body z axis angular velocity error into the angular acceleration command which is used in torque increment
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard  
    AP_SUBGROUPINFO(_p_ang_rate_z,   "_RAT_YAW_",   10, AC_INDI_Control, AC_P),
    
    // @Param: _MASS
    // @DisplayName: Vehicle mass in kg 
    // @Description: Total vehicle mass in kg
    // @Units: kg
    // @Range: 0.01 10
    // @User: Standard
    AP_GROUPINFO("_MASS",                           11, AC_INDI_Control, _mass_kg, 2.0f),

    // @Param: _MOI_XX
    // @DisplayName: Moment of inertia of vehicle  in x-X axis in kg.m² 
    // @Description: Moment of inertia of vehicle in body frame x-x axis in kg.m². Vehicle assumed to be symmetrical about its z axis. 
    // @Units: kg.m²
    // @Range: 0.001 1
    // @User: Standard
    AP_GROUPINFO("_MOI_XX",                         12, AC_INDI_Control, _moment_inertia_xx_kgm2, 0.004f),

    // @Param: _MOI_YY
    // @DisplayName: Moment of inertia of vehicle  in y-y axis in kg.m² 
    // @Description: Moment of inertia of vehicle in body frame y-y axis in kg.m². Vehicle assumed to be symmetrical about its z axis. 
    // @Units: kg.m²
    // @Range: 0.001 1
    // @User: Standard
    AP_GROUPINFO("_MOI_YY",                         13, AC_INDI_Control, _moment_inertia_yy_kgm2, 0.004f),

    // @Param: _MOI_Z
    // @DisplayName: Moment of inertia of vehicle in z axis in kg.m² 
    // @Description: Moment of inertia of vehicle in body frame z axis in kg.m².
    // @Units: kg.m²
    // @Range: 0.001 1
    // @User: Standard
    AP_GROUPINFO("_MOI_Z",                          14, AC_INDI_Control, _moment_inertia_z_kgm2, 0.008f),

    // @Param: _ARM_LEN
    // @DisplayName: Distance to motor in m
    // @Description: Distance between center of mass and motor or arm length
    // @Units: m
    // @Range: 0.01 1
    // @User: Standard
    AP_GROUPINFO("_ARM_LEN",                      15, AC_INDI_Control, _arm_length_m, 0.25f),

    // @Param: _THR_COEF
    // @DisplayName: Thrust coefficent in N/(rad/s)²
    // @Description: Thrust coefficent defines relation between square of motor speed and force generated by motor. This value can be obtained by static thrust test. 
    // @Units: N/(rad/s)²
    // @Range: 0.000001 0.0001
    // @User: Standard
    AP_GROUPINFO("_THR_COEF",                       16, AC_INDI_Control, _thrust_coefficient, 1.489e-6f),

    // @Param: _TRQ_COEF
    // @DisplayName: Torque coefficent in Nm/(rad/s)²
    // @Description: Torque coefficent defines relation between square of motor speed and torque generated by motor.This value can be obtained by static thrust test. 
    // @Units: N/(rad/s)²
    // @Range: 0.000001 0.0001
    // @User: Standard
    AP_GROUPINFO("_TRQ_COEF",                       17, AC_INDI_Control, _torque_coefficient, 1.523e-8f),

    // @Param: _THR2RTRSPD
    // @DisplayName: Throttle command to motor speed coefficient
    // @Description: Defines relation from scaled throttle command(between 0-1) to motor speed in rad/s. Relation between throttle command and motor speed assumed to be linear.
    // @Range: 100 4000
    // @User: Standard
    AP_GROUPINFO("_THR2RTRSPD",                     18, AC_INDI_Control, _throttle2motor_speed, 3251.0f),

    // @Param: _TRQEST_FILT
    // @DisplayName: Torque estimate cutoff frequency in Hz
    // @Description: First order low pass filter applied to the torque estimation
    // @Units: Hz
    // @Range: 20 100
    // @User: Standard
    AP_GROUPINFO("_TRQEST_FILT",                     19, AC_INDI_Control, _torque_est_filter_cutoff, 35.0f),

    // @Param: _STHEST_FILT
    // @DisplayName: Specific thrust cutoff frequency in Hz
    // @Description: Second order low pass filter applied to the specific thrust estimation
    // @Units: Hz
    // @Range: 10 50
    // @User: Standard
    AP_GROUPINFO("_STHEST_FILT",                     20, AC_INDI_Control, _spec_thrust_est_filter_cutoff, 20.0f),

    // @Param: _STHRST_FILT
    // @DisplayName: Specific thrust command filter
    // @Description: First order low pass filter applied to the specific thrust command. This is necessary to reduce motor heat up due to noisy acceleration measurment.
    // @Units: Hz
    // @Range: 2 20
    // @User: Standard    
    AP_GROUPINFO("_STHRST_FILT",                    21, AC_INDI_Control, _spec_thrust_cmd_filter_cutoff, 3.0f),

    // @Param: _YAW_FILT
    // @DisplayName: Toruqe command z axis filter
    // @Description: First order low pass filter applied to the z axis of toruqe command. This is compansate for the mechanical D-term of yaw axis. 
    // @Units: Hz
    // @Range: 2 20
    // @User: Standard    
    AP_GROUPINFO("_YAW_FILT",                     22, AC_INDI_Control, _yaw_rate_filter_cutoff, 35.0f),

    // @Param: _ACC_SEL
    // @DisplayName: Selection of the angular acceleration
    // @Description: 1: gyro - diff - filt - ang_acc / 2: gyro - filt - diff - ang_acc / 3: no filter / 4 : z transform
    // @Units: Hz
    // @Range: 2 20
    // @User: Standard    
    AP_GROUPINFO("_ACC_SEL",                     23, AC_INDI_Control, _ang_acc_sel, 1),
    
    AP_GROUPEND
};

AC_INDI_Control::AC_INDI_Control(AP_AHRS_View& ahrs, const AP_InertialNav& inav) :
// AC_INDI_Control::AC_INDI_Control(AP_AHRS_View& ahrs, const AP_InertialNav& inav, AP_MW_AHRS& mwmw) :
    _ahrs(ahrs),
    _inav(inav),
    _p_pos_xy(1),
    _p_vel_xy(4),
    _p_pos_z(1),
    _p_vel_z(4),
    _p_angle_x(7.5),
    _p_ang_rate_x(65),
    _p_angle_y(7.5),
    _p_ang_rate_y(65),
    _p_angle_z(8),
    _p_ang_rate_z(80)
{
    if (_singleton) {
        AP_HAL::panic("Too many AC_INDI_Control instances");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// INDI
void AC_INDI_Control::set_use_INDI(bool use_INDI)
{
    _use_INDI = use_INDI;
}


// run position and velocity controller in xy axes 
void AC_INDI_Control::run_pos_vel_xy_controller(Vector3f target_point_m, Vector3f meas_dist_m, Vector3f vel_des_mps, Vector3f meas_vel_mps, Vector3f accel_des_mpss)
{
    Vector3f error_pos;

    _pos_target_neu_m.x = target_point_m.x;
    _pos_target_neu_m.y = target_point_m.y;

    error_pos.x = _pos_target_neu_m.x - meas_dist_m.x;
    error_pos.y = _pos_target_neu_m.y - meas_dist_m.y;

    _vel_target_neu_mps.x = _p_pos_xy.get_p(error_pos.x) + vel_des_mps.x;
    _vel_target_neu_mps.y = _p_pos_xy.get_p(error_pos.y) + vel_des_mps.y;

    _lin_acc_desired_ned_mpss.x = accel_des_mpss.x;
    _lin_acc_desired_ned_mpss.y = accel_des_mpss.y;

    _lin_acc_target_ned_mpss.x = _p_vel_xy.get_p(_vel_target_neu_mps.x - meas_vel_mps.x) + _lin_acc_desired_ned_mpss.x;
    _lin_acc_target_ned_mpss.y = _p_vel_xy.get_p(_vel_target_neu_mps.y - meas_vel_mps.y) + _lin_acc_desired_ned_mpss.y;
}

// run position and velocity controller in z axis
void AC_INDI_Control::run_pos_vel_z_controller(float target_alt_m, float meas_alt_m, float vel_des_mps, float meas_clmb_rate_mps, float accel_des_mpss, bool enable_xy, float accel_max_xy_mpss)
{
    float error_pos_z;

    _pos_target_neu_m.z = target_alt_m;
    error_pos_z = _pos_target_neu_m.z - meas_alt_m;

    _vel_target_neu_mps.z = _p_pos_z.get_p(error_pos_z) + vel_des_mps;
    _lin_acc_desired_ned_mpss.z = - accel_des_mpss;
    _lin_acc_target_ned_mpss.z = -_p_vel_z.get_p(_vel_target_neu_mps.z - meas_clmb_rate_mps) + _lin_acc_desired_ned_mpss.z;

    indi_linear_accel(enable_xy, accel_max_xy_mpss);
    spec_thrust_to_scaled_thrust();
}

/* 
    specific thrust increment based on acceleration difference
    conceptually this function behaves like an integrator
*/
void AC_INDI_Control::indi_linear_accel(bool enable_xy, float accel_max_xy_mpss)
{
    const Vector3f acc_flt = _ahrs.get_accel_ef_blended() + Vector3f(0, 0, GRAVITY_MSS);
    _spec_thrust_cmd_ned_mpss = _spec_thrust_est_ned_mpss + _lin_acc_target_ned_mpss - acc_flt;

    // do not reject disturbance in xy axis if position controller is not active. 
    if (!enable_xy) {
        _spec_thrust_cmd_ned_mpss.x = 0.0f;
        _spec_thrust_cmd_ned_mpss.y = 0.0f;
    }

    limit_specific_thrust(accel_max_xy_mpss);

    _spec_thrust_cmd_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), _spec_thrust_cmd_filter_cutoff);
    _spec_thrust_cmd_ned_mpss = _spec_thrust_cmd_filter.apply(_spec_thrust_cmd_ned_mpss);
}

// limit specific thrust command 
// TODO: fix vehicle climb during small lean angle command
// Attention: this can lead to fly away during continuous high yaw rate maneuver, ie circle mode
void AC_INDI_Control::limit_specific_thrust(float accel_max_xy)
{
    float spec_thrust_max;
    spec_thrust_max = sq(_throttle2motor_speed) * _thrust_coefficient * 4.0f / _mass_kg;

    // do not allow z axis acceleration to be bigger than zero
    // z axis of specific thrust should always be negative in NED frame
    _spec_thrust_cmd_ned_mpss.z = MIN(_spec_thrust_cmd_ned_mpss.z, -0.001f);
    
    // adjust maximum xy accel based on z accel
    accel_max_xy = MIN(accel_max_xy, fabsf(_spec_thrust_cmd_ned_mpss.z)*accel_max_xy/GRAVITY_MSS);

    // scale xy axis command to maximum limit
    float sp_th_xy_len = norm(_spec_thrust_cmd_ned_mpss.x, _spec_thrust_cmd_ned_mpss.y);    
    if ((sp_th_xy_len > accel_max_xy) && is_positive(sp_th_xy_len)) {
        _spec_thrust_cmd_ned_mpss.x *= (accel_max_xy / sp_th_xy_len);
        _spec_thrust_cmd_ned_mpss.y *= (accel_max_xy / sp_th_xy_len);
    }

    // do not allow magnitude of commanded specific thrust to be greater than maximum specific thrust
    float sp_th_xyz_len = _spec_thrust_cmd_ned_mpss.length();
    if ((sp_th_xyz_len > spec_thrust_max) && is_positive(sp_th_xyz_len)) {
        _spec_thrust_cmd_ned_mpss *= spec_thrust_max / sp_th_xyz_len;
    }
}

// convert specific thrust command to the total thrust 
// TODO: apply scaling based on throttle max-min limit from motor library
void AC_INDI_Control::spec_thrust_to_scaled_thrust(void)
{
    float total_thrust_max;
    total_thrust_max = sq(_throttle2motor_speed) * _thrust_coefficient * 4.0f;

    _total_thrust_cmd_body_N = - _spec_thrust_cmd_ned_mpss.length() * _mass_kg;

    // can not generate more than maximum thurst, clip it
    _total_thrust_cmd_body_N = MAX(_total_thrust_cmd_body_N, -total_thrust_max);
    _total_thrust_cmd_scaled = - _total_thrust_cmd_body_N / total_thrust_max;
}

/*
    Construct target attitude using specific thrust command and target yaw(heading) angle 
    TODO: yaw should be reference not measurment
*/
Quaternion AC_INDI_Control::input_acc_des_euler_angle_yaw(float yaw_rad)
{
    Vector3f e_cur_z, e_des_x, e_des_y, e_des_z, e_b_y, temp, _rot_vector;
    Quaternion att_target_quat;

    // construct a vector that lies in xy axis of inertial frame
    e_b_y = Vector3f(-sinf(yaw_rad), cosf(yaw_rad), 0);

    e_des_z = - _spec_thrust_cmd_ned_mpss.normalized();

    // take cross product from intermediate vector to target z axis 
    temp = e_b_y % e_des_z;
    e_des_x = temp.normalized();

    // take cross product from target z axis to target x axis
    temp = e_des_z % e_des_x;
    e_des_y = temp.normalized();

    att_target_quat.from_rotation_matrix(Matrix3f(e_des_x, e_des_y, e_des_z).transposed());
    att_target_quat.normalize();

    return att_target_quat;
}

/*
    Calculate attitude error using two ordered rotation. First calculate error between 
    z axis of current and target rotation matrix. Second correct for the yaw error.
    Same as thrust_heading_rotation_angles function in AC_AttitudeControl.cpp
*/
Vector3f AC_INDI_Control::calculate_att_error(Quaternion target, Quaternion measurment)
{   
    Quaternion att_cur_quat;

    _att_target_quat = target;
    att_cur_quat = measurment;

    Vector3f e_cur_z, e_des_z;    

    // ---------------------------------------------------------------------   
    Matrix3f att_cur_matrix;
    att_cur_quat.rotation_matrix(att_cur_matrix);

    e_cur_z = att_cur_matrix.colz();

    // ---------------------------------------------------------------------
    Matrix3f att_target_rot_matrix;
    _att_target_quat.rotation_matrix(att_target_rot_matrix);
    e_des_z = att_target_rot_matrix.colz();

    // ---------------------------------------------------------------------

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_correction_vec_cross = e_cur_z % e_des_z;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    float thrust_correction_vec_dot = acosf(constrain_float(e_cur_z * e_des_z, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    float thrust_correction_vec_length = thrust_correction_vec_cross.length();
    if (is_zero(thrust_correction_vec_length) || is_zero(thrust_correction_vec_dot)) {
        thrust_correction_vec_cross = Vector3f(0, 0, 1);
        thrust_correction_vec_dot = 0.0f;
    } else {
        thrust_correction_vec_cross /= thrust_correction_vec_length;
    }

    Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat.from_axis_angle(thrust_correction_vec_cross, thrust_correction_vec_dot);

    // Rotate thrust_vec_correction_quat to the body frame
    thrust_vec_correction_quat = att_cur_quat.inverse() * thrust_vec_correction_quat * att_cur_quat;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    Quaternion yaw_vec_correction_quat = thrust_vec_correction_quat.inverse() * att_cur_quat.inverse() * _att_target_quat;    

    // calculate the angle error in x and y.
    Vector3f rotation, error_att;
    thrust_vec_correction_quat.to_axis_angle(rotation);
    error_att.x = rotation.x;
    error_att.y = rotation.y;

    // calculate the angle error in z (x and y should be zero here).
    yaw_vec_correction_quat.to_axis_angle(rotation);
    error_att.z = rotation.z;

    return error_att;
}

// run attitude controller
Vector3f AC_INDI_Control::run_attitude_controller(Quaternion target, Quaternion measurment)
{   
    _error_att_save = calculate_att_error(target, measurment);

    return Vector3f(_p_angle_x.get_p(_error_att_save.x), _p_angle_y.get_p(_error_att_save.y), _p_angle_z.get_p(_error_att_save.z));
}

// run angular velocity controller
void AC_INDI_Control::run_angvel_controller(Vector3f target, Vector3f measurment, Vector3f ang_acc_desired)
{
    Vector3f error_ang_vel;

    _ang_vel_target_radps = target;
    error_ang_vel = _ang_vel_target_radps - measurment;

    _error_ang_vel_save = error_ang_vel;

    _ang_acc_target_radpss.x = _p_ang_rate_x.get_p(error_ang_vel.x);
    _ang_acc_target_radpss.y = _p_ang_rate_y.get_p(error_ang_vel.y);
    _ang_acc_target_radpss.z = _p_ang_rate_z.get_p(error_ang_vel.z);

    _ang_acc_target_radpss += ang_acc_desired;

    z_transform_acc();
    indi_angular_accel();
    scale_torque_cmd();

    // Off-CG Sensor data 
    // Off_CG_arrange();

    NAP_cal();
}

// torque increment based on angular acceleration difference
// conceptually this function behaves like an integrator
void AC_INDI_Control::indi_angular_accel(void)
{
    switch (_ang_acc_sel)
    {
        // Vector3f ang_acc_;
    case 1:
        // angular acc from Inertial_sensor_class (deferential and filter)
         _ang_acc_ = _ahrs.get_ang_accel_latest();
        break;
    
    case 2:
        // angular acc from Inertial_sensor_class (first gyro filter and deferential)
         _ang_acc_ = _ahrs.get_ang_accel_gyro_f_latest();
        break;

    case 3:
        // angular acc from Inertial_sensor_class (no filter)
         _ang_acc_ = _ahrs.get_ang_accel_no_f_latest();
        break;

    case 4:
        // angular acc from z transform
         _ang_acc_ = get_ang_acc_z_transform();
        break;
    
    default:
        break;
    }

    _ang_acc_check_x = _ang_acc_.x;
    _ang_acc_check_y = _ang_acc_.y;
    _ang_acc_check_z = _ang_acc_.z;


    Matrix3f moment_of_inertia_xyz (
        _moment_inertia_xx_kgm2, 0.0f, 0.0f,
        0.0f, _moment_inertia_yy_kgm2, 0.0f,
        0.0f, 0.0f, _moment_inertia_z_kgm2
    );

    // Differ origin INID, I calculate the torque cmd
    _torque_cmd_body_Nm = _torque_est_body_Nm + moment_of_inertia_xyz * (_ang_acc_target_radpss - _ang_acc_);

    // mechanical yaw is not considered in arducopter current control allocation 
    // filter body z axis torque command to compansate for the mechanicaly yaw
    // this render rotor inertia information(_motor_moment_inertia_kgm2) unneccessary 
    
    _yaw_rate_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), _yaw_rate_filter_cutoff);
    _yaw_rate_filter.apply(_torque_cmd_body_Nm.z);
    _torque_cmd_body_Nm.z = _yaw_rate_filter.get();
}

// TODO: apply scaling to the torque command to correct scaling due to the arducopter control allocation
void AC_INDI_Control::scale_torque_cmd(void)
{
    float xy_scale = 2.0f / (4.0f * sq(_throttle2motor_speed) * _thrust_coefficient * _arm_length_m * HALF_SQRT_2);
    float z_scale = 2.0f / (4.0f * sq(_throttle2motor_speed) * _torque_coefficient );
    
    // ******
    xy_scale = 1.0f;
    z_scale = 1.0f;
    // z_scale *= 0.1f; 
    z_scale = 1.0f;  
    // ******

    _torque_cmd_scaled.x = _torque_cmd_body_Nm.x * xy_scale;
    _torque_cmd_scaled.y = _torque_cmd_body_Nm.y * xy_scale;
    _torque_cmd_scaled.z = _torque_cmd_body_Nm.z * z_scale;
}

void AC_INDI_Control::z_transform_acc(void)
{
    const Vector3f &ang_vel_z = _ahrs.get_gyro();

    // angular acceleration of z transform
    _p_dot_z_transform = 5.714f * _p_u_prev_1 - 5.714 * _p_u_prev_2 + 1.824 * _p_y_prev_1 - 0.838 * _p_y_prev_2;
    _p_u_prev_2 = _p_u_prev_1;
    _p_u_prev_1 = double(ang_vel_z.x);
    _p_y_prev_2 = _p_y_prev_1;
    _p_y_prev_1 = _p_dot_z_transform;

    _q_dot_z_transform = 5.714f * _q_u_prev_1 - 5.714 * _q_u_prev_2 + 1.824 * _q_y_prev_1 - 0.838 * _q_y_prev_2;
    _q_u_prev_2 = _q_u_prev_1;
    _q_u_prev_1 = double(ang_vel_z.y);
    _q_y_prev_2 = _q_y_prev_1;
    _q_y_prev_1 = _q_dot_z_transform;

    _r_dot_z_transform = 5.714f * _r_u_prev_1 - 5.714 * _r_u_prev_2 + 1.824 * _r_y_prev_1 - 0.838 * _r_y_prev_2;
    _r_u_prev_2 = _r_u_prev_1;
    _r_u_prev_1 = double(ang_vel_z.z);
    _r_y_prev_2 = _r_y_prev_1;
    _r_y_prev_1 = _r_dot_z_transform;

    _angular_acc_z.x = _p_dot_z_transform;
    _angular_acc_z.y = _q_dot_z_transform;
    _angular_acc_z.z = _r_dot_z_transform;
}

void AC_INDI_Control::Off_CG_arrange(void)
{
    // Num 1 : Right
    
}

void AC_INDI_Control::NAP_cal(void)
{

    mw_1_acc.y = AP_MW_AHRS::acc_mw_ahrs[0][0];;
    mw_1_acc.z = -mw_1_acc.y;
}

// NOT USED: Arducopter control allocation used at the moment.
// allocate torque and thrust commands to the motors 
// TODO: Tidy up matrix operation
void AC_INDI_Control::control_allocation(void)
{
    // pre calculate some common constant 
    float a = 1.0f / (_arm_length_m * HALF_SQRT_2);
    float b = _thrust_coefficient / _torque_coefficient;
    float c = 1.0f / _thrust_coefficient;
    float d = 1.0f / _throttle2motor_speed;

    // inverse of control allocation matrix for quad-x frame type
    // motor order start from top right and increase counter clockwise
    // defines relation between torque command and thrust command to the required thrust per motor
    // float inv_control_alloc_G1[4][4] = {
    //     {-a,  a,  b,  -1.0f},
    //     {-a, -a, -b,  -1.0f},
    //     { a, -a,  b,  -1.0f},
    //     { a,  a, -b,  -1.0f}};

    float inv_control_alloc_G1[4][4] = {
        {-a,  a,  b,  -1.0f},
        {a, -a, b,  -1.0f},
        { a, a,  -b,  -1.0f},
        {-a, -a, -b,  -1.0f}};

    // torque and thrust command from attitude and positon controller 
    float cmds[4] = {_torque_cmd_body_Nm.x, _torque_cmd_body_Nm.y, _torque_cmd_body_Nm.z,_total_thrust_cmd_body_N};

    // perform multiplication
    // can whole multiplication be performed in single step?
    for (uint8_t i=0; i < 4; i++) {
        float temp = 0.0f;
        for (uint8_t j=0; j < 4; j++) {
            temp += inv_control_alloc_G1[i][j] * cmds[j];
        }
        temp = fabsf(temp);
        // // dont allow negative motor command. 
        // // TODO: replace with tilt prioritized allocation 
        if (is_negative(temp)) {
            _motor_cmd_radps[i] = 0;
        } else {
            _motor_cmd_radps[i] = safe_sqrt(temp * 0.25f * c);
        }
        _motor_cmd_scaled[i] = _motor_cmd_radps[i] * d;
    }
}

// get current rotation speed of each motor in rad/s
void AC_INDI_Control::get_motor_speed(void) 
{
    const AP_RPM *rpm = AP_RPM::get_singleton();

    if (!rpm->get_rpm(0, _rpm_indi_1)) {
            // No valid RPM data
            _rpm_indi_1 = -10;
    }
    if (!rpm->get_rpm(1, _rpm_indi_2)) {
            // No valid RPM data
            _rpm_indi_2 = -10;
    }
    if (!rpm->get_rpm(2, _rpm_indi_3)) {
            // No valid RPM data
            _rpm_indi_3 = -10;
    }
    if (!rpm->get_rpm(3, _rpm_indi_4)) {
            // No valid RPM data
            _rpm_indi_4 = -10;
    }

    // motor_speed_rpm[0] = rpm_indi_1;
    // motor_speed_rpm[1] = rpm_indi_2;
    // motor_speed_rpm[2] = rpm_indi_3;
    // motor_speed_rpm[3] = rpm_indi_4;

    // M1 - RPM4
    _motor_speed_rpm[0] = _rpm_indi_4;
    // M2 - RPM3
    _motor_speed_rpm[1] = _rpm_indi_3;
    // M3 - RPM2
    _motor_speed_rpm[2] = _rpm_indi_2;
    // M4 - RPM1
    _motor_speed_rpm[3] = _rpm_indi_1;


// #ifdef HAVE_AP_BLHELI_SUPPORT
//     // get motor rotation speed telemetry
//     float esc_freq_hz[4] = {0.0f, 0.0f, 0.0f, 0.0f};    
//     AP_BLHeli::get_singleton()->get_motor_frequencies_hz(4, esc_freq_hz);

//     // map esc servo order to quad x frame order
//     float temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//     for (uint8_t i=0; i < 4; i++) {
//         uint8_t chan;
//         SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
//         SRV_Channels::find_channel(function, chan);
//         temp[i] = esc_freq_hz[chan];
//     }

//     // change quad x frame order so that motor order start from top right and increases counter clockwise
//     // TODO: add support for different frame type and configuration
//     motor_speed_hz[0] = temp[0];
//     motor_speed_hz[1] = temp[3];
//     motor_speed_hz[2] = temp[1];
//     motor_speed_hz[3] = temp[2];
// #endif
// #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//     float motor_speed_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//     AP::rpm()->get_rpms(0, motor_speed_rpm);
//     for (int i=0; i<4; i++) {
//         motor_speed_hz[i] = motor_speed_rpm[i] / 60.0f;
//     }
// #endif
 
    for (uint8_t i=0; i < 4; i++) {
        _motor_speed_hz[i]          = _motor_speed_rpm[i] * 0.016667;
        _motor_speed_meas_radps[i]  = _motor_speed_rpm[i] * 0.10472;
    }    
}

/* 
    Calculate estimate of torque and thrust produced by motor using
    the measured motor speed.  
    Motor speed derivative is not taken into account since it is not considered
    in control allocation.
    TODO: might require motor reordering in real hardware
*/ 
void AC_INDI_Control::calculate_torque_thrust_est(void)
{
    // read last rotation speed of motors
    get_motor_speed();

    // pre calculate some common constant 
    float l = _arm_length_m * HALF_SQRT_2;
    float k = _torque_coefficient / _thrust_coefficient;

    // control allocation matrix for quad-x frame type
    // motor order start from top right and increase counter clockwise
    // float control_alloc_G1[4][4] = {
    //     {   -l,    -l,     l,     l},
    //     {    l,    -l,    -l,     l},
    //     {    k,    -k,     k,    -k},
    //     {-1.0f, -1.0f, -1.0f, -1.0f}};

    float control_alloc_G1[4][4] = {
        {   -l,    l,     l,     -l},
        {    l,    -l,    l,     -l},
        {    k,    k,     -k,    -k},
        {-1.0f, -1.0f, -1.0f, -1.0f}};

    // calculate square of measured motor speed
    float motor_speed_meas_sq[4];
    for (uint8_t i=0; i < 4; i++) {
        motor_speed_meas_sq[i] = sq(_motor_speed_meas_radps[i]);
    }

    float cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // perform multiplication
    // can whole multiplication be performed in single step?
    for (uint8_t i=0; i < 4; i++) {
        for (uint8_t j=0; j < 4; j++) {
            cmd[i] += control_alloc_G1[i][j] * motor_speed_meas_sq[j];
       }
       cmd[i] *= _thrust_coefficient;
    }

    // filter torque estimate
    _torque_est_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), _torque_est_filter_cutoff);
    _torque_est_body_Nm = _torque_est_filter.apply(Vector3f(cmd[0], cmd[1], cmd[2]));
    
    // filter specific thrust estimate and rotate specific thrust vector from body to NED frame
    _spec_thrust_est_filter.set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(), _spec_thrust_est_filter_cutoff);
    _spec_thrust_est_ned_mpss = _ahrs.get_rotation_body_to_ned() * Vector3f(0, 0, _spec_thrust_est_filter.apply(cmd[3] / _mass_kg)) ;
}


void AC_INDI_Control::write_log(void)
{
    const Vector3f &pos_target = get_pos_target();
    const Vector3f &vel_target = get_vel_target();
    const Vector3f &position = _inav.get_position_neu_cm()*0.01f;
    const Vector3f &velocity = _inav.get_velocity_neu_cms()*0.01f;

// @LoggerMessage: IND1
// @Description: INDI controller
// @Field: TimeUS: Time since system startup
// @Field: TPX: Target position relative to origin, X-axis
// @Field: TPY: Target position relative to origin, Y-axis
// @Field: TPZ: Target position relative to origin, Z-axis
// @Field: PX: Position relative to origin, X-axis
// @Field: PY: Position relative to origin, Y-axis
// @Field: PZ: Position relative to origin, Z-axis
// @Field: TVX: Target velocity, X-axis
// @Field: TVY: Target velocity, Y-axis
// @Field: TVZ: Target velocity, Z-axis
// @Field: VX: Velocity, X-axis
// @Field: VY: Velocity, Y-axis
// @Field: VZ: Velocity, Z-axis
// @Field: TAX: Target acceleration, X-axis
// @Field: TAY: Target acceleration, Y-axis
// @Field: TAZ: Target acceleration, Z-axis

    AP::logger().Write("IND1",
                        "TimeUS,TPX,TPY,TPZ,PX,PY,PZ,TVX,TVY,TVZ,VX,VY,VZ",
                        "smmmmmmnnnnnn",
                        "F000000000000",
                        "Qffffffffffff",
                        AP_HAL::micros64(),
                        double(pos_target.x),
                        double(pos_target.y),
                        double(pos_target.z),
                        double(position.x),
                        double(position.y),
                        double(position.z),
                        double(vel_target.x),
                        double(vel_target.y),
                        double(vel_target.z),
                        double(velocity.x),
                        double(velocity.y),
                        double(velocity.z));

    const Vector3f &lin_accel_target = get_lin_accel_target();
    const Vector3f acc_flt = _ahrs.get_accel_ef_blended() + Vector3f(0, 0, GRAVITY_MSS);
    AP::logger().Write("IND2",
                        "TimeUS,TAX,TAY,TAZ,DAX,DAY,DAZ,AX,AY,AZ",
                        "sooooooooo",
                        "F000000000",
                        "Qfffffffff",
                        AP_HAL::micros64(),
                        double(lin_accel_target.x),
                        double(lin_accel_target.y),
                        double(lin_accel_target.z),
                        double(_lin_acc_desired_ned_mpss.x),
                        double(_lin_acc_desired_ned_mpss.y),
                        double(_lin_acc_desired_ned_mpss.z),
                        double(acc_flt.x),
                        double(acc_flt.y),
                        double(acc_flt.z));


    // log attitude controller  
    // const float &att_target = get_attitude_quad_target().to_euler();
    const Vector3f &ang_vel_target = get_ang_vel_target();
    const Vector3f &ang_acc_target = get_ang_acc_target();
    const Vector3f &ang_vel = _ahrs.get_gyro();

    AP::logger().Write("IND3",
                        "TimeUS,TPX,TPY,TPZ,PX,PY,PZ,TVX,TVY,TVZ,VX,VY,VZ",
                        "sddddddkkkkkk",
                        "F000000000000",
                        "Qffffffffffff",
                        AP_HAL::micros64(),
                        double(get_attitude_quad_target().get_euler_roll() * RAD_TO_DEG),
                        double(get_attitude_quad_target().get_euler_pitch() * RAD_TO_DEG),
                        double(get_attitude_quad_target().get_euler_yaw() * RAD_TO_DEG),
                        double(_ahrs.roll_sensor*0.01f),
                        double(_ahrs.pitch_sensor*0.01f),
                        double(wrap_360(_ahrs.yaw_sensor*0.01f)),
                        double(ang_vel_target.x) * RAD_TO_DEG,
                        double(ang_vel_target.y) * RAD_TO_DEG,
                        double(ang_vel_target.z) * RAD_TO_DEG,
                        double(ang_vel.x) * RAD_TO_DEG,
                        double(ang_vel.y) * RAD_TO_DEG,
                        double(ang_vel.z) * RAD_TO_DEG);

    AP::logger().Write("IND4",
                        "TimeUS,TAX,TAY,TAZ,DAX,DAY,DAZ,accx,accy,accz",
                        "seeeeeeeee",
                        "F000000000",
                        "Qfffffffff",
                        AP_HAL::micros64(),
                        double(ang_acc_target.x) * RAD_TO_DEG,
                        double(ang_acc_target.y) * RAD_TO_DEG,
                        double(ang_acc_target.z) * RAD_TO_DEG,
                        double(_ang_acc_desired_radpss.x) * RAD_TO_DEG,
                        double(_ang_acc_desired_radpss.y) * RAD_TO_DEG,
                        double(_ang_acc_desired_radpss.z) * RAD_TO_DEG,
                        double(_ang_acc_check_x) * RAD_TO_DEG,
                        double(_ang_acc_check_y) * RAD_TO_DEG,
                        double(_ang_acc_check_z) * RAD_TO_DEG);


    // log commanded and estimated specific thrust and torque values
    AP::logger().Write("IND5",
                    "TimeUS,STCX,STCY,STCZ,STEX,STEY,STEZ,TCX,TCY,TCZ,TEX,TEY,TEZ",
                    "soooooo------",
                    "F000000000000",
                    "Qffffffffffff",
                    AP_HAL::micros64(),
                    double(_spec_thrust_cmd_ned_mpss.x),
                    double(_spec_thrust_cmd_ned_mpss.y),
                    double(_spec_thrust_cmd_ned_mpss.z),
                    double(_spec_thrust_est_ned_mpss.x),
                    double(_spec_thrust_est_ned_mpss.y),
                    double(_spec_thrust_est_ned_mpss.z),
                    double(_torque_cmd_body_Nm.x),
                    double(_torque_cmd_body_Nm.y),
                    double(_torque_cmd_body_Nm.z),
                    double(_torque_est_body_Nm.x),
                    double(_torque_est_body_Nm.y),
                    double(_torque_est_body_Nm.z));

    // log scaled torque (to the mixer)
    AP::logger().Write("IND6",
                    "TimeUS,SCAX,SCAY,SCAZ",
                    "s---",
                    "F000",
                    "Qfff",
                    AP_HAL::micros64(),
                    _torque_cmd_scaled.x,
                    _torque_cmd_scaled.y,
                    _torque_cmd_scaled.z);

    // log rpm, rpm[rad/s], rpm[Hz]
    AP::logger().Write("IND7",
                    "TimeUS,rp1,rp2,rp3,rp4,rpr1,rpr2,rpr3,rpr4,rph1,rph2,rph3,rph4",
                    "s------------",
                    "F000000000000",
                    "Qffffffffffff",
                    AP_HAL::micros64(),
                    double(_motor_speed_rpm[0]),
                    double(_motor_speed_rpm[1]),
                    double(_motor_speed_rpm[2]),
                    double(_motor_speed_rpm[3]),
                    double(_motor_speed_meas_radps[0]),
                    double(_motor_speed_meas_radps[1]),
                    double(_motor_speed_meas_radps[2]),
                    double(_motor_speed_meas_radps[3]),
                    double(_motor_speed_hz[0]),
                    double(_motor_speed_hz[1]),
                    double(_motor_speed_hz[2]),
                    double(_motor_speed_hz[3]));

    // angular acc from Inertial_sensor_class
    const Vector3f ang_acc_flt     = _ahrs.get_ang_accel_latest();
    // angular acc from Inertial_sensor_class (first gyro filter and deferential)
    const Vector3f ang_acc_gyro_f  = _ahrs.get_ang_accel_gyro_f_latest();
    // angular acc from Inertial_sensor_class (no filter)
    const Vector3f ang_acc_no_f    = _ahrs.get_ang_accel_no_f_latest();
    // angular acc from z transform
    const Vector3f ang_acc_z_trans = get_ang_acc_z_transform();

    // angular acceleration of z transform
    AP::logger().Write("IND8",
                    "TimeUS,accx,accy,accz,acgx,acgy,acgz",
                    "seeeeee",
                    "F000000",
                    "Qffffff",
                    AP_HAL::micros64(),
                    double(ang_acc_flt.x) * RAD_TO_DEG,
                    double(ang_acc_flt.y) * RAD_TO_DEG,
                    double(ang_acc_flt.z) * RAD_TO_DEG,
                    double(ang_acc_gyro_f.x) * RAD_TO_DEG,
                    double(ang_acc_gyro_f.y) * RAD_TO_DEG,
                    double(ang_acc_gyro_f.z) * RAD_TO_DEG);

    AP::logger().Write("IND9",
                    "TimeUS,acnx,acny,acnz,aczx,aczy,aczz,actx,acty,actz",
                    "seeeeeeeee",
                    "F000000000",
                    "Qfffffffff",
                    AP_HAL::micros64(),
                    double(ang_acc_no_f.x) * RAD_TO_DEG,
                    double(ang_acc_no_f.y) * RAD_TO_DEG,
                    double(ang_acc_no_f.z) * RAD_TO_DEG,
                    double(ang_acc_z_trans.x) * RAD_TO_DEG,
                    double(ang_acc_z_trans.y) * RAD_TO_DEG,
                    double(ang_acc_z_trans.z) * RAD_TO_DEG,
                    double(_ang_acc_check_x) * RAD_TO_DEG,
                    double(_ang_acc_check_y) * RAD_TO_DEG,
                    double(_ang_acc_check_z) * RAD_TO_DEG);

    // const Vector3f &ang_err_log = get_ang_err();
    // AP::logger().Write("IN10",
    //                 "TimeUS,Euex,Euey,Euez,Avex,Avey,Avez",
    //                 "sdddkkk",
    //                 "F000000",
    //                 "Qffffff",
    //                 AP_HAL::micros64(),
    //                 double(ang_err_log.x) * RAD_TO_DEG,
    //                 double(ang_err_log.y) * RAD_TO_DEG,
    //                 double(ang_err_log.z) * RAD_TO_DEG,
    //                 double(_error_ang_vel_save.x) * RAD_TO_DEG,
    //                 double(_error_ang_vel_save.y) * RAD_TO_DEG,
    //                 double(_error_ang_vel_save.z) * RAD_TO_DEG);

    const Vector3f &offcg_mw = get_mw_1_acc();
    AP::logger().Write("IN11",
                "TimeUS,ax2,ax3",
                "s--",
                "F00",
                "Qff",
                AP_HAL::micros64(),
                offcg_mw.y,
                offcg_mw.z);
}

AC_INDI_Control *AC_INDI_Control::_singleton = nullptr;

/*
 * Get the AC_INDI_Control singleton
 */
AC_INDI_Control *AC_INDI_Control::get_singleton()
{
    return AC_INDI_Control::_singleton;
}

namespace AP {

AC_INDI_Control &indi_control()
{
    return *AC_INDI_Control::get_singleton();
}

};