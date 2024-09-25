#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_PID/AC_P.h>               // P library
#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>

#include <SRV_Channel/SRV_Channel.h>

#include <AP_MW_AHRS/AP_MW_AHRS.h>

// rpm hall sensor
#include <AP_RPM/AP_RPM.h>

class AC_INDI_Control
{
public:
    /// Constructor
    AC_INDI_Control(AP_AHRS_View& ahrs, const AP_InertialNav& inav);
    // AC_INDI_Control(AP_AHRS_View& ahrs, const AP_InertialNav& inav, AP_MW_AHRS& mwmw);

    /* Do not allow copies */
    AC_INDI_Control(const AC_INDI_Control &other) = delete;
    AC_INDI_Control &operator=(const AC_INDI_Control&) = delete;

    static AC_INDI_Control *get_singleton();

    // bool enabled(void) const;

    void write_log(void);

    // run position controller for xy axis in inertial frame
    // accept linear accleration feedforward in inertial frame
    void run_pos_vel_xy_controller(Vector3f target_point_m, Vector3f meas_dist_m, Vector3f vel_des_mps, Vector3f meas_vel_mps, Vector3f accel_des_mpss);

    // run position controller for z axis in inertial frame
    // altitude is above home in unit m 
    // accept linear accleration feedforward in inertial frame
    void run_pos_vel_z_controller(float target_alt_m, float meas_alt_m, float vel_des_mps, float meas_clmb_rate_mps, float accel_des_mpss, bool enable_xy, float accel_max_xy_mpss);

    // run attitude controller 
    Vector3f run_attitude_controller(Quaternion target, Quaternion meas);

    // run angular velocity controller 
    void run_angvel_controller(Vector3f target, Vector3f meas, Vector3f ang_acc_desired);

    // run angular velocity I controller
    Vector3f angular_velocity_I_controller(Vector3f error);

    // calculate estimated torque and thrust values using current motor speed
    void calculate_torque_thrust_est(void);

    // construct an attitude target using command specific thrust and 
    // yaw(heading) angle in radian
    Quaternion input_acc_des_euler_angle_yaw(float yaw_rad);

    // get throttle command that is send to mixer 
    const float& get_total_thrust_cmd_scaled() const {return _total_thrust_cmd_scaled; }

    // get torque command that is send to mixer
    const Vector3f& get_torque_cmd_scaled() const { return _torque_cmd_scaled; }


    const Vector3f& get_pos_target() const { return _pos_target_neu_m; }
    const Vector3f& get_vel_target() const { return _vel_target_neu_mps; }
    const Vector3f& get_lin_accel_target() const { return _lin_acc_target_ned_mpss; }
    const Vector3f& get_spec_thrst_cmd() const { return _spec_thrust_cmd_ned_mpss; }

    const Quaternion& get_attitude_quad_target() const { return _att_target_quat; }
    const Vector3f& get_ang_vel_target() const { return _ang_vel_target_radps; }
    const Vector3f& get_ang_vel_integrator() const {return av_integrator; }
    const Vector3f& get_ang_acc_target() const { return _ang_acc_target_radpss; }
    const Vector3f& get_torque_cmd() const { return _torque_cmd_body_Nm; }

    const Vector3f& get_ang_err() const { return _error_att_save; }

    const Vector3f& get_ang_acc_z_transform() const { return _angular_acc_z; } // [rad/s^2]

    // Output of the Off-CG data
    const Vector3f& get_mw_1_acc() const {return mw_1_acc;}
    const Vector3f& get_mw_1_acc_f() const {return mw_1_acc_f;}
    const Vector3f& get_mw_1_acc_wo_g() const {return mw_1_acc_wo_g;}
    const Vector3f& get_mw_1_acc_wo_g_f() const {return mw_1_acc_wo_g_f;}
    const Vector3f& get_mw_1_gyr() const {return mw_1_gyr;}

    const Vector3f& get_mw_2_acc() const {return mw_2_acc;}
    const Vector3f& get_mw_2_acc_f() const {return mw_2_acc_f;}
    const Vector3f& get_mw_2_acc_wo_g() const {return mw_2_acc_wo_g;}
    const Vector3f& get_mw_2_acc_wo_g_f() const {return mw_2_acc_wo_g_f;}
    const Vector3f& get_mw_2_gyr() const {return mw_2_gyr;}

    const Vector3f& get_mw_3_acc() const {return mw_3_acc;}
    const Vector3f& get_mw_3_acc_f() const {return mw_3_acc_f;}
    const Vector3f& get_mw_3_acc_wo_g() const {return mw_3_acc_wo_g;}
    const Vector3f& get_mw_3_acc_wo_g_f() const {return mw_3_acc_wo_g_f;}
    const Vector3f& get_mw_3_gyr() const {return mw_3_gyr;}

    const Vector3f& get_mw_4_acc() const {return mw_4_acc;}
    const Vector3f& get_mw_4_acc_f() const {return mw_4_acc_f;}
    const Vector3f& get_mw_4_acc_wo_g() const {return mw_4_acc_wo_g;}
    const Vector3f& get_mw_4_acc_wo_g_f() const {return mw_4_acc_wo_g_f;}
    const Vector3f& get_mw_4_gyr() const {return mw_4_gyr;}

    const Vector3f& get_mw_5_acc() const {return mw_5_acc;}
    const Vector3f& get_mw_5_acc_f() const {return mw_5_acc_f;}
    const Vector3f& get_mw_5_acc_wo_g() const {return mw_5_acc_wo_g;}
    const Vector3f& get_mw_5_acc_wo_g_f() const {return mw_5_acc_wo_g_f;}
    const Vector3f& get_mw_5_gyr() const {return mw_5_gyr;}

    const Vector3f& get_cg_acc_wo_grav() const {return cg_acc_wo_grav;}

    // Output of the NAP
    const Vector3f& get_acc_nap() const {return acc_nap;}

    // Output of the 3aw
    const Vector3f& get_acc_3aw() const {return acc_3aw;}

    // Output of the 6aw
    const Vector3f& get_acc_6aw() const {return acc_6aw;}

    // Output of the tri_axis
    const Vector3f& get_acc_tri_axis() const {return acc_tri_axis;}

    // get P controllers
    AC_P& get_pos_xy_p() { return _p_pos_xy; }
    AC_P& get_vel_xy_p() { return _p_vel_xy; }
    AC_P& get_pos_z_p() { return _p_pos_z; }
    AC_P& get_pos_p() { return _p_pos_z; }

    AC_P& get_ang_roll_p() { return _p_angle_x; }
    AC_P& get_ang_pitch_p() { return _p_angle_y; }
    AC_P& get_ang_yaw_p() { return _p_angle_z; }
    AC_P& get_rate_roll_p() { return _p_ang_rate_x; }
    AC_P& get_rate_pitch_p() { return _p_ang_rate_y; }
    AC_P& get_rate_yaw_p() { return _p_ang_rate_z; }

    static const struct AP_Param::GroupInfo var_info[];

    // INDI on/off switch
    void set_use_INDI(bool use_INDI);
    bool get_use_INDI() { return _use_INDI;}

protected:
    // add delta linear accleration to current specific thrust to obtain
    // specific thrust command
    void indi_linear_accel(bool enable_xy, float accel_max_xy_mpss);

    // First limit magnitude of specific thrust command
    // then limit xy axis of specific thrust command to no
    // grater than 1 g or z axis command
    void limit_specific_thrust(float accel_max_xy);

    // convert specific thrust command to the scaled thrust command
    void spec_thrust_to_scaled_thrust(void);

    // calculate attiude error
    // same as thrust_heading_rotation_angles function in AC_AttitudeControl.cpp
    Vector3f calculate_att_error(Quaternion target, Quaternion meas);

    // add delta angular accleration to current torque to obtain torque command
    void indi_angular_accel(void);

    // set mixer input from the torque and thrust command
    void scale_torque_cmd(void);

    // *************************** Angular acceleration *********************************

    // Off-CG data arrange (1~5 sensors)
    void Off_CG_data_arrange(void);

    // z transform 400hz for angular acceleration using angular velocity
    void z_transform_acc(void);

    // calculating of the angular acceleration using NAP
    void NAP_cal(void);

    // calculating of the angular acceleration using 3aw
    void acc_3aw_cal(void);

    // calculating of the angular acceleration using 6aw
    void acc_6aw_cal(void);

    // calculating of the angular acceleration using tri-axis
    void acc_tri_axis_cal(void);

    // **********************************************************************************

    // allocate torque and thrust cmd to the each motor thrust
    void control_allocation(void);

    // assign measured motor speed to _motor_speed_meas_radps
    void get_motor_speed(void);

    // references to inertial nav and ahrs libraries
    AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;

    // Parameters
    AP_Int8     enable_chan;

    // position and velocity P controller parameters
    AC_P        _p_pos_xy;
    AC_P        _p_vel_xy;
    AC_P        _p_pos_z;
    AC_P        _p_vel_z;

    // attitude and angular velocity P controller
    AC_P        _p_angle_x;
    AC_P        _p_angle_y;
    AC_P        _p_angle_z;
    AC_P        _p_ang_rate_x;
    AC_P        _p_ang_rate_y;
    AC_P        _p_ang_rate_z;

    // angular velocity I controller and max
    AP_Float        _i_ang_rate_x;
    AP_Float        _i_ang_rate_y;
    AP_Float        _i_ang_rate_z;

    AP_Float        _i_ang_rate_x_max;
    AP_Float        _i_ang_rate_y_max;
    AP_Float        _i_ang_rate_z_max;

    Vector3f av_integrator = {0.0f, 0.0f, 0.0f};

    // vehicle properties
    AP_Float    _mass_kg;                       // mass in kg
    AP_Float    _moment_inertia_xx_kgm2;        // moment of inertia of xx axis in kg.m²
    AP_Float    _moment_inertia_yy_kgm2;        // moment of inertia of yy axis in kg.m²
    AP_Float    _moment_inertia_z_kgm2;         // moment of inertia of z axis in kg.m²
    AP_Float    _arm_length_m;                  // distance to motors in m
    AP_Float    _thrust_coefficient;            // thrust coefficient in N/(rad/s)²
    AP_Float    _torque_coefficient;            // torque coefficient in Nm/(rad/s)²
    AP_Float    _throttle2motor_speed;          // coefficient between scaled throttle(between 0-1) command and motor speed in rad/s 
    AP_Float    _torque_est_filter_cutoff;      // rpm filter cutoff frequency for torque estimation in Hz
    AP_Float    _spec_thrust_est_filter_cutoff; // rpm filter cutoff frequency for specific thrust estimation in Hz
    AP_Float    _spec_thrust_cmd_filter_cutoff; // specific thrust command filter cutoff frequency in Hz
    AP_Float    _yaw_rate_filter_cutoff;        // torque command filter cutoff frequency in Hz
    AP_Int32    _ang_acc_sel;                   // Select angular acc
    AP_Float    _rpm_filt;                      // rpm filter
    AP_Float    _x_scale;                         // scale x
    AP_Float    _y_scale;                         // scale y
    AP_Float    _z_scale;                         // scale z
    AP_Float    _mw1_lpf;                       // Cutoff frequency of LPF about MW 1 sensor
    AP_Float    _mw2_lpf;                       // Cutoff frequency of LPF about MW 2 sensor
    AP_Float    _mw3_lpf;                       // Cutoff frequency of LPF about MW 3 sensor
    AP_Float    _mw4_lpf;                       // Cutoff frequency of LPF about MW 4 sensor
    AP_Float    _mw5_lpf;                       // Cutoff frequency of LPF about MW 5 sensor

    Vector3f    _pos_target_neu_m;              // position target in NEU frame in m
    Vector3f    _vel_target_neu_mps;            // velocity target in NEU frame in m/s
    Vector3f    _lin_acc_target_ned_mpss;       // linear accleration target in NED frame m/s²
    Vector3f    _lin_acc_desired_ned_mpss;      // feedforwarded linear acceleration in NED frame m/s²
    Vector3f    _spec_thrust_cmd_ned_mpss;      // specifc thrust command in NED frame in m/s²
    Vector3f    _spec_thrust_est_ned_mpss;      // estimated specific thrust from motor speed in NED frame in m/s²
    float       _total_thrust_cmd_body_N;       // total thrust command in body frame in N
    float       _total_thrust_cmd_scaled;       // scaled total thrust command between 0 ~ 1

    Quaternion  _att_target_quat;               // target attitude defined using desired yaw and specific thrust command
    float       _att_target_euler_angle_yaw_rad;// target attitude used only for target heading in rad
    Vector3f    _ang_vel_target_radps;          // angular velocity target in body frame in rad/s
    Vector3f    _ang_acc_target_radpss;         // angular acceleration target in body frame in rad/s²
    Vector3f    _ang_acc_desired_radpss;        // feedforwarded angular acceleration in NED frame rad/s²
    Vector3f    _torque_cmd_body_Nm;            // torque command in body frame in Nm
    Vector3f    _torque_cmd_scaled ;            // scaled torque command between -1 ~ +1
    Vector3f    _torque_est_body_Nm;            // estimated torque from motor speed in body frame in Nm

    Vector3f    _error_att_save;                 // For saving
    Vector3f    _error_ang_vel_save;             // For saving

    Vector3f    _ang_acc_;
    float       _ang_acc_check_x;
    float       _ang_acc_check_y;
    float       _ang_acc_check_z;

    double _motor_cmd_radps[4];                  // motor command in rad/s   !!!NOT USED
    double _motor_cmd_scaled[4];                 // scaled motor command 0-1 !!!NOT USED
    double _motor_speed_meas_radps[4];           // current motor speed in rad/s
    double _motor_speed_meas_radps_f[4];           // current motor speed in rad/s

    double _motor_speed_hz[4];
    double _motor_speed_rpm[4];

    LowPassFilterVector3f _torque_est_filter;
    LowPassFilter2pFloat _spec_thrust_est_filter;
    LowPassFilterVector3f _spec_thrust_cmd_filter;
    LowPassFilterFloat _yaw_rate_filter;

    LowPassFilterFloat _rpm_1_filter;
    LowPassFilterFloat _rpm_2_filter;
    LowPassFilterFloat _rpm_3_filter;
    LowPassFilterFloat _rpm_4_filter;

    LowPassFilterVector3f _mw1_lpf_filter;
    LowPassFilterVector3f _mw2_lpf_filter;
    LowPassFilterVector3f _mw3_lpf_filter;
    LowPassFilterVector3f _mw4_lpf_filter;
    LowPassFilterVector3f _mw5_lpf_filter;

    // Angular acc of z transform
    float _p_dot_z_transform = 0.0f;
    float _q_dot_z_transform = 0.0f;
    float _r_dot_z_transform = 0.0f;

    // roll axis
    float _p_u_prev_1 = 0.0f;
    float _p_u_prev_2 = 0.0f;
    float _p_y_prev_1 = 0.0f;
    float _p_y_prev_2 = 0.0f;

    // pitch axis
    float _q_u_prev_1 = 0.0f;
    float _q_u_prev_2 = 0.0f;
    float _q_y_prev_1 = 0.0f;
    float _q_y_prev_2 = 0.0f;

    // yaw axis
    float _r_u_prev_1 = 0.0f;
    float _r_u_prev_2 = 0.0f;
    float _r_y_prev_1 = 0.0f;
    float _r_y_prev_2 = 0.0f;

    Vector3f _angular_acc_z;

    // Arranging of the Off-CG sensor data
    Vector3f mw_1_acc = {0.0f,0.0f,0.0f};
    Vector3f mw_1_acc_f = {0.0f,0.0f,0.0f};
    Vector3f mw_1_acc_wo_g = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_1_acc_wo_g_f = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_1_gyr = {0.0f,0.0f,0.0f};

    Vector3f mw_2_acc = {0.0f,0.0f,0.0f};
    Vector3f mw_2_acc_f = {0.0f,0.0f,0.0f};
    Vector3f mw_2_acc_wo_g = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_2_acc_wo_g_f = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_2_gyr = {0.0f,0.0f,0.0f};

    Vector3f mw_3_acc = {0.0f,0.0f,0.0f};
    Vector3f mw_3_acc_f = {0.0f,0.0f,0.0f};
    Vector3f mw_3_acc_wo_g = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_3_acc_wo_g_f = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_3_gyr = {0.0f,0.0f,0.0f};

    Vector3f mw_4_acc = {0.0f,0.0f,0.0f};
    Vector3f mw_4_acc_f = {0.0f,0.0f,0.0f};
    Vector3f mw_4_acc_wo_g = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_4_acc_wo_g_f = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_4_gyr = {0.0f,0.0f,0.0f};

    Vector3f mw_5_acc = {0.0f,0.0f,0.0f};
    Vector3f mw_5_acc_f = {0.0f,0.0f,0.0f};
    Vector3f mw_5_acc_wo_g = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_5_acc_wo_g_f = {0.0f,0.0f,0.0f}; // w/o gravity
    Vector3f mw_5_gyr = {0.0f,0.0f,0.0f};
private:
    static AC_INDI_Control *_singleton;

    double Ct = 3.336e-5; // N/(rad/s_^2)
    double Cq = 5.694e-7; // Nm/(rad/s_^2)

    // Num 1: Right (Coordinate : NWU)
    Vector3f mw_1_pos = {0.0f, -0.48f, 0.0f};

    // Num 2: Left (Coordinate : NWU)
    Vector3f mw_2_pos = {0.0f, 0.48f, 0.0f};

    // Num 3: Front (Coordinate : NWU)
    Vector3f mw_3_pos = {0.48f, 0.0f, 0.0f};

    // Num 4: Rear (Coordinate : NWU)
    Vector3f mw_4_pos = {-0.48f, 0.0f, 0.0f};

    // Num 5: Top (Coordinate : NWU)
    Vector3f mw_5_pos = {0.0f, 0.0f, -0.15f};

    // NAP : angular acceleration (NWU)
    Vector3f acc_nap = {0.0f,0.0f,0.0f};

    // 3aw : angular acceleration (NWU)
    Vector3f acc_3aw = {0.0f, 0.0f, 0.0f};

    // 6aw : angular acceleration (NWU)
    Vector3f acc_6aw = {0.0f,0.0f,0.0f};

    // try-axis : angular acceleration (NWU)
    Vector3f acc_tri_axis = {0.0f,0.0f,0.0f};

    // accelerometer from CG w/ gravity
    Vector3f cg_acc = {0.0f,0.0f,0.0f};

    // accelerometer from CG w/o gravity : NED [m/s^2] no gravity
    Vector3f cg_acc_wo_grav = {0.0f,0.0f,0.0f};

    Vector3f accel_gravity{0.0f, 0.0f, GRAVITY_MSS}; // m/s/s NED, body frame

    Vector3f acc_body_no_gravity = {0.0f,0.0f,0.0f};

    // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
    void ned_to_body(const Vector3f& euler_rad, const Vector3f& ned_contents, Vector3f& body_contents);

    float pwm_test_1;


public:
    // INDI on/off switch
    bool _use_INDI = true;

    float _rpm_indi_1 = 0.0f;
    float _rpm_indi_2 = 0.0f;
    float _rpm_indi_3 = 0.0f;
    float _rpm_indi_4 = 0.0f;
};

namespace AP {
    AC_INDI_Control &indi_control();
};
