#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

void AC_AttitudeControl::indi_log(void) const
{

    AP::logger().WriteStreaming("NDIR", "TimeUS,praw,pfilt,pacc1,pacc2,pacc3,pacc4,nu,nupr,nuf,delu,indio", "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           (double)_indi_monitor.p_raw,
                                           (double)_indi_monitor.filter_p_now,
                                           (double)_indi_monitor.p_acc_diff,
                                           (double)_indi_monitor.filter_p_acc_diff,
                                           (double)_indi_monitor.accel_p,
                                           (double)_indi_monitor.sensor_p_dot_z_tranform,
                                           (double)_indi_monitor.nu_p,
                                           (double)_indi_monitor.nu_pr,
                                           (double)_indi_monitor.nu_f,
                                           (double)_indi_monitor.delta_u_p,
                                           (double)_indi_monitor.indi_out_p,
                                           (double)_indi_monitor.indi_roll_flag
                                        );

    AP::logger().WriteStreaming("NDIP", "TimeUS,qraw,qfilt,qacc1,qacc2,qacc3,pacc4,nu,delu,indio", "Qffffffffff",
                                           AP_HAL::micros64(),
                                           (double)_indi_monitor.q_raw,
                                           (double)_indi_monitor.filter_q_now,
                                           (double)_indi_monitor.q_acc_diff,
                                           (double)_indi_monitor.filter_q_acc_diff,
                                           (double)_indi_monitor.accel_q,
                                           (double)_indi_monitor.sensor_q_dot_z_tranform,
                                           (double)_indi_monitor.nu_q,
                                           (double)_indi_monitor.delta_u_q,
                                           (double)_indi_monitor.indi_out_q,
                                           (double)_indi_monitor.indi_pitch_flag
                                          );

    AP::logger().WriteStreaming("NDIY", "TimeUS,rraw,rfilt,racc1,racc2,racc3,pacc4,nu,delu,indio", "Qffffffffff",
                                           AP_HAL::micros64(),
                                           (double)_indi_monitor.r_raw,
                                           (double)_indi_monitor.filter_r_now,
                                           (double)_indi_monitor.r_acc_diff,
                                           (double)_indi_monitor.filter_r_acc_diff,
                                           (double)_indi_monitor.accel_r,
                                           (double)_indi_monitor.sensor_r_dot_z_tranform,
                                           (double)_indi_monitor.nu_r,
                                           (double)_indi_monitor.delta_u_r,
                                           (double)_indi_monitor.indi_out_r,
                                           (double)_indi_monitor.indi_yaw_flag
                                          );
}
