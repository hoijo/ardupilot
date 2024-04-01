#include "AP_MW_AHRS.h"
#include <AP_Logger/AP_Logger.h>

void AP_MW_AHRS::Write_MW_AHRS_ID1() const
{
    const struct log_MW_AHRS_ID1 pkt1{
        LOG_PACKET_HEADER_INIT(LOG_MW_AHRS_ID1_MSG),
        time_us     : AP_HAL::micros64(),
        acc_x_id1   : acc_mw_ahrs[0][0],
        acc_y_id1   : acc_mw_ahrs[1][0],
        acc_z_id1   : acc_mw_ahrs[2][0],
        gyr_x_id1   : gyr_mw_ahrs[0][0],
        gyr_y_id1   : gyr_mw_ahrs[1][0],
        gyr_z_id1   : gyr_mw_ahrs[2][0],
        ang_x_id1   : ang_mw_ahrs[0][0],
        ang_y_id1   : ang_mw_ahrs[1][0],
        ang_z_id1   : ang_mw_ahrs[2][0],
    };
    AP::logger().WriteBlock(&pkt1, sizeof(pkt1));
}

void AP_MW_AHRS::Write_MW_AHRS_ID2() const
{
    const struct log_MW_AHRS_ID2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_MW_AHRS_ID2_MSG),
        time_us     : AP_HAL::micros64(),
        acc_x_id2   : acc_mw_ahrs[0][1],
        acc_y_id2   : acc_mw_ahrs[1][1],
        acc_z_id2   : acc_mw_ahrs[2][1],
        gyr_x_id2   : gyr_mw_ahrs[0][1],
        gyr_y_id2   : gyr_mw_ahrs[1][1],
        gyr_z_id2   : gyr_mw_ahrs[2][1],
        ang_x_id2   : ang_mw_ahrs[0][1],
        ang_y_id2   : ang_mw_ahrs[1][1],
        ang_z_id2   : ang_mw_ahrs[2][1],
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void AP_MW_AHRS::Write_MW_AHRS_ID3() const
{
    const struct log_MW_AHRS_ID3 pkt3{
        LOG_PACKET_HEADER_INIT(LOG_MW_AHRS_ID3_MSG),
        time_us     : AP_HAL::micros64(),
        acc_x_id3   : acc_mw_ahrs[0][2],
        acc_y_id3   : acc_mw_ahrs[1][2],
        acc_z_id3   : acc_mw_ahrs[2][2],
        gyr_x_id3   : gyr_mw_ahrs[0][2],
        gyr_y_id3   : gyr_mw_ahrs[1][2],
        gyr_z_id3   : gyr_mw_ahrs[2][2],
        ang_x_id3   : ang_mw_ahrs[0][2],
        ang_y_id3   : ang_mw_ahrs[1][2],
        ang_z_id3   : ang_mw_ahrs[2][2],
    };
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}

void AP_MW_AHRS::Write_MW_AHRS_ID4() const
{
    const struct log_MW_AHRS_ID4 pkt4{
        LOG_PACKET_HEADER_INIT(LOG_MW_AHRS_ID4_MSG),
        time_us     : AP_HAL::micros64(),
        acc_x_id4   : acc_mw_ahrs[0][3],
        acc_y_id4   : acc_mw_ahrs[1][3],
        acc_z_id4   : acc_mw_ahrs[2][3],
        gyr_x_id4   : gyr_mw_ahrs[0][3],
        gyr_y_id4   : gyr_mw_ahrs[1][3],
        gyr_z_id4   : gyr_mw_ahrs[2][3],
        ang_x_id4   : ang_mw_ahrs[0][3],
        ang_y_id4   : ang_mw_ahrs[1][3],
        ang_z_id4   : ang_mw_ahrs[2][3],
    };
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}

void AP_MW_AHRS::Write_MW_AHRS_ID5() const
{
    const struct log_MW_AHRS_ID5 pkt5{
        LOG_PACKET_HEADER_INIT(LOG_MW_AHRS_ID5_MSG),
        time_us     : AP_HAL::micros64(),
        acc_x_id5   : acc_mw_ahrs[0][4],
        acc_y_id5   : acc_mw_ahrs[1][4],
        acc_z_id5   : acc_mw_ahrs[2][4],
        gyr_x_id5   : gyr_mw_ahrs[0][4],
        gyr_y_id5   : gyr_mw_ahrs[1][4],
        gyr_z_id5   : gyr_mw_ahrs[2][4],
        ang_x_id5   : ang_mw_ahrs[0][4],
        ang_y_id5   : ang_mw_ahrs[1][4],
        ang_z_id5   : ang_mw_ahrs[2][4],
    };
    AP::logger().WriteBlock(&pkt5, sizeof(pkt5));
}