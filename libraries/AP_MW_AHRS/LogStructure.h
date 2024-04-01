#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_FROM_MW_AHRS \
    LOG_MW_AHRS_ID1_MSG, \
    LOG_MW_AHRS_ID2_MSG, \
    LOG_MW_AHRS_ID3_MSG, \
    LOG_MW_AHRS_ID4_MSG, \
    LOG_MW_AHRS_ID5_MSG

// @LoggerMessage: MWA1
// @Description: MW_AHRS sensor ID1 data
// @Field: TimeUS: Time since system startup
// @Field: AccXID1: X-axis acceleration of sensor ID1
// @Field: AccYID1: Y-axis acceleration of sensor ID1
// @Field: AccZID1: Z-axis acceleration of sensor ID1
// @Field: GyrXID1: Roll angular velocity of sensor ID1
// @Field: GyrYID1: Pitch angular velocity of sensor ID1
// @Field: GyrZID1: Yaw angular velocity of sensor ID1
// @Field: AngXID1: Roll angle of sensor ID1
// @Field: AngYID1: Pitch angle of sensor ID1
// @Field: AngZID1: Yaw angle of sensor ID1
struct PACKED log_MW_AHRS_ID1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float acc_x_id1, acc_y_id1, acc_z_id1;
    float gyr_x_id1, gyr_y_id1, gyr_z_id1;
    float ang_x_id1, ang_y_id1, ang_z_id1;
};

// @LoggerMessage: MWA2
// @Description: MW_AHRS sensor ID2 data
// @Field: TimeUS: Time since system startup
// @Field: AccXID2: X-axis acceleration of sensor ID2
// @Field: AccYID2: Y-axis acceleration of sensor ID2
// @Field: AccZID2: Z-axis acceleration of sensor ID2
// @Field: GyrXID2: Roll angular velocity of sensor ID2
// @Field: GyrYID2: Pitch angular velocity of sensor ID2
// @Field: GyrZID2: Yaw angular velocity of sensor ID2
// @Field: AngXID2: Roll angle of sensor ID2
// @Field: AngYID2: Pitch angle of sensor ID2
// @Field: AngZID2: Yaw angle of sensor ID2
struct PACKED log_MW_AHRS_ID2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float acc_x_id2, acc_y_id2, acc_z_id2;
    float gyr_x_id2, gyr_y_id2, gyr_z_id2;
    float ang_x_id2, ang_y_id2, ang_z_id2;
};


// @LoggerMessage: MWA3
// @Description: MW_AHRS sensor ID3 data
// @Field: TimeUS: Time since system startup
// @Field: AccXID3: X-axis acceleration of sensor ID3
// @Field: AccYID3: Y-axis acceleration of sensor ID3
// @Field: AccZID3: Z-axis acceleration of sensor ID3
// @Field: GyrXID3: Roll angular velocity of sensor ID3
// @Field: GyrYID3: Pitch angular velocity of sensor ID3
// @Field: GyrZID3: Yaw angular velocity of sensor ID3
// @Field: AngXID3: Roll angle of sensor ID3
// @Field: AngYID3: Pitch angle of sensor ID3
// @Field: AngZID3: Yaw angle of sensor ID3
struct PACKED log_MW_AHRS_ID3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float acc_x_id3, acc_y_id3, acc_z_id3;
    float gyr_x_id3, gyr_y_id3, gyr_z_id3;
    float ang_x_id3, ang_y_id3, ang_z_id3;
};


// @LoggerMessage: MWA4
// @Description: MW_AHRS sensor ID1 data
// @Field: TimeUS: Time since system startup
// @Field: AccXID4: X-axis acceleration of sensor ID4
// @Field: AccYID4: Y-axis acceleration of sensor ID4
// @Field: AccZID4: Z-axis acceleration of sensor ID4
// @Field: GyrXID4: Roll angular velocity of sensor ID4
// @Field: GyrYID4: Pitch angular velocity of sensor ID4
// @Field: GyrZID4: Yaw angular velocity of sensor ID4
// @Field: AngXID4: Roll angle of sensor ID4
// @Field: AngYID4: Pitch angle of sensor ID4
// @Field: AngZID4: Yaw angle of sensor ID4
struct PACKED log_MW_AHRS_ID4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float acc_x_id4, acc_y_id4, acc_z_id4;
    float gyr_x_id4, gyr_y_id4, gyr_z_id4;
    float ang_x_id4, ang_y_id4, ang_z_id4;
};


// @LoggerMessage: MWA5
// @Description: MW_AHRS sensor ID1 data
// @Field: TimeUS: Time since system startup
// @Field: AccXID5: X-axis acceleration of sensor ID5
// @Field: AccYID5: Y-axis acceleration of sensor ID5
// @Field: AccZID5: Z-axis acceleration of sensor ID5
// @Field: GyrXID5: Roll angular velocity of sensor ID5
// @Field: GyrYID5: Pitch angular velocity of sensor ID5
// @Field: GyrZID5: Yaw angular velocity of sensor ID5
// @Field: AngXID5: Roll angle of sensor ID5
// @Field: AngYID5: Pitch angle of sensor ID5
// @Field: AngZID5: Yaw angle of sensor ID5
struct PACKED log_MW_AHRS_ID5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float acc_x_id5, acc_y_id5, acc_z_id5;
    float gyr_x_id5, gyr_y_id5, gyr_z_id5;
    float ang_x_id5, ang_y_id5, ang_z_id5;
};

#define LOG_STRUCTURE_FROM_MW_AHRS \
    { LOG_MW_AHRS_ID1_MSG, sizeof(log_MW_AHRS_ID1), \
        "MWA1", "Qfffffffff", "TimeUS,AccX1,AccY1,AccZ1,GyrX1,GyrY1,GyrZ1,AngX1,AngY1,AngZ1", "soookkkddd", "F---------" }, \
    { LOG_MW_AHRS_ID2_MSG, sizeof(log_MW_AHRS_ID2), \
        "MWA2", "Qfffffffff", "TimeUS,AccX2,AccY2,AccZ2,GyrX2,GyrY2,GyrZ2,AngX2,AngY2,AngZ2", "soookkkddd", "F---------" }, \
    { LOG_MW_AHRS_ID3_MSG, sizeof(log_MW_AHRS_ID3), \
        "MWA3", "Qfffffffff", "TimeUS,AccX3,AccY3,AccZ3,GyrX3,GyrY3,GyrZ3,AngX3,AngY3,AngZ3", "soookkkddd", "F---------" }, \
    { LOG_MW_AHRS_ID4_MSG, sizeof(log_MW_AHRS_ID4), \
        "MWA4", "Qfffffffff", "TimeUS,AccX4,AccY4,AccZ4,GyrX4,GyrY4,GyrZ4,AngX4,AngY4,AngZ4", "soookkkddd", "F---------" }, \
    { LOG_MW_AHRS_ID5_MSG, sizeof(log_MW_AHRS_ID5), \
        "MWA5", "Qfffffffff", "TimeUS,AccX5,AccY5,AccZ5,GyrX5,GyrY5,GyrZ5,AngX5,AngY5,AngZ5", "soookkkddd", "F---------" },
