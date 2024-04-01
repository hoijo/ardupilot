#pragma once

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_HAL/Semaphores.h>

#include <AP_Param/AP_Param.h>

#include <atomic>

// there are 12 motor functions in SRV_Channel but CAN driver can't keep up
#define MAX_NUM_MW_AHRS 5
#define ACC 0x33
#define GYR 0x34
#define ANG 0x35

class AP_MW_AHRS : public AP_CANDriver {
public:
    AP_MW_AHRS();
    
    /* Do not allow copies */
    AP_MW_AHRS(const AP_MW_AHRS &other) = delete;
    AP_MW_AHRS &operator=(const AP_MW_AHRS&) = delete;

    // Return MW_AHRS from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_MW_AHRS *get_mw_ahrs(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // check that arming can happen
    bool pre_arm_check(char* reason, uint8_t reason_len);

    float acc_mw_ahrs[3][5];
    float gyr_mw_ahrs[3][5];
    float ang_mw_ahrs[3][5];

private:
    void loop();
    void Write_MW_AHRS_ID1(void) const;
    void Write_MW_AHRS_ID2(void) const;
    void Write_MW_AHRS_ID3(void) const;
    void Write_MW_AHRS_ID4(void) const;
    void Write_MW_AHRS_ID5(void) const;

    bool _initialized;
    char _thread_name[11];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;

    // sensor detected information
    uint16_t _sens_present_bitmask;
    uint16_t _sens_max_node_id;

    union frame_id_t {
        struct PACKED {
            uint32_t node_id:11;
            uint32_t unused:21;
        };
        uint32_t value;
    };
    
    static const uint8_t SENS_NODE_ID_FIRST = 1;

    static const uint32_t LOOP_INTERVAL_US = 2000;

    float acc_mw_ahrs_prev[3][5];
    float gyr_mw_ahrs_prev[3][5];
    float ang_mw_ahrs_prev[3][5];
};