#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>
#include "AP_MW_AHRS.h"
#include <AP_CANManager/AP_CANManager.h>

extern const AP_HAL::HAL& hal;

float AP_MW_AHRS::acc_mw_ahrs[3][5];
float AP_MW_AHRS::gyr_mw_ahrs[3][5];
float AP_MW_AHRS::ang_mw_ahrs[3][5];

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "MW_AHRS", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

AP_MW_AHRS::AP_MW_AHRS()
{
    debug_can(AP_CANManager::LOG_INFO, "constructed");
}

AP_MW_AHRS *AP_MW_AHRS::get_mw_ahrs(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_MW_AHRS) {
        return nullptr;
    }
    return static_cast<AP_MW_AHRS*>(AP::can().get_driver(driver_index));
}

bool AP_MW_AHRS::add_interface(AP_HAL::CANIface* can_iface) {

    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "Multiple Interface not supported");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Cannot add event handle");
        return false;
    }
    return true;
}

void AP_MW_AHRS::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_INFO, "starting init");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "already initialized");
        return;
    }

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "Interface not found");
        return;
    }

    frame_id_t id = { { .node_id = 0,
                      .unused = 0 } };

    uint32_t start = AP_HAL::millis();

    // wait 1 second for answers
    while (AP_HAL::millis() - start < 1000) {
        AP_HAL::CANFrame sens_id_frame {};
        uint64_t rx_time;
        AP_HAL::CANIface::CanIOFlags flags = 0;

        int16_t n = _can_iface->receive(sens_id_frame, rx_time, flags);

        if (n != 1) {
            continue;
        }

        id.value = sens_id_frame.id & AP_HAL::CANFrame::MaskStdID;

        if (id.node_id >= (MAX_NUM_MW_AHRS + SENS_NODE_ID_FIRST)) {
            continue;
        }

        _sens_present_bitmask |= (1 << (id.node_id - SENS_NODE_ID_FIRST));

        debug_can(AP_CANManager::LOG_DEBUG, "found MW_AHRS sensor id %u", id.node_id);
    }

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MW_AHRS::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(AP_CANManager::LOG_ERROR, "couldn't create thread");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "init done");

    return;
}

void AP_MW_AHRS::loop()
{
    uint64_t timeout;

    AP_HAL::CANFrame empty_frame { (0 | AP_HAL::CANFrame::FlagEFF), nullptr, 0 };
    
    while(true) {
        
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "not initialized");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        uint64_t now = AP_HAL::micros64();
        bool read_select = true;
        bool write_select = false;
        bool select_ret;

        if (!_sens_present_bitmask) {
            hal.scheduler->delay(1000);
            continue;
        }

        // always look for received frames
        timeout = now + LOOP_INTERVAL_US;
            
        // Immediately check if rx buffer not empty
        select_ret = _can_iface->select(read_select, write_select, &empty_frame, timeout);
        
        if (select_ret && read_select) {
            AP_HAL::CANFrame frame;
            uint64_t rx_time;
            AP_HAL::CANIface::CanIOFlags flags {};

            int16_t res = _can_iface->receive(frame, rx_time, flags);

            if (res == 1) {

                frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskStdID };

                if ((1 << (id.node_id - SENS_NODE_ID_FIRST)) & _sens_present_bitmask) {
                    switch (frame.data[1]) {
                        case ACC:
                        {
                            if (frame.dlc != 8 || frame.data[0] != 0xF0) {
                                break;
                            }

                            int16_t acc_x = ((int)(unsigned char)frame.data[2] | (int)(unsigned char)frame.data[3] << 8);
                            int16_t acc_y = ((int)(unsigned char)frame.data[4] | (int)(unsigned char)frame.data[5] << 8);
                            int16_t acc_z = ((int)(unsigned char)frame.data[6] | (int)(unsigned char)frame.data[7] << 8);

                            float accx = acc_x * 9.8 / 1000.0;
                            float accy = acc_y * 9.8 / 1000.0;
                            float accz = acc_z * 9.8 / 1000.0;

                            if (accx != acc_mw_ahrs_prev[0][id.node_id] || accy != acc_mw_ahrs_prev[1][id.node_id] || accz != acc_mw_ahrs_prev[2][id.node_id]) {
                                
                                if (accx == 0 && accy == 0 && accz == 0) {
                                    break;
                                }

                                acc_mw_ahrs[0][id.node_id-1] = accx;
                                acc_mw_ahrs[1][id.node_id-1] = accy;
                                acc_mw_ahrs[2][id.node_id-1] = accz;

                                acc_mw_ahrs_prev[0][id.node_id-1] = acc_mw_ahrs[0][id.node_id-1];
                                acc_mw_ahrs_prev[1][id.node_id-1] = acc_mw_ahrs[1][id.node_id-1];
                                acc_mw_ahrs_prev[2][id.node_id-1] = acc_mw_ahrs[2][id.node_id-1];

                                switch (id.node_id) {
                                    case 1:
                                    {
                                        Write_MW_AHRS_ID1();
                                        break;
                                    }
                                    case 2:
                                    {
                                        Write_MW_AHRS_ID2();
                                        break;
                                    }
                                    case 3:
                                    {
                                        Write_MW_AHRS_ID3();
                                        break;
                                    }
                                    case 4:
                                    {
                                        Write_MW_AHRS_ID4();
                                        break;
                                    }
                                    case 5:
                                    {
                                        Write_MW_AHRS_ID5();
                                        break;
                                    }
                                    default:
                                    {
                                        break;
                                    }
                                }
                            }

                            break;
                        }
                        case GYR:
                        {
                            if (frame.dlc != 8 || frame.data[0] != 0xF0) {
                                break;
                            }

                            int16_t gyr_x = ((int)(unsigned char)frame.data[2] | (int)(unsigned char)frame.data[3] << 8);
                            int16_t gyr_y = ((int)(unsigned char)frame.data[4] | (int)(unsigned char)frame.data[5] << 8);
                            int16_t gyr_z = ((int)(unsigned char)frame.data[6] | (int)(unsigned char)frame.data[7] << 8);

                            float gyrx = gyr_x / 10.0;
                            float gyry = gyr_y / 10.0;
                            float gyrz = gyr_z / 10.0;

                            if (gyrx != gyr_mw_ahrs_prev[0][id.node_id] || gyry != gyr_mw_ahrs_prev[1][id.node_id] || gyrz != gyr_mw_ahrs_prev[2][id.node_id]) {

                                if (gyrx == 0 && gyry == 0 && gyrz == 0) {
                                    break;
                                }

                                gyr_mw_ahrs[0][id.node_id-1] = gyrx;
                                gyr_mw_ahrs[1][id.node_id-1] = gyry;
                                gyr_mw_ahrs[2][id.node_id-1] = gyrz;

                                gyr_mw_ahrs_prev[0][id.node_id-1] = gyr_mw_ahrs[0][id.node_id-1];
                                gyr_mw_ahrs_prev[1][id.node_id-1] = gyr_mw_ahrs[1][id.node_id-1];
                                gyr_mw_ahrs_prev[2][id.node_id-1] = gyr_mw_ahrs[2][id.node_id-1];

                                switch (id.node_id) {
                                    case 1:
                                    {
                                        Write_MW_AHRS_ID1();
                                        break;
                                    }
                                    case 2:
                                    {
                                        Write_MW_AHRS_ID2();
                                        break;
                                    }
                                    case 3:
                                    {
                                        Write_MW_AHRS_ID3();
                                        break;
                                    }
                                    case 4:
                                    {
                                        Write_MW_AHRS_ID4();
                                        break;
                                    }
                                    case 5:
                                    {
                                        Write_MW_AHRS_ID5();
                                        break;
                                    }
                                    default:
                                    {
                                        break;
                                    }
                                }
                            }

                            break;
                        }
                        case ANG:
                        {
                            if (frame.dlc != 8 || frame.data[0] != 0xF0) {
                                break;
                            }

                            int16_t ang_x = ((int)(unsigned char)frame.data[2] | (int)(unsigned char)frame.data[3] << 8);
                            int16_t ang_y = ((int)(unsigned char)frame.data[4] | (int)(unsigned char)frame.data[5] << 8);
                            int16_t ang_z = ((int)(unsigned char)frame.data[6] | (int)(unsigned char)frame.data[7] << 8);

                            float angx = ang_x / 100.0;
                            float angy = ang_y / 100.0;
                            float angz = ang_z / 100.0;

                            if (angx != ang_mw_ahrs_prev[0][id.node_id] || angy != ang_mw_ahrs_prev[1][id.node_id] || angz != ang_mw_ahrs_prev[2][id.node_id]) {

                                if (angx == 0 && angy == 0 && angz == 0) {
                                    break;
                                }

                                ang_mw_ahrs[0][id.node_id-1] = angx;
                                ang_mw_ahrs[1][id.node_id-1] = angy;
                                ang_mw_ahrs[2][id.node_id-1] = angz;

                                ang_mw_ahrs_prev[0][id.node_id-1] = ang_mw_ahrs[0][id.node_id-1];
                                ang_mw_ahrs_prev[1][id.node_id-1] = ang_mw_ahrs[1][id.node_id-1];
                                ang_mw_ahrs_prev[2][id.node_id-1] = ang_mw_ahrs[2][id.node_id-1];

                                switch (id.node_id) {
                                    case 1:
                                    {
                                        Write_MW_AHRS_ID1();
                                        break;
                                    }
                                    case 2:
                                    {
                                        Write_MW_AHRS_ID2();
                                        break;
                                    }
                                    case 3:
                                    {
                                        Write_MW_AHRS_ID3();
                                        break;
                                    }
                                    case 4:
                                    {
                                        Write_MW_AHRS_ID4();
                                        break;
                                    }
                                    case 5:
                                    {
                                        Write_MW_AHRS_ID5();
                                        break;
                                    }
                                    default:
                                    {
                                        break;
                                    }
                                }
                            }

                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                }
            }
        }
    }
}

bool AP_MW_AHRS::pre_arm_check(char* reason, uint8_t reason_len)
{
    uint8_t num_present_sens = __builtin_popcount(_sens_present_bitmask);

    if (num_present_sens < 1) {
        debug_can(AP_CANManager::LOG_DEBUG, "too few MW_AHRS sensor detected (%u of %u)", (int)num_present_sens, 1);
        return false;
    }

    if (num_present_sens > MAX_NUM_MW_AHRS) {
        debug_can(AP_CANManager::LOG_DEBUG, "too many MW_AHRS sensor detected (%u > %u)", (int)num_present_sens, (int)MAX_NUM_MW_AHRS);
        return false;
    }

    return true;
}