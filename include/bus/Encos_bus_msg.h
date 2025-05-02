/**
 * @file Encos_bus_msg.h
 * @author kx zhang
 * @brief Bitbot Encos bus msg header file
 * @details Bitbot Encos bus msg header file, used to define the bus msg structure.
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <inttypes.h>
#include <string.h>

 //********************************************//
 //***********EtherCAT Message*****************//
 //********************************************//

#pragma pack(push, 1)

/**
 * @brief Bitbot Encos 中的CAN消息结构体。
 *
 */
struct CAN_Device_Msg
{
    /// @brief 设备ID
    uint32_t id;
    /// @brief 帧类型
    uint8_t rtr;
    /// @brief 设备数据长度
    uint8_t dlc;
    /// @brief 设备数据
    uint8_t data[8];
};

/**
 * @brief Bitbot Encos 中的EtherCAT消息结构体。
 *
 */
typedef struct
{
    /// @brief CAN设备数量
    uint8_t device_number;

    /// @brief CAN总线ide标识符
    uint8_t can_ide;

    /// @brief CAN总线设备消息
    struct CAN_Device_Msg device[6];

} EtherCAT_Msg;

#pragma pack(pop)
