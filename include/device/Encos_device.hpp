/**
 * @file Encos_device.hpp
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief Bitbot Encos device header file
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "bitbot_kernel/device/device.hpp"
#include "bus/Encos_bus_msg.h"
#include "vector"

namespace bitbot
{
    class EncosBus;

    /**
     * @brief Encos设备类型, 开发者可以在该枚举中添加自己的设备类型。该类型仅适用于开发者使用，用户无需关心其实现细节。
     *
     */
    enum class EncosDeviceType : uint32_t
    {
        Encos_DEVICE = 1000,
        Encos_JOINT,
        Yesense_IMU,
        Xinxiangyang_BATTERY
    };

    /**
     * @brief Encos设备类，继承自Device, 用户可以使用该类来管理Encos设备。
     * @details Encos设备类，继承自Device, 用户可以使用该类来管理Encos设备。
     * 该类定义了Encos设备的基本功能，用户和开发者无需关心该类的实现细节。
     * 建议开发者使用Encos_CANBusDevice和Encos_VirtualBusDevice类来实现自己的设备。
     *
     */
    class EncosDevice : public Device
    {
        friend class EncosBus;

    public:
        /**
         * @brief 构造函数
         * @details 构造函数，该函数由Bitbot Encos内核创建。
         *
         * @param device_node 设备节点的配置文件
         */
        EncosDevice(const pugi::xml_node& device_node)
            : Device(device_node)
        {
        }

        virtual ~EncosDevice() = default;

        /**
         * @brief 更新运行时数据，该函数会被内核周期性调用。
         * @details 更新运行时数据，该函数会被内核周期性调用。开发者必须在设备中实现该函数。
         *
         */
        virtual void UpdateRuntimeData() = 0;

        /**
         * @brief 判断设备是否为虚拟总线设备，虚拟总线设备是指未实际挂载在EtherCAT总线上，但也需要进行数据交换的设备。
         *
         * @return true
         * @return false
         */
        constexpr virtual bool VirtualBusDevice() const = 0;

    protected:
        /**
         * @brief 默认的上电函数，开发者可以在该函数中实现自己的上电逻辑。
         *
         * @return true
         * @return false
         */
        virtual bool PowerOn()
        {
            return true;
        }

        /**
         * @brief 判断设备是否有上电配置，开发者可以根据设备类型的实际情况实现该函数。
         *
         * @return true 设备具备上电功能
         * @return false 设备不具备上电功能
         */
        virtual bool HasPowerCfg()
        {
            return false;
        }

        /**
         * @brief 默认的下电函数，开发者可以在该函数中实现自己的下电逻辑。
         *
         * @return true
         * @return false
         */
        virtual bool PowerOff()
        {
            return true;
        }
    };

    /**
     * @brief Encos总线设备类，继承自EncosDevice, 用户可以使用该类来管理Encos总线设备。
     * 该类型仅适用于开发者使用，用户无需关心其实现细节。该类型设备是指实际挂载在EtherCAT总线上的设备。
     *
     */
    class Encos_CANBusDevice : public EncosDevice
    {
        friend class EncosBus;

    public:
        /**
         * @brief 构造函数
         * @details 构造函数，该函数由Bitbot Encos内核创建。
         *
         * @param device_node 设备节点的配置文件
         */
        Encos_CANBusDevice(const pugi::xml_node& device_node)
            : EncosDevice(device_node)
        {
        }

        /**
         * @brief 析构函数
         * @details 析构函数。设备的生命期由Bitbot Encos内核管理，开发者无需关心设备生命周期。
         *
         */
        virtual ~Encos_CANBusDevice() = default;

    protected:

        /**
         * @brief 获取EtherCAT从站ID
         *
         * @return size_t 从站ID
         */
        virtual size_t get_EtherCAT_Slave_ID() const = 0;

        /**
         * @brief 获取EtherCAT转CAN总线的设备ID
         *
         * @param ids 设备ID列表。一个设备可以有多个CAN总线ID。
         */
        virtual void get_CAN_IDs(std::vector<size_t>& ids) const = 0;

        /**
         * @brief 进行一次读取操作，开发者需要在该函数中实现自己的读取逻辑。该函数会被总线管理器周期性调用。
         *
         * @param data 读取的数据
         */
        virtual void ReadBus(const CAN_Device_Msg& data) = 0;

        /**
         * @brief 进行一次写入操作，开发者需要在该函数中实现自己的写入逻辑。该函数会被总线管理器周期性调用。
         *
         * @param data 写入的数据
         */
        virtual void WriteBus(CAN_Device_Msg& data) = 0;

        constexpr virtual bool VirtualBusDevice() const final override
        {
            return false;
        }
    };


    /**
     * @brief Encos虚拟总线设备类，继承自EncosDevice, 用户可以使用该类来管理Encos虚拟总线设备。
     * 该类型仅适用于开发者使用，用户无需关心其实现细节。该类型设备是指未挂载在EtherCAT总线上的设备。
     *
     */
    class Encos_VirtualBusDevice : public EncosDevice
    {
        friend class EncosBus;

    public:
        /**
         * @brief 构造函数
         * @details 构造函数，该函数由Bitbot Encos内核创建。
         *
         * @param device_node 设备节点的配置文件
         */
        Encos_VirtualBusDevice(const pugi::xml_node& device_node)
            : EncosDevice(device_node)
        {
        }

        /**
         * @brief 析构函数
         * @details 析构函数。设备的生命期由Bitbot Encos内核管理，开发者无需关心设备生命周期。
         *
         */
        virtual ~Encos_VirtualBusDevice() = default;

    protected:
        /**
         * @brief 进行一次读取操作，开发者需要在该函数中实现自己的读取逻辑。该函数会被总线管理器周期性调用，但不实际读写总线。
         *
         */
        virtual void ReadOnce() = 0;

        /**
         * @brief 进行一次写入操作，开发者需要在该函数中实现自己的写入逻辑。该函数会被总线管理器周期性调用，但不实际读写总线。
         *
         */
        virtual void WriteOnce() = 0;

        constexpr virtual bool VirtualBusDevice() const final override
        {
            return true;
        }
    };
};