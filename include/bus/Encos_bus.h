/**
 * @file Encos_bus.h
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief Bitbot Encos bus header file
 * @details Bitbot Encos bus header file, used to manage the bus devices and handle the bus communication.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "bitbot_kernel/bus/bus_manager.hpp"
#include "device/Encos_device.hpp"
#include "bus/Encos_bus_msg.h"
#include "atomic"
#include "vector"
#include "map"

namespace bitbot
{
    /**
     * @brief Bitbot Encos总线类，继承自BusManagerTpl，该类型仅适用于开发者使用，用户无需关心其实现细节。
     * @details Bitbot Encos总线类，继承自BusManagerTpl，用于管理Bitbot Encos总线设备。
     * 该总线管理器使用Bitbot内核相似的总线管理器模型，与之不同的是，BitbotEncos总线使用了SOEM库来实现EtherCAT总线的通信，
     * 以及使用EtherCAT转CAN总线的方式来实现对Encos电机的控制。
     * 此外还引入了虚拟总线机制来兼容并未实际挂载在EtherCAT总线上，但也需要进行数据交换的设备(e.g. IMU)。
     *
     */
    class EncosBus : public BusManagerTpl<EncosBus, EncosDevice>
    {
    public:
        /**
         * @brief 构造函数
         *
         */
        EncosBus();

        /**
         * @brief 析构函数
         *
         */
        virtual ~EncosBus();

        /**
         * @brief 初始化函数
         *
         */
        void Init();

        // interface
        /**
         * @brief 初始化EtherCAT总线
         * @details 初始化EtherCAT总线，使用SOEM库来实现EtherCAT总线的通信。
         * @param ifname 网络接口名称
         * @return true 成功
         * @return false 失败
         */
        bool InitEtherCAT(const std::string& ifname);

        /**
         * @brief 写入总线数据，该函数会将数据写入到总线中，该函数会被内核周期性调用。
         *
         */
        void WriteBus();

        /**
         * @brief 读取总线数据，该函数会从总线中读取数据，该函数会被内核周期性调用。
         *
         */
        void ReadBus();

        /**
         * @brief 注册设备函数，该函数会将设备注册到总线中。开发者可以在该函数中注册自己的设备。
         *
         */
        void RegisterDevices();

        /**
         * @brief 控制设备上电，该函数会将设备上电。
         *
         * @param id 设备ID，默认为-1，表示所有设备
         * @details 该函数会将设备上电，开发者可以在该函数中实现自己的设备上电逻辑。
         */
        void PowerOnDevice(int id = -1);

        /**
         * @brief 控制设备下电，该函数会将设备下电。
         *
         * @param id 设备ID，默认为-1，表示所有设备
         * @details 该函数会将设备下电，开发者可以在该函数中实现自己的设备下电逻辑。
         */
        void PowerOffDevice(int id = -1);

        /// @brief 是否发生错误
        std::atomic_bool ErrorFlag;

    private:
        // bitbot bus variables
        std::vector<std::vector<Encos_CANBusDevice*>> CAN_Device_By_EtherCAT_ID; // for device write
        std::map<size_t, std::vector<Encos_CANBusDevice*>> CAN_Device_By_CAN_ID; // for device read
        std::vector<Encos_VirtualBusDevice*> VirtualBusDevices;

        std::vector<EtherCAT_Msg*> CAN_BusReadBuffer;
        std::vector<EtherCAT_Msg*> CAN_BusWriteBuffer;



    private:
        // soem related vairables
        bool needlf;
        bool inOP;
        uint64_t num;
        int expectedWKC;
        int8_t IOmap[4096];
        int wkc;
        int err_count = 0;
        int err_iteration_count = 0;

        static constexpr int K_ETHERCAT_ERR_PERIOD = 100;
        static constexpr int K_ETHERCAT_ERR_MAX = 20;
        static constexpr int EC_TIMEOUTMON = 500;
        static constexpr size_t currentgroup = 0;

        void EtherCATStateCheck();
        static constexpr size_t check_cycle = 100;
        size_t check_cnt = 1;
    };
};
