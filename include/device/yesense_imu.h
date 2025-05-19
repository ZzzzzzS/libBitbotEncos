/**
 * @file yesense_imu.h
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief Bitbot Encos Yesense IMU header file
 * @details Bitbot Encos Yesense IMU header file, used to manage the Yesense IMU device.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "atomic"
#include "yesense_sdk/analysis_data.h"
#include "Encos_device.hpp"
#include <fstream>
#include <termios.h>
#include <array>

namespace bitbot
{
    class LimxBus;

    /**
     * @brief IMU运行时数据结构体
     *
     */
    struct ImuRuntimeData
    {
        /// @brief IMU当前滚转角度(rad)
        std::atomic<float> roll = 0;

        /// @brief IMU当前俯仰角度(rad)
        std::atomic<float> pitch = 0;

        /// @brief IMU当前偏航角度(rad)
        std::atomic<float> yaw = 0;

        /// @brief IMU当前x轴加速度(m/s^2)
        std::atomic<float> a_x = 0;

        /// @brief IMU当前y轴加速度(m/s^2)
        std::atomic<float> a_y = 0;

        /// @brief IMU当前z轴加速度(m/s^2)
        std::atomic<float> a_z = 0;

        /// @brief IMU当前x轴角速度(rad/s)
        std::atomic<float> w_x = 0;

        /// @brief IMU当前y轴角速度(rad/s)
        std::atomic<float> w_y = 0;

        /// @brief IMU当前z轴角速度(rad/s)
        std::atomic<float> w_z = 0;

        /// @brief IMU当前温度(°C)
        std::atomic<float> IMU_temp = 0;
    };

    /**
     * @brief IMU数据结构体
     *
     */
    struct ImuData
    {
        /// @brief IMU当前姿态数据
        ImuRuntimeData runtime;
    };

    /**
     * @brief Bitbot Encos Yesense IMU类，继承自Encos_VirtualBusDevice, 用户可以使用该类来管理Yesense IMU设备。
     * @details Bitbot Encos Yesense IMU类，继承自Encos_VirtualBusDevice, 用户可以使用该类来管理Yesense IMU设备。
     * 该类使用了Yesense SDK来实现IMU数据的读取和解析。用户可以使用该类来获取IMU的姿态数据和加速度数据。
     *
     */
    class YesenseIMU : public Encos_VirtualBusDevice
    {
        friend class EncosBus;

    public:
        /**
         * @brief 构造IMU对象，该对象由Bitbot Encos内核创建。用户无需关心该对象的创建过程。
         *
         * @param imu_node IMU节点的配置文件
         */
        YesenseIMU(const pugi::xml_node& imu_node);

        /**
         * @brief 析构IMU对象，该对象由Bitbot Encos内核创建。用户无需关心该对象的析构过程。
         *
         */
        virtual ~YesenseIMU();

        /**
         * @brief 获取IMU滚转角度
         *
         * @return float 单位为弧度(rad)
         */
        float GetRoll();

        /**
         * @brief 获取IMU俯仰角度
         *
         * @return float 单位为弧度(rad)
         */
        float GetPitch();

        /**
         * @brief 获取IMU偏航角度
         *
         * @return float 单位为弧度(rad)
         */
        float GetYaw();

        /**
         * @brief 获取IMU x轴加速度
         *
         * @return float 单位为m/s^2
         */
        float GetAccX();

        /**
         * @brief 获取IMU y轴加速度
         *
         * @return float 单位为m/s^2
         */
        float GetAccY();

        /**
         * @brief 获取IMU z轴加速度
         *
         * @return float 单位为m/s^2
         */
        float GetAccZ();

        /**
         * @brief 获取IMU x轴角速度
         *
         * @return float 单位为rad/s
         */
        float GetGyroX();

        /**
         * @brief 获取IMU y轴角速度
         *
         * @return float 单位为rad/s
         */
        float GetGyroY();

        /**
         * @brief 获取IMU z轴角速度
         *
         * @return float 单位为rad/s
         */
        float GetGyroZ();

        /**
         * @brief 获取IMU温度
         *
         * @return float 单位为摄氏度(°C)
         */
        float GetIMUTemperature();

    private:
        void UpdateRuntimeData() override;
        void UpdateImuDataFromBus();
        void ReadOnce(); // call this function in the bus loop
        void WriteOnce() {};

        std::array<float, 3> cvtEuler(const float* quat);

    private:
        ImuData imu_data_;

        constexpr static int RX_BUF_LEN = 1024;
        unsigned char g_recv_buf[RX_BUF_LEN * 2] = { 0 };
        unsigned short g_recv_buf_idx = 0;
        protocol_info_t g_output_info = { 0 };

        int fd;
        int nread;
        char buffer[RX_BUF_LEN];
        std::string dev;
        struct termios newtio;
        unsigned short cnt = 0;
        int pos = 0;
        speed_t speed = B921600;

        constexpr float r2d = 180.0f / M_PI;
        constexpr float d2r = M_PI / 180.0f;
    };
}