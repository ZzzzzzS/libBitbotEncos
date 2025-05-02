/**
 * @file Encos_joint.h
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief Bitbot Encos joint header file
 * @details Bitbot Encos joint header file, used to manage the joint device.
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "Encos_device.hpp"
#include "atomic"
#include "bus/Encos_bus_msg.h"
#include <tuple>

namespace bitbot
{

    class EncosBus;

    /**
     * @brief Encos电机工作模式
     * @details Encos电机工作模式，包含位置模式、速度模式、力矩模式和运动模式。
     * 位置模式：用于控制电机绝对位置，电机会以指定速度到达指定位置，并尽最大可能保持在该位置。推荐用户在需要电机保持绝对位置时使用。
     * 速度模式: 用于控制电机的速度，电机会以指定速度运动，并尽最大可能保持在该速度。推荐用户在需要电机保持绝对速度时使用。
     * 力矩模式: 用于控制电机的力矩，电机会以指定力矩运动，并尽最大可能保持在该力矩。推荐用户在需要电机保持绝对力矩时使用。
     * 运动模式: 用于控制电机的运动，电机会综合考虑用户的指定位置，速度，前馈力矩，反馈PD等参数，结合内部位置速度环，电流环，
     * 使电机以相对平滑的方式到达用户指定的位置和速度。推荐用户在电机高动态运动的场景下使用。该模式下电机内部PD环为20kHz。
     *
     */
    enum class EncosJointMode : uint32_t
    {
        Position = 0,
        Velocity,
        Torque,
        Motion
    };

    /**
     * @brief Encos电机运行时数据结构体
     *
     */
    struct MotorRuntimeData
    {
        /// @brief 电机当前转速(rad/s)
        std::atomic<float> CurrentVelocity;
        /// @brief 电机目标转速(rad/s)
        std::atomic<float> TargetVelocity;
        /// @brief 电机当前转角(rad)
        std::atomic<float> CurrentPosition;
        /// @brief 电机目标转角(rad)
        std::atomic<float> TargetPosition;
        /// @brief 电机当前电流(A)
        std::atomic<float> CurrentCurrent;
        /// @brief 电机目标电流(A)
        std::atomic<float> TargetCurrent;
        /// @brief 电机当前力矩(Nm)
        std::atomic<float> TargetTorque;

        /// @brief 电机运动模式下位置环的比例系数
        std::atomic<float> Kp;
        /// @brief 电机运动模式下位置环的微分系数
        std::atomic<float> Kd;

        /// @brief 电机在位置/速度模式下的电流限制(A)
        std::atomic<float> CurrentLimit;

        /// @brief 电机当前温度(摄氏度)
        std::atomic<float> MotorTemperature;
        /// @brief 驱动器当前温度(摄氏度)
        std::atomic<float> DriverTemperature;

        /// @brief 设置电机的加速度(rad/s^2)
        std::atomic<float> SettingsAcc;

        /// @brief 设置电机磁链补偿
        std::atomic<float> SettingsMagLinkGain;

        /// @brief 设置电机扰动补偿
        std::atomic<float> SettingsDisturbComp;

        /// @brief 设置电机反馈PD环的比例系数
        std::atomic<float> Settings_FB_KP;

        /// @brief 设置电机反馈PD环的微分系数
        std::atomic<float> Settings_FB_KD;
    };

    /**
     * @brief 电机配置数据结构体
     * @details 电机配置数据结构体，包含电机的配置数据，包括电机的转动方向，位置范围，速度范围，力矩范围等。
     * **注意除电机转动方向外，其他数据均需要和电机手册中指定的范围一致，否则电机无法正常工作。或导致损坏！**
     *
     */
    struct MotorConigurationData
    {
        /// @brief 电机位置环的比例系数范围下限，注意与电机手册一致
        const float KP_MIN;

        /// @brief 电机位置环的比例系数范围上限，注意与电机手册一致
        const float KP_MAX;

        /// @brief 电机位置环的微分系数范围下限，注意与电机手册一致
        const float KD_MIN;

        /// @brief 电机位置环的微分系数范围上限，注意与电机手册一致
        const float KD_MAX;

        /// @brief 电机速度范围下限，注意与电机手册一致
        const float POS_MIN;

        /// @brief 电机速度范围上限，注意与电机手册一致
        const float POS_MAX;

        /// @brief 电机速度范围下限，注意与电机手册一致
        const float SPD_MIN;

        /// @brief 电机速度范围上限，注意与电机手册一致
        const float SPD_MAX;

        /// @brief 电机力矩范围下限，注意与电机手册一致
        const float T_MIN;

        /// @brief 电机力矩范围上限，注意与电机手册一致
        const float T_MAX;

        /// @brief 电机电流范围下限，注意与电机手册一致
        const float I_MIN;

        /// @brief 电机电流范围上限，注意与电机手册一致
        const float I_MAX;

        /// @brief 电机的转动方向，1表示正转，-1表示反转
        const int MOTOR_DIRECTION;

        /**
         * @brief 配置电机的参数
         *
         * @param motor_direction 电机的转动方向，1表示正转，-1表示反转
         * @param kp_range 电机位置环的比例系数范围，注意与电机手册一致
         * @param kd_range 电机位置环的微分系数范围，注意与电机手册一致
         * @param vel_range 电机速度范围，注意与电机手册一致
         * @param pos_range 电机位置范围，注意与电机手册一致
         * @param torque_range 电机力矩范围，注意与电机手册一致
         * @param current_range 电机电流范围，注意与电机手册一致
         */
        MotorConigurationData(int motor_direction, float kp_range,
            float kd_range, float vel_range,
            float pos_range, float torque_range, float current_range)
            : KP_MAX(kp_range),
            KP_MIN(0.0),
            KD_MAX(kd_range),
            KD_MIN(0.0),
            SPD_MAX(vel_range),
            SPD_MIN(-vel_range),
            POS_MAX(pos_range),
            POS_MIN(-pos_range),
            T_MIN(-torque_range),
            T_MAX(torque_range),
            I_MAX(current_range),
            I_MIN(-current_range),
            MOTOR_DIRECTION(motor_direction)
        {
        }
    };

    /**
     * @brief Encos电机类，继承自Encos_CANBusDevice, 用户可以使用该类来管理电机设备。
     * @details Encos电机类，继承自Encos_CANBusDevice, 用户可以使用该类来管理电机设备。
     * 该类提供了电机的基本操作，包括上电，下电，设置目标位置，速度，力矩，设置零位等。
     * 该类还提供了电机的运行时数据，包括电机的当前转速，目标转速，当前转角，目标转角，当前电流，目标电流等。
     * Encos电机有多种工作模式，包括位置模式，速度模式，力矩模式和运动模式。
     * * 位置模式：用于控制电机绝对位置，电机会以指定速度到达指定位置，并尽最大可能保持在该位置。推荐用户在需要电机保持绝对位置时使用。
     * * 速度模式: 用于控制电机的速度，电机会以指定速度运动，并尽最大可能保持在该速度。推荐用户在需要电机保持绝对速度时使用。
     * * 力矩模式: 用于控制电机的力矩，电会以指定力矩运动，并尽最大可能保持在该力矩。推荐用户在需要电机保持绝对力矩时使用。
     * * 运动模式: 用于控制电机的运动，电机会综合考虑用户的指定位置，速度，前馈力矩，反馈PD等参数，结合内部位置速度环，电流环，
     * 使电机以相对平滑的方式到达用户指定的位置和速度。推荐用户在电机高动态运动的场景下使用。该模式下电机内部PD环为20kHz。
     *
     */
    class EncosJoint : public Encos_CANBusDevice
    {
        friend class EncosBus;

    public:
        /**
         * @brief 构造函数
         * @details 构造函数，用户无需关心该函数的实现，该函数由Bitbot Encos内核创建。
         *
         * @param joint_node 电机节点的配置文件
         */
        EncosJoint(const pugi::xml_node& joint_node);

        /**
         * @brief 析构函数
         *
         */
        virtual ~EncosJoint();

    public: // interface

        /**
         * @brief 查询电机是否使能
         * @details 查询电机是否使能。电机使能后才能上电，未使能的电机无法上电但可以读取
         * 电机的运行时数据。电机的使能状态由用户在配置文件中指定。
         *
         * @return true 使能
         * @return false 未使能
         */
        bool isEnable();

        /**
         * @brief 查询电机是否上电。
         * @details 查询电机是否上电。电机上电后才能运动，未上电的电机无法运动。但可以读取
         * 电机的运行时数据。电机的上电状态由内核和总线管理器控制。
         *
         * @return true
         * @return false
         */
        bool isPowerOn();

        /**
         * @brief 设置电机的工作模式
         * @details 设置电机的工作模式。电机的默认工作模式由用户在配置文件中指定。
         * 用户也可以在程序中动态修改电机的工作模式。电机的工作模式包括位置模式，速度模式，力矩模式和运动模式。
         *
         * @param mode 工作模式
         */
        void SetMode(EncosJointMode mode);

        /**
         * @brief 获取电机当前的工作模式
         * @details 获取电机当前的工作模式。电机的工作模式包括位置模式，速度模式，力矩模式和运动模式。
         * @return EncosJointMode 电机当前的工作模式
         */
        EncosJointMode GetMode();

        /**
         * @brief 获取电机的实际位置
         *
         * @return float 电机的实际位置，单位为弧度(rad)
         */
        float GetActualPosition();

        /**
         * @brief 获取电机的实际速度
         *
         * @return float 电机的实际速度，单位为弧度/秒(rad/s)
         */
        float GetActualVelocity();

        /**
         * @brief 获取电机的实际电流
         *
         * @return float 电机的实际电流，单位为安培(A)
         */
        float GetActualCurrent();

        /**
         * @brief 获取电机运动模式下的位置环比例系数
         *
         * @return float 电机运动模式下的位置环比例系数
         */
        float GetMotionKp();

        /**
         * @brief 获取电机运动模式下的位置环微分系数
         *
         * @return float 电机运动模式下的位置环微分系数
         */
        float GetMotionKd();

        /**
         * @brief 获取电机的温度
         *
         * @return std::tuple<float, float> 电机温度和驱动器温度，单位为摄氏度(°C)
         */
        std::tuple<float, float> GetMotorTemperature();

        /**
         * @brief 获取电机的目标位置
         *
         * @return float 电机的目标位置，单位为弧度(rad)
         */
        float GetTargetPosition();

        /**
         * @brief 获取电机的目标速度
         *
         * @return float 电机的目标速度，单位为弧度/秒(rad/s)
         */
        float GetTargetVelocity();

        /**
         * @brief 获取电机的目标力矩
         *
         * @return float 电机的目标力矩，单位为牛米(Nm)
         */
        float GetTargetTorque();

        /**
         * @brief 获取电机的目标电流
         *
         * @return float 电机的目标电流，单位为安培(A)
         */
        float GetTargetCurrent();

        /**
         * @brief 设置电机运动模式下的位置环比例系数
         *
         * @param Kp 电机运动模式下的位置环比例系数
         */
        void SetMotionKp(float Kp);

        /**
         * @brief 设置电机运动模式下的位置环微分系数
         *
         * @param Kd 电机运动模式下的位置环微分系数
         */
        void SetMotionKd(float Kd);

        /**
         * @brief 设置电机的目标位置，该指令可以在位置模式和运动模式下使用。
         * @details 设置电机的目标位置，单位为弧度(rad)。
         * * 在位置模式下，电机会以期望速度达到期望位置，并在电流允许的范围内保持在该位置。
         * 若未指定速度和电流限制，则电机会以配置文件中设定的最大速度和电流限制到达目标位置。
         * * 在运动模式下，该指令会设定电机的目标位置，速度和力矩限制将会被忽略，
         * 用户可结合速度/力矩设置指令指定期望速度和前馈力矩来综合控制电机。
         * * 在其他模式下该指令无法使用。
         *
         * @param position 电机的目标位置，单位为弧度(rad)
         * @param move_velocity 电机的目标速度，单位为弧度/秒(rad/s)，值为0时使用配置文件中的最大速度
         * @param current_limit 电机的电流限制，单位为安培(A)，值为0时使用配置文件中的最大电流
         */
        void SetTargetPosition(float position, float move_velocity = 0, float current_limit = 0);

        /**
         * @brief 设置电机的目标速度，该指令可以在速度模式和运动模式下使用。
         * @details 设置电机的目标速度，单位为弧度/秒(rad/s)。
         * * 在速度模式下，电机会以期望速度运动，并在电流允许的范围内保持在该速度。
         * 若未指定电流限制，则电机会以配置文件中设定的最大电流限制运动。
         * * 在运动模式下，该指令会设定电机的目标速度，力矩限制将会被忽略，
         * 用户可结合位置/力矩设置指令指定期望位置和前馈力矩来综合控制电机。
         * * 在其他模式下该指令无法使用。
         *
         * @param velocity 电机的目标速度，单位为弧度/秒(rad/s)
         * @param current_limit 电机的电流限制，单位为安培(A)，值为0时使用配置文件中的最大电流
         */
        void SetTargetVelocity(float velocity, float current_limit = 0);

        /**
         * @brief 设置电机的目标力矩，该指令可以在力矩模式和运动模式下使用。
         * @details 设置电机的目标力矩，单位为牛米(Nm)。
         * * 在力矩模式下，电机会下发期望力矩运动。
         * * 在运动模式下，该指令会设定电机的前馈力矩，用户可结合位置/速度设置指令指定期望位置和速度来综合控制电机。
         * * 在其他模式下该指令无法使用。
         * @param torque 电机的目标力矩，单位为牛米(Nm)
         */
        void SetTargetTorque(float torque);

        /**
         * @brief 设置电机目标电流，该指令可以在任何模式下使用。
         * @details 设置电机的目标电流，单位为安培(A)。该指令遵循电机的电流限制范围。该指令可以在任何模式下使用。
         *
         * @param current 电机的目标电流，单位为安培(A)
         */
        void SetTargetCurrent(float current);

        /**
         * @brief 设置电机目标运动，该指令仅在运动模式下使用。
         * @details 该指令会设置目标位置，速度和前馈力矩，电机会综合考虑这些参数来运动。
         * 运动模式下电机内部PD环为20kHz。电机内部包含一个位置环来计算电机最终的电流值。
         * 公式为：

          \f[
          I = (Kp \cdot (Position - CurrentPosition) + Kd \cdot (Velocity - CurrentVelocity) + Torque) / torque\_constant
          \f]

         * 其中，I为电机的电流值，Kp和Kd为电机的比例系数和微分系数，Position和Velocity分别为目标位置和速度，
         * Torque为目标力矩，torque_constant为电机的扭矩常数。
         *
         * @param Position 电机的目标位置，单位为弧度(rad)
         * @param Velocity 电机的目标速度，单位为弧度/秒(rad/s)
         * @param Torque 电机的目标力矩，单位为牛米(Nm)
         */
        void SetTargetMotion(float Position, float Velocity = 0, float Torque = 0);
        // void HotfixMotorAcceleration(float value);
        // void HotfixMotorLinkageSpeedKI(float linkage, float KI);
        // void HotfixMotorFeedbackKP_PD(float Kp, float Kd);

        /**
         * @brief 设置电机当前位置为零位
         * @details 设置电机当前位置为零位，该指令会将电机的当前位置设置为零位。
         * 电机设置零位后会有约500ms的延迟，电机会在此期间停止运动，也会使得电机获取出现停滞。
         * 出于安全考虑，该指令设置后将使得电机的目标位置，目标速度和目标力矩均为0。用户在设置零位后需要重新设置目标位置，速度和力矩。
         */
        void ResetMotorPosition();

    private:
        enum class WriteCmdType_e
        {
            SET_ZERO,
            READ_COMM_MODE,
            READ_CAN_ID,
            MOTION_CONTROL,
            POSITION_CONTROL,
            VELOCITY_CONTROL,
            TORQUE_CONTROL,
            CURRENT_CONTROL,
            SETTINGS_MOTOR_ACC,
            SETTINGS_LINK_COMP,
            SETTINGS_FB_KD
        };
        std::atomic<WriteCmdType_e> WriteCmdType__;

    private:
        virtual void UpdateRuntimeData() override final;
        virtual void ReadBus(const CAN_Device_Msg& data) override final;
        virtual void WriteBus(CAN_Device_Msg& data) override final;
        virtual size_t get_EtherCAT_Slave_ID() const override final;
        virtual void get_CAN_IDs(std::vector<size_t>& ids) const override final;

        virtual bool PowerOn() override;
        virtual bool HasPowerCfg() override;
        virtual bool PowerOff() override;

    private:
        EncosJointMode JointMode__;
        bool Enable__;
        bool PowerOn__;
        int isConfig__;
        std::atomic<bool> HighPriorityCommandWriting__;

        size_t Slave_ID__;
        static constexpr size_t Alternative_ID__ = 0x7FF;

        const MotorConigurationData* ConfigData__;
        MotorRuntimeData RuntimeData__;

    private:
        void ProcessErrorCode(uint8_t error_code);
        uint float_to_uint(float x, float x_min, float x_max, int bits);
        float uint_to_float(uint x_int, float x_min, float x_max, int bits);

        void WriteBusSetMotorMotionControl(CAN_Device_Msg& data);
        void WriteBusSetMotorPositionControl(CAN_Device_Msg& data);
        void WriteBusSetMotorVelocityControl(CAN_Device_Msg& data);
        void WriteBusSetMotorCurrentControl(CAN_Device_Msg& data);
        void WriteBusSetMotorTorqueControl(CAN_Device_Msg& data);
        void WriteBusSetMotorDisabledControl(CAN_Device_Msg& data);
        void WriteBusSetZero(CAN_Device_Msg& data);
        void WriteBusReadCommMode(CAN_Device_Msg& data);
        void WriteBusRead_CAN_ID(CAN_Device_Msg& data);
        void WriteBusSettingsMotorAcc(CAN_Device_Msg& data);
        void WriteBusSettingsLinkComp(CAN_Device_Msg& data);
        void WriteBusSettings_FB_KD(CAN_Device_Msg& data);

        void ShiftCommand();
    };
}
