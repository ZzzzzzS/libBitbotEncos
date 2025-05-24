#include "device/Encos_joint.h"
#define _USE_MATH_DEFINES
#include "cmath"
#include "math.h"
#include "iostream"

namespace bitbot
{

    union RV_TypeConvert
    {
        float to_float;
        int to_int;
        unsigned int to_uint;
        uint8_t buf[4];
    };

    EncosJoint::EncosJoint(const pugi::xml_node& joint_node)
        : Encos_CANBusDevice(joint_node),
        JointMode__(EncosJointMode::Motion),
        Enable__(false),
        PowerOn__(false),
        isConfig__(0),
        WriteCmdType__(WriteCmdType_e::TORQUE_CONTROL)
    {
        this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::MOTOR);
        this->type_ = static_cast<uint32_t>(EncosDeviceType::Encos_JOINT);
        monitor_header_.headers = { "status", "mode", "actual_position", "target_position", "actual_velocity", "target_velocity", "actual_current", "target_torque", "motor_temp", "driver_temp" };
        monitor_data_.resize(monitor_header_.headers.size());

        std::string mode;
        ConfigParser::ParseAttribute2s(mode, joint_node.attribute("mode"));
        if (mode.compare("position") == 0)
        {
            this->SetMode(EncosJointMode::Position);
            this->WriteCmdType__.store(WriteCmdType_e::POSITION_CONTROL);
        }
        else if (mode.compare("torque") == 0)
        {
            this->SetMode(EncosJointMode::Torque);
            this->WriteCmdType__.store(WriteCmdType_e::TORQUE_CONTROL);
        }
        else if (mode.compare("velocity") == 0)
        {
            this->SetMode(EncosJointMode::Velocity);
            this->WriteCmdType__.store(WriteCmdType_e::VELOCITY_CONTROL);
        }
        else if (mode.compare("motion") == 0)
        {
            this->SetMode(EncosJointMode::Motion);
            this->WriteCmdType__.store(WriteCmdType_e::MOTION_CONTROL);
        }
        else
        {
            this->logger_->error("Unknown motor mode, fallback to motion mode. Motor mode can only be \"position\", \"torque\", \"velocity\" or \"motion\". Please check your xml file.");
            this->SetMode(EncosJointMode::Motion);
            this->WriteCmdType__.store(WriteCmdType_e::MOTION_CONTROL);
        }

        ConfigParser::ParseAttribute2b(this->Enable__, joint_node.attribute("enable"));

        int id;
        ConfigParser::ParseAttribute2i(id, joint_node.attribute("slave_id"));
        this->Slave_ID__ = static_cast<decltype(this->Slave_ID__)>(id);

        int MotorDirection;
        double kp_range, kd_range, vel_range, pos_range, torque_range, current_range, KT;
        ConfigParser::ParseAttribute2i(MotorDirection, joint_node.attribute("motor_direction"));
        MotorDirection /= std::abs(MotorDirection);
        ConfigParser::ParseAttribute2d(kp_range, joint_node.attribute("kp_range"));
        ConfigParser::ParseAttribute2d(kd_range, joint_node.attribute("kd_range"));
        ConfigParser::ParseAttribute2d(vel_range, joint_node.attribute("vel_range"));
        ConfigParser::ParseAttribute2d(pos_range, joint_node.attribute("pos_range"));
        ConfigParser::ParseAttribute2d(torque_range, joint_node.attribute("torque_range"));
        ConfigParser::ParseAttribute2d(current_range, joint_node.attribute("current_range"));
        ConfigParser::ParseAttribute2d(KT, joint_node.attribute("torque_constant"));

        this->ConfigData__ = new MotorConigurationData(static_cast<float>(MotorDirection),
            static_cast<float>(kp_range),
            static_cast<float>(kd_range),
            static_cast<float>(vel_range),
            static_cast<float>(pos_range),
            static_cast<float>(torque_range),
            static_cast<float>(current_range),
            static_cast<float>(KT)
        );
        this->RuntimeData__.CurrentLimit.store(current_range);

        double kp, kd;
        ConfigParser::ParseAttribute2d(kp, joint_node.attribute("kp"));
        ConfigParser::ParseAttribute2d(kd, joint_node.attribute("kd"));
        this->SetMotionKp(kp);
        this->SetMotionKd(kd);
    }

    EncosJoint::~EncosJoint()
    {
        delete this->ConfigData__;
    }

    bool EncosJoint::isEnable()
    {
        return this->Enable__;
    }
    bool EncosJoint::isPowerOn()
    {
        return this->PowerOn__;
    }

    bool EncosJoint::PowerOn()
    {
        if (this->Enable__)
            this->PowerOn__ = true;
        else
            this->PowerOn__ = false;
        this->RuntimeData__.TargetPosition.store(this->RuntimeData__.CurrentPosition.load());
        this->RuntimeData__.TargetVelocity.store(0);
        this->RuntimeData__.TargetTorque.store(0);
        this->RuntimeData__.TargetCurrent.store(0);
        return this->PowerOn__;
    }

    bool EncosJoint::PowerOff()
    {
        this->PowerOn__ = false;
        return true;
    }

    bool EncosJoint::HasPowerCfg()
    {
        return this->Enable__;
    }

    float EncosJoint::GetActualPosition()
    {
        return this->RuntimeData__.CurrentPosition.load();
    }

    float EncosJoint::GetActualVelocity()
    {
        return this->RuntimeData__.CurrentVelocity.load();
    }

    float EncosJoint::GetActualCurrent()
    {
        return this->RuntimeData__.CurrentCurrent.load();
    }

    float EncosJoint::GetActualTorque()
    {
        return this->RuntimeData__.CurrentCurrent.load() * this->ConfigData__->KT;
    }

    float EncosJoint::GetTargetPosition()
    {
        return this->RuntimeData__.TargetPosition.load();
    }

    float EncosJoint::GetTargetVelocity()
    {
        return this->RuntimeData__.TargetVelocity.load();
    }

    float EncosJoint::GetTargetTorque()
    {
        return this->RuntimeData__.TargetTorque.load();
    }

    float EncosJoint::GetTargetCurrent()
    {
        return this->RuntimeData__.TargetCurrent.load();
    }

    void EncosJoint::SetMode(EncosJointMode mode)
    {
        this->JointMode__ = mode;
        switch (mode)
        {
        case EncosJointMode::Position:
        {
            this->WriteCmdType__ = WriteCmdType_e::POSITION_CONTROL;
            break;
        }
        case EncosJointMode::Velocity:
        {
            this->WriteCmdType__ = WriteCmdType_e::VELOCITY_CONTROL;
            break;
        }
        case EncosJointMode::Torque:
        {
            this->WriteCmdType__ = WriteCmdType_e::TORQUE_CONTROL;
            break;
        }
        case EncosJointMode::Motion:
        {
            this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
            break;
        }
        default:
            this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
            break;
        }
    }
    EncosJointMode EncosJoint::GetMode()
    {
        return this->JointMode__;
    }
    void EncosJoint::SetMotionKp(float kp)
    {
        kp = std::clamp(kp, this->ConfigData__->KP_MIN, this->ConfigData__->KP_MAX);
        this->RuntimeData__.Kp.store(kp);
    }
    void EncosJoint::SetMotionKd(float kd)
    {
        kd = std::clamp(kd, this->ConfigData__->KD_MIN, this->ConfigData__->KD_MAX);
        this->RuntimeData__.Kd.store(kd);
    }
    float EncosJoint::GetMotionKp()
    {
        return this->RuntimeData__.Kp.load();
    }
    float EncosJoint::GetMotionKd()
    {
        return this->RuntimeData__.Kd.load();
    }

    std::tuple<float, float> EncosJoint::GetMotorTemperature()
    {
        return { this->RuntimeData__.MotorTemperature.load(), this->RuntimeData__.DriverTemperature.load() };
    }

    void EncosJoint::SetTargetPosition(float position, float move_velocity, float current_limit)
    {
        if (this->JointMode__ == EncosJointMode::Position)
        {
            position = std::clamp(position, this->ConfigData__->POS_MIN, this->ConfigData__->POS_MAX);
            this->RuntimeData__.TargetPosition.store(position);

            if (move_velocity == 0)
                move_velocity = this->ConfigData__->SPD_MAX;
            else
                move_velocity = std::clamp(move_velocity, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX);
            this->RuntimeData__.TargetVelocity.store(move_velocity);

            if (current_limit == 0)
                current_limit = this->ConfigData__->I_MAX;
            else
                current_limit = std::clamp(current_limit, this->ConfigData__->I_MIN, this->ConfigData__->I_MAX);
            this->RuntimeData__.CurrentLimit.store(current_limit);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::POSITION_CONTROL;
        }
        else if (this->JointMode__ == EncosJointMode::Motion)
        {
            position = std::clamp(position, this->ConfigData__->POS_MIN, this->ConfigData__->POS_MAX);
            this->RuntimeData__.TargetPosition.store(position);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
        }
        else
        {
            std::string msg = "Motor ID: " + std::to_string(this->id_) + " is not in position mode or motion mode, command will be ignored!";
            this->logger_->error(msg);
        }
    }

    void EncosJoint::SetTargetVelocity(float velocity, float current_limit)
    {
        if (this->JointMode__ == EncosJointMode::Velocity)
        {
            velocity = std::clamp(velocity, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX);
            this->RuntimeData__.TargetVelocity.store(velocity);

            if (current_limit == 0)
                current_limit = this->ConfigData__->I_MAX;
            else
                current_limit = std::clamp(current_limit, this->ConfigData__->I_MIN, this->ConfigData__->I_MAX);
            this->RuntimeData__.CurrentLimit.store(current_limit);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::VELOCITY_CONTROL;
        }
        else if (this->JointMode__ == EncosJointMode::Motion)
        {
            velocity = std::clamp(velocity, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX);
            this->RuntimeData__.TargetVelocity.store(velocity);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
        }
        else
        {
            std::string msg = "Motor ID: " + std::to_string(this->id_) + " is not in velocity mode or motion mode, command will be ignored!";
            this->logger_->error(msg);
        }
    }

    void EncosJoint::SetTargetTorque(float torque)
    {
        if (this->JointMode__ == EncosJointMode::Torque)
        {
            torque = std::clamp(torque, this->ConfigData__->T_MIN, this->ConfigData__->T_MAX);
            this->RuntimeData__.TargetTorque.store(torque);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::TORQUE_CONTROL;
        }
        else if (this->JointMode__ == EncosJointMode::Motion)
        {
            torque = std::clamp(torque, this->ConfigData__->T_MIN, this->ConfigData__->T_MAX);
            this->RuntimeData__.TargetTorque.store(torque);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
        }
        else
        {
            std::string msg = "Motor ID: " + std::to_string(this->id_) + " is not in torque mode or motion mode, command will be ignored!";
            this->logger_->error(msg);
        }
    }

    void EncosJoint::SetTargetMotion(float Position, float Velocity, float Torque)
    {
        if (this->JointMode__ == EncosJointMode::Motion) [[likely]]
        {
            Position = std::clamp(Position, this->ConfigData__->POS_MIN, this->ConfigData__->POS_MAX);
            Velocity = std::clamp(Velocity, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX);
            Torque = std::clamp(Torque, this->ConfigData__->T_MIN, this->ConfigData__->T_MAX);

            if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
                this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
        }
        else
        {
            std::string msg = "Motor ID: " + std::to_string(this->id_) + " is not in motion mode, command will be ignored!";
            this->logger_->error(msg);
        }
    }

    void EncosJoint::SetTargetCurrent(float current)
    {
        current = std::clamp(current, this->ConfigData__->I_MIN, this->ConfigData__->I_MAX);
        this->RuntimeData__.TargetCurrent.store(current);

        if (!this->HighPriorityCommandWriting__.load()) // write this command only when high priority command is not writing the bus.
            this->WriteCmdType__ = WriteCmdType_e::CURRENT_CONTROL;
    }

    // void EncosJoint::HotfixMotorAcceleration(float value)
    // {
    //     value = std::clamp(value, 0.0f, 20.0f);
    //     this->RuntimeData__.SettingsAcc.store(value);
    //     this->WriteCmdType__ = WriteCmdType_e::SETTINGS_MOTOR_ACC;
    //     this->HighPriorityCommandWriting__.store(true);
    // }

    // void EncosJoint::HotfixMotorLinkageSpeedKI(float linkage, float KI)
    // {
    //     linkage = std::clamp(linkage, 0.0f, 1.0f);
    //     KI = std::clamp(KI, 0.0f, 1.0f);
    //     this->RuntimeData__.SettingsMagLinkGain.store(linkage);
    //     this->RuntimeData__.SettingsDisturbComp.store(KI);
    //     this->WriteCmdType__ = WriteCmdType_e::SETTINGS_LINK_COMP;
    //     this->HighPriorityCommandWriting__.store(true);
    // }

    // void EncosJoint::HotfixMotorFeedbackKP_PD(float Kp, float Kd)
    // {
    //     Kp = std::clamp(Kp, 0.0f, 1.0f);
    //     Kd = std::clamp(Kd, 0.0f, 1.0f);
    //     this->RuntimeData__.Settings_FB_KP.store(Kp);
    //     this->RuntimeData__.Settings_FB_KD.store(Kd);
    //     this->WriteCmdType__ = WriteCmdType_e::SETTINGS_FB_KD;
    //     this->HighPriorityCommandWriting__.store(true);
    // }

    void EncosJoint::ResetMotorPosition()
    {
        this->WriteCmdType__ = WriteCmdType_e::SET_ZERO;
        this->HighPriorityCommandWriting__.store(true);
    }

    void EncosJoint::UpdateRuntimeData()
    {
        constexpr float r2d = 180.0f / M_PI;

        if (this->Enable__)
        {
            this->monitor_data_[0] = (this->PowerOn__ == true) ? 1.0 : 0.0;
        }
        else
        {
            this->monitor_data_[0] = -1.0;
        }

        this->monitor_data_[1] = static_cast<uint8_t>(this->JointMode__);
        this->monitor_data_[2] = this->RuntimeData__.CurrentPosition.load() * r2d;
        this->monitor_data_[3] = this->RuntimeData__.TargetPosition.load() * r2d;
        this->monitor_data_[4] = this->RuntimeData__.CurrentVelocity.load();
        this->monitor_data_[5] = this->RuntimeData__.TargetVelocity.load();
        this->monitor_data_[6] = this->RuntimeData__.CurrentCurrent.load();
        this->monitor_data_[7] = this->RuntimeData__.TargetTorque.load();
        this->monitor_data_[8] = this->RuntimeData__.MotorTemperature.load();
        this->monitor_data_[9] = this->RuntimeData__.DriverTemperature.load();
    }

    void EncosJoint::ProcessErrorCode(uint8_t error_code)
    {
        switch (error_code)
        {
        case 0x00:
            break;
        case 0x01:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x01, motor over temperature.");
            break;
        case 0x02:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x02, motor over current.");
            break;
        case 0x03:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x03, motor under voltage.");
            break;
        case 0x04:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x04, motor encoder error.");
            break;
        case 0x05:
            break;
        case 0x06:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x06, motor break voltage too high.");
            break;
        case 0x07:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: 0x07, motor driver error.");
            break;
        default:
            this->logger_->error(std::string("Motor ID: ") + std::to_string(this->id_) + " receive error code: " + std::to_string(error_code) + ", unknown error.");
            break;
        }
    }

    void EncosJoint::ReadBus(const CAN_Device_Msg& data)
    {
        if (data.dlc == 0) [[unlikely]]
            return;

            if (data.id == this->id_) [[likely]]
            {
                int ack_status = data.data[0] >> 5;
                uint8_t error_code = data.data[0] & 0x1F;
                this->ProcessErrorCode(error_code);

                int pos_int = 0;
                int spd_int = 0;
                int cur_int = 0;

                constexpr float deg2rad = M_PI / 180;
                constexpr float rpm2rads = 2 * M_PI / 60;

                switch (ack_status)
                {
                case 1:
                {
                    pos_int = data.data[1] << 8 | data.data[2];
                    spd_int = data.data[3] << 4 | (data.data[4] & 0xF0) >> 4;
                    cur_int = (data.data[4] & 0x0F) << 8 | data.data[5];
                    float angle = this->uint_to_float(pos_int, this->ConfigData__->POS_MIN, this->ConfigData__->POS_MAX, 16) * this->ConfigData__->MOTOR_DIRECTION;
                    float vel = this->uint_to_float(spd_int, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX, 12) * this->ConfigData__->MOTOR_DIRECTION;
                    float current = this->uint_to_float(cur_int, this->ConfigData__->I_MIN, this->ConfigData__->I_MAX, 12);
                    float motor_temp = (static_cast<float>(data.data[6] - 50)) / 2.0;
                    float mos_temp = (static_cast<float>(data.data[7] - 50)) / 2.0;

                    this->RuntimeData__.CurrentPosition.store(angle);
                    this->RuntimeData__.CurrentVelocity.store(vel);
                    this->RuntimeData__.CurrentCurrent.store(current);
                    this->RuntimeData__.MotorTemperature.store(motor_temp);
                    this->RuntimeData__.DriverTemperature.store(mos_temp);
                    break;
                }

                case 2:
                {
                    RV_TypeConvert rv_type_convert;
                    rv_type_convert.buf[0] = data.data[4];
                    rv_type_convert.buf[1] = data.data[3];
                    rv_type_convert.buf[2] = data.data[2];
                    rv_type_convert.buf[3] = data.data[1];
                    float pos = rv_type_convert.to_float * deg2rad * this->ConfigData__->MOTOR_DIRECTION;
                    int16_t cur_int = data.data[5] << 8 | data.data[6];

                    this->RuntimeData__.CurrentPosition.store(pos);
                    this->RuntimeData__.CurrentCurrent.store(static_cast<float>(cur_int) / 100.0f);
                    this->RuntimeData__.CurrentVelocity.store(0);
                    this->RuntimeData__.DriverTemperature.store(0);
                    this->RuntimeData__.MotorTemperature.store((data.data[7] - 50.0) / 2.0);
                    break;
                }

                case 3:
                {
                    RV_TypeConvert rv_type_convert;
                    rv_type_convert.buf[0] = data.data[4];
                    rv_type_convert.buf[1] = data.data[3];
                    rv_type_convert.buf[2] = data.data[2];
                    rv_type_convert.buf[3] = data.data[1];
                    float vel = rv_type_convert.to_float * rpm2rads * this->ConfigData__->MOTOR_DIRECTION;
                    int16_t cur_int = data.data[5] << 8 | data.data[6];

                    this->RuntimeData__.CurrentPosition.store(0);
                    this->RuntimeData__.CurrentCurrent.store(static_cast<float>(cur_int) / 100.0f);
                    this->RuntimeData__.CurrentVelocity.store(vel);
                    this->RuntimeData__.DriverTemperature.store(0);
                    this->RuntimeData__.MotorTemperature.store((data.data[7] - 50.0) / 2.0);
                    break;
                }
                case 4:
                {
                    if (data.dlc != 3)
                        return;
                    if (data.data[2] == 0x01)
                    {
                        std::string msg = "Motor ID: " + std::to_string(this->id_) + " set command " + std::to_string(data.data[1]) + " succeed.";
                        this->logger_->info(msg);
                    }
                    else
                    {
                        std::string msg = "Motor ID: " + std::to_string(this->id_) + " set command " + std::to_string(data.data[1]) + " failed.";
                        this->logger_->error(msg);
                    }
                }
                case 5:
                    // TODO: add unpack code for message 5
                    break;

                default:
                    break;
                }
            }
            else if (data.id == this->Alternative_ID__)
            {

                if (data.data[2] != 0x01) // not response frame
                    return;

                if ((data.data[0] == 0xff) && (data.data[1] == 0xFF)) // inquire success
                {
                    uint16_t motor_id = data.data[3] << 8 | data.data[4];
                    if (motor_id == this->id_)
                    {
                        std::string msg = "Motor ID: " + std::to_string(motor_id) + " is online.";
                        this->logger_->info(msg);
                    }
                }
                else if ((data.data[0] == 0x80) && (data.data[1] == 0x80)) // inquire failed
                {
                    // motor process error
                }
                else if ((data.data[0] == 0x7F) && (data.data[1] == 0x7F)) // reset ID succeed
                {
                    std::string msg = "Motor ID: " + std::to_string(this->id_) + " is set to 1, however is strongly NOT recommend use motor reset command in Bitbot Encos!";
                    this->logger_->warn(msg);
                }
                else
                {
                    uint16_t motor_id = data.data[0] << 8 | data.data[1];
                    uint8_t motor_fbd = data.data[3];
                    if (motor_id == this->id_)
                    {
                        switch (motor_fbd)
                        {
                        case 0x01:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " is set in auto reply mode. however, it is strongly NOT recommend use motor auto reply mode in Bitbot Encos!";
                            this->logger_->warn(msg);
                            break;
                        }
                        case 0x02:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " is set in query reply mode.";
                            this->logger_->info(msg);
                            break;
                        }
                        case 0x80:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " receive code: 0x80, query failed.";
                            this->logger_->error(msg);
                            break;
                        }
                        case 0x03:
                        {
                            static std::chrono::high_resolution_clock::time_point last_call;
                            std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
                            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call).count() > 1000)
                            {
                                std::string msg = "Motor ID: " + std::to_string(motor_id) + " sets current position as zero.";
                                this->logger_->info(msg);
                            }
                            last_call = now;

                            break;
                        }
                        case 0x04:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " has been updated, remember to update the ID in Bitbot configuration file!";
                            this->logger_->info(msg);
                            break;
                        }
                        case 0x00:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " receive code: 0x00， command failed.";
                            this->logger_->error(msg);
                            break;
                        }
                        default:
                        {
                            std::string msg = "Motor ID: " + std::to_string(motor_id) + " receive unknown command: " + std::to_string(motor_fbd);
                            this->logger_->error(msg);
                        }
                        break;
                        }
                    }
                }
            }
            else
            {
                /* code */
            }
    }

    void EncosJoint::ShiftCommand()
    {
        switch (this->JointMode__)
        {
        case EncosJointMode::Position:
        {
            this->WriteCmdType__ = WriteCmdType_e::POSITION_CONTROL;
            break;
        }
        case EncosJointMode::Velocity:
        {
            this->WriteCmdType__ = WriteCmdType_e::VELOCITY_CONTROL;
            break;
        }
        case EncosJointMode::Torque:
        {
            this->WriteCmdType__ = WriteCmdType_e::TORQUE_CONTROL;
            break;
        }
        case EncosJointMode::Motion:
        {
            this->WriteCmdType__ = WriteCmdType_e::MOTION_CONTROL;
            break;
        }
        }
    }

    void EncosJoint::WriteBus(CAN_Device_Msg& data)
    {
        constexpr uint32_t CFG_BUS_WAIT = 500;
        this->HighPriorityCommandWriting__.store(false);
        if (this->isConfig__ == 0) [[likely]]
        {
            data.id = this->id_;
            switch (this->WriteCmdType__.load())
            {
            case WriteCmdType_e::SET_ZERO:
            {
                this->WriteBusSetZero(data);
                this->isConfig__ = CFG_BUS_WAIT;
                this->ShiftCommand();
                this->RuntimeData__.TargetTorque.store(0);
                this->RuntimeData__.TargetVelocity.store(0);
                this->RuntimeData__.TargetPosition.store(0);
                break;
            }
            case WriteCmdType_e::READ_COMM_MODE:
            {
                this->WriteBusReadCommMode(data);
                this->ShiftCommand();
                break;
            }
            case WriteCmdType_e::READ_CAN_ID:
            {
                this->WriteBusRead_CAN_ID(data);
                this->ShiftCommand();
                break;
            }
            case WriteCmdType_e::MOTION_CONTROL:
            {
                this->WriteBusSetMotorMotionControl(data);
                break;
            }
            case WriteCmdType_e::POSITION_CONTROL:
            {
                this->WriteBusSetMotorPositionControl(data);
                break;
            }
            case WriteCmdType_e::VELOCITY_CONTROL:
            {
                this->WriteBusSetMotorVelocityControl(data);
                break;
            }
            case WriteCmdType_e::TORQUE_CONTROL:
            {
                this->WriteBusSetMotorTorqueControl(data);
                break;
            }
            case WriteCmdType_e::CURRENT_CONTROL:
            {
                this->WriteBusSetMotorCurrentControl(data);
                break;
            }
            case WriteCmdType_e::SETTINGS_MOTOR_ACC:
            {
                this->WriteBusSettingsMotorAcc(data);
                this->ShiftCommand();
                break;
            }
            case WriteCmdType_e::SETTINGS_LINK_COMP:
            {
                this->WriteBusSettingsLinkComp(data);
                this->ShiftCommand();
                break;
            }
            case WriteCmdType_e::SETTINGS_FB_KD:
            {
                this->WriteBusSettings_FB_KD(data);
                this->ShiftCommand();
                break;
            }
            default:
            {
                std::string msg = "Motor ID: " + std::to_string(this->id_) + " Parse Command Failed, fatel error, please report this bug to maintainers! Parser code: " + std::to_string(static_cast<uint32_t>(this->WriteCmdType__.load()));
                this->logger_->critical(msg);
            }
            }
        }
        else
        {
            data.dlc = 0;
            data.rtr = 0;
            for (size_t i = 0; i < 8; i++)
            {
                data.data[i] = 0;
            }
            this->isConfig__--;
        }
    }
    size_t EncosJoint::get_EtherCAT_Slave_ID() const
    {
        return this->Slave_ID__;
    }

    void EncosJoint::get_CAN_IDs(std::vector<size_t>& ids) const
    {
        ids.resize(2);
        ids[0] = this->id_;
        ids[1] = this->Alternative_ID__;
    }

    uint EncosJoint::float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (uint)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float EncosJoint::uint_to_float(uint x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    void EncosJoint::WriteBusSetMotorMotionControl(CAN_Device_Msg& data)
    {
        if (this->Enable__ && this->PowerOn__)
        {
            data.dlc = 8;
            data.rtr = 0;
            data.id = this->id_;

            int kp_int;
            int kd_int;
            int pos_int;
            int spd_int;
            int tor_int;

            kp_int = this->float_to_uint(this->RuntimeData__.Kp.load(), this->ConfigData__->KP_MIN, this->ConfigData__->KP_MAX, 12);
            kd_int = this->float_to_uint(this->RuntimeData__.Kd.load(), this->ConfigData__->KD_MIN, this->ConfigData__->KD_MAX, 9);
            pos_int = this->float_to_uint(this->RuntimeData__.TargetPosition.load() * this->ConfigData__->MOTOR_DIRECTION, this->ConfigData__->POS_MIN, this->ConfigData__->POS_MAX, 16);
            spd_int = this->float_to_uint(this->RuntimeData__.TargetVelocity.load() * this->ConfigData__->MOTOR_DIRECTION, this->ConfigData__->SPD_MIN, this->ConfigData__->SPD_MAX, 12);
            tor_int = this->float_to_uint(this->RuntimeData__.TargetTorque.load() * this->ConfigData__->MOTOR_DIRECTION, this->ConfigData__->T_MIN, this->ConfigData__->T_MAX, 12);

            data.data[0] = 0x00 | (kp_int >> 7);                             // kp5
            data.data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
            data.data[2] = kd_int & 0xFF;
            data.data[3] = pos_int >> 8;
            data.data[4] = pos_int & 0xFF;
            data.data[5] = spd_int >> 4;
            data.data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
            data.data[7] = tor_int & 0xff;
        }
        else
        {
            return this->WriteBusSetMotorDisabledControl(data);
        }
    }

    void EncosJoint::WriteBusSetMotorPositionControl(CAN_Device_Msg& data)
    {
        if (this->Enable__ && this->PowerOn__)
        {
            data.rtr = 0;
            data.dlc = 8;
            data.id = this->id_;

            constexpr float r2d = 180.0f / M_PI;
            constexpr float rads2rpm = 60.0f / (2 * M_PI);
            RV_TypeConvert rv_type_convert;
            rv_type_convert.to_float = this->RuntimeData__.TargetPosition.load() * this->ConfigData__->MOTOR_DIRECTION * r2d;
            float vel = std::abs(this->RuntimeData__.TargetVelocity.load()) * rads2rpm;
            uint16_t spd = static_cast<uint16_t>(vel * 10.0f);
            uint16_t cur = static_cast<uint16_t>(this->RuntimeData__.CurrentLimit.load() * 10.0f);
            uint16_t ack_status = 1;

            data.data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
            data.data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
            data.data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
            data.data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
            data.data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
            data.data[5] = (spd & 0x3FC) >> 2;
            data.data[6] = (spd & 0x03) << 6 | (cur >> 6);
            data.data[7] = (cur & 0x3F) << 2 | ack_status;
        }
        else
        {
            return this->WriteBusSetMotorDisabledControl(data);
        }
    }

    void EncosJoint::WriteBusSetMotorVelocityControl(CAN_Device_Msg& data)
    {
        if (this->Enable__ && this->PowerOn__)
        {
            data.rtr = 0;
            data.dlc = 7;
            data.id = this->id_;

            constexpr float rads2rpm = 60.0f / (2 * M_PI);
            RV_TypeConvert rv_type_convert;
            rv_type_convert.to_float = this->RuntimeData__.TargetVelocity.load() * this->ConfigData__->MOTOR_DIRECTION * rads2rpm;
            uint16_t cur = static_cast<uint16_t>(this->RuntimeData__.CurrentLimit.load() * 10.0f);
            uint16_t ack_status = 1;

            data.data[0] = 0x40 | ack_status;
            data.data[1] = rv_type_convert.buf[3];
            data.data[2] = rv_type_convert.buf[2];
            data.data[3] = rv_type_convert.buf[1];
            data.data[4] = rv_type_convert.buf[0];
            data.data[5] = cur >> 8;
            data.data[6] = cur & 0xff;
        }
        else
        {
            return this->WriteBusSetMotorDisabledControl(data);
        }
    }

    void EncosJoint::WriteBusSetMotorCurrentControl(CAN_Device_Msg& data)
    {
        if (this->Enable__ && this->PowerOn__)
        {
            data.id = this->id_;
            data.dlc = 3;
            data.rtr = 0;
            constexpr uint8_t ack_status = 0x01;
            constexpr uint8_t ctrl_status = 0x00;

            int16_t cur_tor = this->RuntimeData__.TargetCurrent.load() * 100.0;
            data.data[0] = 0x60 | ctrl_status << 2 | ack_status;
            data.data[1] = cur_tor >> 8;
            data.data[2] = cur_tor & 0xff;
        }
        else
        {
            return this->WriteBusSetMotorDisabledControl(data);
        }
    }

    void EncosJoint::WriteBusSetMotorTorqueControl(CAN_Device_Msg& data)
    {
        if (this->Enable__ && this->PowerOn__)
        {
            data.id = this->id_;
            data.dlc = 3;
            data.rtr = 0;
            constexpr uint8_t ack_status = 0x01;
            constexpr uint8_t ctrl_status = 0x01;

            int16_t cur_tor = this->RuntimeData__.TargetTorque.load() * 100.0;
            data.data[0] = 0x60 | ctrl_status << 2 | ack_status;
            data.data[1] = cur_tor >> 8;
            data.data[2] = cur_tor & 0xff;
        }
        else
        {
            return this->WriteBusSetMotorDisabledControl(data);
        }
    }

    void EncosJoint::WriteBusSetMotorDisabledControl(CAN_Device_Msg& data)
    {

        data.id = this->id_;
        data.dlc = 3;
        data.rtr = 0;
        constexpr uint8_t ack_status = 0x01;
        constexpr uint8_t ctrl_status = 0x00;
        constexpr int16_t cur_tor = 0x00;
        data.data[0] = 0x60 | ctrl_status << 2 | ack_status;
        data.data[1] = cur_tor >> 8;
        data.data[2] = cur_tor & 0xff;
    }

    void EncosJoint::WriteBusSetZero(CAN_Device_Msg& data)
    {
        data.id = this->Alternative_ID__;
        data.rtr = 0;
        data.dlc = 4;
        uint16_t motor_id = this->id_;
        data.data[0] = motor_id >> 8;
        data.data[1] = motor_id & 0xff;
        data.data[2] = 0x00;
        data.data[3] = 0x03;
    }

    void EncosJoint::WriteBusReadCommMode(CAN_Device_Msg& data)
    {
        data.rtr = 0;
        data.id = this->Alternative_ID__;
        data.dlc = 4;

        uint16_t motor_id = this->id_;
        data.data[0] = motor_id >> 8;
        data.data[1] = motor_id & 0xff;
        data.data[2] = 0x00;
        data.data[3] = 0x81;
    }

    void EncosJoint::WriteBusRead_CAN_ID(CAN_Device_Msg& data)
    {
        data.rtr = 0;
        data.id = this->Alternative_ID__;
        data.dlc = 4;

        data.data[0] = 0xFF;
        data.data[1] = 0xFF;
        data.data[2] = 0x00;
        data.data[3] = 0x82;
    }

    void EncosJoint::WriteBusSettingsMotorAcc(CAN_Device_Msg& data)
    {
        data.dlc = 4;
        data.rtr = 0;
        data.id = this->id_;

        uint16_t acc = this->RuntimeData__.SettingsAcc.load() * 100.0;
        constexpr uint8_t ack_status = 0x01;
        data.data[0] = 0xC0 | ack_status;
        data.data[1] = 0x01;
        data.data[2] = acc >> 8;
        data.data[3] = acc & 0xff;
    }

    void EncosJoint::WriteBusSettingsLinkComp(CAN_Device_Msg& data)
    {
        data.dlc = 6;
        data.rtr = 0;
        data.id = this->id_;

        uint16_t linkage = this->RuntimeData__.SettingsMagLinkGain.load() * 10000.0;
        uint16_t speedKI = this->RuntimeData__.SettingsDisturbComp.load() * 10000.0;

        constexpr uint8_t ack_status = 0x01;
        data.data[0] = 0xC0 | ack_status;
        data.data[1] = 0x02;
        data.data[2] = linkage >> 8;
        data.data[3] = linkage & 0xff;
        data.data[4] = speedKI >> 8;
        data.data[5] = speedKI & 0xff;
    }

    void EncosJoint::WriteBusSettings_FB_KD(CAN_Device_Msg& data)
    {
        data.rtr = 0;
        data.dlc = 6;
        data.id = this->id_;

        uint16_t fdbKP = this->RuntimeData__.Settings_FB_KP * 10000.0;
        uint16_t fdbKD = this->RuntimeData__.Settings_FB_KD * 10000.0;
        constexpr uint8_t ack_status = 0x01;
        data.data[0] = 0xC0 | ack_status;
        data.data[1] = 0x03;
        data.data[2] = fdbKP >> 8;
        data.data[3] = fdbKP & 0xff;
        data.data[4] = fdbKD >> 8;
        data.data[5] = fdbKD & 0xff;
    }
};