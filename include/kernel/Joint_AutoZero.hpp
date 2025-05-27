/**
 * @file Joint_AutoZero.hpp
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief JointAutoZero类用于自动归零电机位置
 * @details JointAutoZero类用于自动归零电机位置，支持多个电机组的归零操作。用户可以通过配置文件指定每个电机的归零参数。
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "bus/Encos_bus.h"
#include "device/Encos_joint.h"
#include "bitbot_kernel/utils/logger.h"

namespace bitbot
{
    class JointAutoZero
    {
    public:
        /**
         * @brief JointResetStates枚举类
         *
         */
        enum class JointResetStates
        {
            /// @brief 等待状态
            Waiting,

            /// @brief 应用力矩状态
            ApplyingTorque,

            /// @brief 归零状态
            Resetting,

            /// @brief 归零完成状态
            Finished,

            /// @brief 停止状态
            Stopped
        };

    public:
        /**
         * @brief JointAutoZero构造函数
         *
         * @param reset_node xml节点，包含归零配置
         * @param period 内核运行周期时间
         * @param logger 日志记录器指针
         * @param joints 电机列表，包含所有的电机
         */
        JointAutoZero(const pugi::xml_node &reset_node, double period, SpdLoggerSharedPtr logger, std::vector<EncosJoint *> &joints)
        {
            this->logger__ = logger;
            this->current_state__ = JointResetStates::Waiting;

            pugi::xml_node reset_group_node = reset_node.child("resetter");
            while (reset_group_node != nullptr)
            {
                pugi::xml_node resetter_node = reset_group_node;
                joint_groups__.emplace_back(std::vector<EncosJoint *>());
                joint_ranges__.emplace_back(std::vector<std::pair<float, float>>());
                reset_torque__.emplace_back(std::vector<float>());
                edge_position__.emplace_back(std::vector<float>());
                reset_velocities__.emplace_back(std::vector<float>());

                double reset_period;
                ConfigParser::ParseAttribute2d(reset_period, reset_group_node.attribute("reset_period"));
                this->reset_period_count__.emplace_back(static_cast<uint64_t>(reset_period / period));

                std::string joint_name;
                double joint_lower_limit, joint_upper_limit, joint_reset_torque, joint_reset_velocity;
                ConfigParser::ParseAttribute2s(joint_name, resetter_node.attribute("name"));
                ConfigParser::ParseAttribute2d(joint_lower_limit, resetter_node.attribute("joint_lower_limit"));
                ConfigParser::ParseAttribute2d(joint_upper_limit, resetter_node.attribute("joint_upper_limit"));
                ConfigParser::ParseAttribute2d(joint_reset_torque, resetter_node.attribute("joint_reset_torque"));
                ConfigParser::ParseAttribute2d(joint_reset_velocity, resetter_node.attribute("joint_reset_velocity"));
                EncosJoint *joint = GetJointById(joint_name, joints);

                if (joint == nullptr)
                {
                    this->logger__->error("JointAutoZero: joint {} not found", joint_name);
                    throw std::runtime_error("JointAutoZero: joint not found");
                }
                joints__.emplace_back(joint);

                joint_groups__.back().emplace_back(joint);
                joint_ranges__.back().emplace_back(std::make_pair(joint_lower_limit, joint_upper_limit));
                reset_torque__.back().emplace_back(joint_reset_torque);
                edge_position__.back().emplace_back(0);
                reset_velocities__.back().emplace_back(joint_reset_velocity);

                reset_group_node = reset_group_node.next_sibling("resetter");
            }
        }

        /**
         * @brief 开始归零操作
         *
         * @param period_count 归零操作的周期计数
         * @return true 开始归零操作成功
         * @return false 开始归零操作失败，可能是当前状态不允许开始归零
         */
        bool StartReset(uint64_t period_count)
        {
            if (current_state__ == JointResetStates::Waiting || current_state__ == JointResetStates::Stopped)
            {
                this->current_state__ = JointResetStates::ApplyingTorque;
                this->current_group_index__ = 0;
                this->next_state_period_count__ = period_count + reset_period_count__[current_group_index__];
                this->logger__->info("\n\nJointAutoZero: Applying constant torque in joint 0");
                for (auto joint : joints__)
                {
                    joint->SetMode(EncosJointMode::Torque);
                    joint->SetTargetTorque(0);
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        /**
         * @brief 停止归零操作
         *
         * @return true 归零操作停止成功
         * @return false 归零操作停止失败，可能是当前状态不允许停止归零
         */
        bool StopReset()
        {
            if (current_state__ == JointResetStates::Waiting || this->current_state__ == JointResetStates::Finished)
            {
                this->logger__->info("JointAutoZero: reset not start or finished");
                return false;
            }
            else if (current_state__ == JointResetStates::Stopped)
            {
                this->logger__->info("JointAutoZero: reset already stopped");
                return false;
            }
            else
            {
                this->current_state__ = JointResetStates::Stopped;
                this->logger__->info("JointAutoZero: reset stopped!!!");
                for (auto joint : joints__)
                {
                    joint->SetMode(EncosJointMode::Torque);
                    joint->SetTargetTorque(0);
                }
                return true;
            }
        }

        /**
         * @brief 重置状态
         *
         * @param period_count 归零操作的周期计数
         */
        void ResetState(uint64_t period_count)
        {
            // std::cout << "p_cnt:" << period_count << " target:" << next_state_period_count__ << " state:" << static_cast<int>(current_state__) << std::endl;
            switch (current_state__)
            {
            case JointResetStates::Waiting:
                break;
            case JointResetStates::ApplyingTorque:
                if (period_count >= next_state_period_count__)
                {
                    this->current_state__ = JointResetStates::Resetting;
                    next_state_period_count__ = period_count + reset_period_count__[current_group_index__];
                    for (auto joint : joints__)
                    {
                        joint->SetMode(EncosJointMode::Torque);
                        joint->SetTargetTorque(0);
                    }

                    std::string info = "JointAutoZero: resetting zero for joint ";
                    info += std::to_string(current_group_index__);
                    this->logger__->info(info);
                }
                break;
            case JointResetStates::Resetting:
                if (period_count >= next_state_period_count__)
                {
                    this->EventSetZero();
                    if (this->current_group_index__ == joint_groups__.size() - 1)
                    {
                        this->current_state__ = JointResetStates::Finished;
                        this->logger__->info("JointAutoZero: resetting zero finished");
                    }
                    else
                    {
                        this->current_state__ = JointResetStates::ApplyingTorque;
                        this->current_group_index__++;
                        this->next_state_period_count__ = period_count + reset_period_count__[current_group_index__];

                        std::string info = "\n\nJointAutoZero: Applying constant torque in joint ";
                        info += std::to_string(current_group_index__);
                        this->logger__->info(info);
                    }

                    // for (auto joint : joints__)
                    // {
                    //     joint->SetMode(EncosJointMode::Torque);
                    //     joint->SetTargetTorque(0);
                    // }
                }
                break;
            case JointResetStates::Finished:
                break;
            case JointResetStates::Stopped:
                break;
            }

            switch (current_state__)
            {
            case JointResetStates::ApplyingTorque:
                StateApplyTorque();
                break;
            case JointResetStates::Resetting:
                StateGotoZero();
                break;
            case JointResetStates::Finished:
                StateKeepZero();
                break;
            default:
                StateOther();
                break;
            }
        }

    private:
        void StateApplyTorque()
        {
            for (size_t i = 0; i < joint_groups__[current_group_index__].size(); i++)
            {
                joint_groups__[current_group_index__][i]->SetMode(EncosJointMode::Torque);
                joint_groups__[current_group_index__][i]->SetTargetTorque(reset_torque__[current_group_index__][i]);
                edge_position__[current_group_index__][i] = joint_groups__[current_group_index__][i]->GetActualPosition();
            }
        }

        void EventSetZero()
        {
            for (size_t i = 0; i < joint_groups__[current_group_index__].size(); i++)
            {
                this->logger__->info("JointAutoZero: joint {} zero point error= {}", joint_groups__[current_group_index__][i]->Name(), joint_groups__[current_group_index__][i]->GetActualPosition() * this->r2d);
                joint_groups__[current_group_index__][i]->ResetMotorPosition();
            }
        }

        void StateGotoZero()
        {
            for (size_t i = 0; i < joint_groups__[current_group_index__].size(); i++)
            {
                joint_groups__[current_group_index__][i]->SetMode(EncosJointMode::Position);

                float target_position = reset_torque__[current_group_index__][i] > 0 ? joint_ranges__[current_group_index__][i].second : joint_ranges__[current_group_index__][i].first;
                target_position = edge_position__[current_group_index__][i] - target_position;

                joint_groups__[current_group_index__][i]->SetTargetPosition(target_position, reset_velocities__[current_group_index__][i]);
            }
        }

        void StateKeepZero()
        {
            for (auto joint : joints__)
            {
                joint->SetMode(EncosJointMode::Position);
                joint->SetTargetPosition(0);
            }
        }

        void StateOther()
        {
            for (auto joint : joints__)
            {
                joint->SetMode(EncosJointMode::Torque);
                joint->SetTargetTorque(0);
            }
        }

        EncosJoint *GetJointById(std::string name, const std::vector<EncosJoint *> &joints)
        {
            for (auto joint : joints)
            {
                if (joint->Name() == name)
                {
                    return joint;
                }
            }
            return nullptr;
        }

    private:
        SpdLoggerSharedPtr logger__;

        std::vector<EncosJoint *> joints__;

        std::vector<std::vector<EncosJoint *>> joint_groups__;
        std::vector<std::vector<std::pair<float, float>>> joint_ranges__; // lower upper
        std::vector<std::vector<float>> reset_torque__;
        std::vector<std::vector<float>> reset_velocities__;

        std::vector<uint64_t> reset_period_count__;

        size_t current_group_index__;
        uint64_t next_state_period_count__;
        JointResetStates current_state__ = JointResetStates::Waiting;
        std::vector<std::vector<float>> edge_position__;

        static constexpr float r2d = 180 / M_PI;
    };
};