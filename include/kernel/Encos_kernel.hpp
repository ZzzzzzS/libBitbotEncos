/**
 * @file Encos_kernel.hpp
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief Bitbot Encos kernel header file
 * @details Bitbot Encos kernel header file, used to manage the kernel and handle the kernel events.
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "bitbot_kernel/kernel/kernel.hpp"
#include "bus/Encos_bus.h"
#include "string"
#include "iostream"
#include "Joint_AutoZero.hpp"

namespace bitbot
{
    /// @brief Bitbot Encos 内核事件类型，继承自EventId
    enum class EncosKernelEvent : EventId
    {
        POWER_ON = 100,
        START_RESET_ZERO,
        STOP_RESET_ZERO
    };

    /// @brief Bitbot Encos 内核状态类型，继承自StateId
    enum class EncosKernelState : StateId
    {
        POWER_ON_FINISH = 100,
        AUTO_ZEROING
    };

    /**
     * @brief Bitbot Encos 内核类型，继承自KernelTpl
     * @details Bitbot Encos 内核类型，继承自KernelTpl， 用于处理Bitbot状态机相关的事件。
     * 关于Bitbot内核的详细信息请参阅https://bitbot.lmy.name/docs/bitbot-kernel-intro
     *
     * @tparam UserData 用户自定义数据类型，该类型将在内核运行时回调函数中传递给用户。
     */
    template <typename UserData>
    class EncosKernel : public KernelTpl<EncosKernel<UserData>, EncosBus, UserData>
    {
    public:
        /**
         * @brief 创建一个EncosKernel对象
         * @details BitbotEncos使用和Bitbot内核相同的编程模型，
         * 关于该编程模型的详细信息请参阅https://bitbot.lmy.name/docs/bitbot-programming
         * @param config_file 配置文件路径，关于配置文件的信息请参阅配置文件章节。//TODO: add hyperlink to config file
         */
        EncosKernel(std::string config_file)
            : KernelTpl<EncosKernel<UserData>, EncosBus, UserData>(config_file)
        {
            this->KernelRegisterEvent("power_on", static_cast<EventId>(EncosKernelEvent::POWER_ON), [this](EventValue, UserData&)
                {
                    this->logger_->info("joints power on");
                    this->busmanager_.PowerOnDevice();
                    this->logger_->info("joints power on finished");
                    return static_cast<StateId>(EncosKernelState::POWER_ON_FINISH); }, false);
            this->KernelRegisterState("power on finish", static_cast<StateId>(EncosKernelState::POWER_ON_FINISH),
                [this](const bitbot::KernelInterface& kernel, ExtraData& extra_data, UserData& user_data) {}, { static_cast<EventId>(KernelEvent::START),static_cast<EventId>(EncosKernelEvent::START_RESET_ZERO) });

            this->InjectEventsToState(static_cast<StateId>(KernelState::IDLE), { static_cast<EventId>(EncosKernelEvent::POWER_ON),static_cast<EventId>(EncosKernelEvent::START_RESET_ZERO) });



            pugi::xml_node EncosKernel_node = this->parser_->GetBitbotNode();
            pugi::xml_node Encos_node = EncosKernel_node.child("Encos");
            std::string NetWorkCard_name_const;
            ConfigParser::ParseAttribute2s(NetWorkCard_name_const, Encos_node.attribute("NetWorkCardName"));

            int bus_freq;
            ConfigParser::ParseAttribute2i(bus_freq, Encos_node.attribute("BusFrequency"));
            this->run_period = 1e6 / bus_freq;

            this->is_init = false;
            for (size_t i = 0; i < 5; i++)
            {
                std::string info = std::string("Attempting to start EtherCAT, try ") + std::to_string(i) + std::string(" of 5.");
                this->logger_->info(info);
                this->is_init = this->busmanager_.InitEtherCAT(NetWorkCard_name_const);
                if (this->is_init)
                    break;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (this->is_init)
            {
                this->kernel_config_data_.stop_flag = false;
                this->busmanager_.ErrorFlag = false;
                this->logger_->info("EtherCAT init successed");
                this->busmanager_.Init();
            }
            else
            {
                /*ethercat初始化失败哩*/
                // 初始化失败了喵，怎么办呢，喵~
                this->busmanager_.ErrorFlag = true;
                this->logger_->error("HaHaHa, your ethercat init fail! please check your network_card's name.");
                this->logger_->error("唔，初始化失败惹喵，应该怎么办呢喵，好着急呀喵，这下实验做不成了吧，小杂鱼~♥");
            }


            std::vector<Encos_CANBusDevice*> dev = this->busmanager_.get_CAN_Devices();
            std::vector<EncosJoint*> joints;
            for (auto& device : dev)
            {
                EncosJoint* joint = dynamic_cast<EncosJoint*>(device);
                if (joint != nullptr)
                {
                    joints.push_back(joint);
                }
            }
            this->joint_auto_zero__ = new JointAutoZero(EncosKernel_node.child("reset"), static_cast<double>(1.0 / static_cast<double>(bus_freq)), this->logger_, joints);
            this->KernelRegisterEvent("start_reset_zero", static_cast<EventId>(EncosKernelEvent::START_RESET_ZERO), [this](EventValue e, UserData& d) {
                bool ok = this->joint_auto_zero__->StartReset(this->kernel_runtime_data_.periods_count);
                if (ok)
                {
                    this->logger_->info("JointAutoZero: start reset zero");
                    return static_cast<StateId>(EncosKernelState::AUTO_ZEROING);
                }
                else
                {
                    this->logger_->error("JointAutoZero: start reset zero failed");
                    return std::optional<StateId>();
                }
                });


            this->KernelRegisterEvent("stop_reset_zero", static_cast<EventId>(EncosKernelEvent::STOP_RESET_ZERO), [this](EventValue e, UserData& d) {
                bool ok = this->joint_auto_zero__->StopReset();
                if (ok)
                {
                    this->logger_->info("JointAutoZero: stop reset zero");
                    return static_cast<StateId>(KernelState::IDLE);
                }
                else
                {
                    this->logger_->error("JointAutoZero: stop reset zero failed");
                    return std::optional<StateId>();
                }
                });


            this->KernelRegisterState("auto zeroing", static_cast<StateId>(EncosKernelState::AUTO_ZEROING),
                [this](const bitbot::KernelInterface& kernel, ExtraData& extra_data, UserData& user_data)
                {
                    this->joint_auto_zero__->StartReset(this->kernel_runtime_data_.periods_count);
                },
                { static_cast<EventId>(EncosKernelEvent::START_RESET_ZERO), static_cast<EventId>(EncosKernelEvent::STOP_RESET_ZERO) });

            this->PrintWelcomeMessage(); // MUST PRIENT WELCOME MESSAGE!!!!!!
        }

        /**
         * @brief 析构函数
         *
         */
        ~EncosKernel()
        {
            delete this->joint_auto_zero__;
            std::cout << "\033[32mGood bye from Bitbot Encos. Make Bitbot Everywhere! \033[0m" << std::endl;
        }

    protected:
        /**
         * @brief 内核开始运行
         * @details 内核开始运行，开发者可以在该函数中实现自己的内核启动逻辑。
         *
         */
        void doStart()
        {
            this->logger_->info("EncosKernel Start");
        }

        /**
         * @brief 内核运行函数
         * @details 内核运行函数，开发者可以在该函数中实现自己的内核运行逻辑。
         *
         */
        void doRun()
        {
            // init bus
            // loop
            if (this->is_init)
                std::cout << "kernel start running!" << std::endl;

            std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point last_time = start_time;
            std::chrono::high_resolution_clock::time_point end_time = start_time;

            constexpr float ns_to_ms = 1 / 1e6;
            constexpr float ms_to_ms = 1 / 1e3;
            constexpr float s_to_ms = 1e3;

            while (!this->kernel_config_data_.stop_flag)
            {
                start_time = std::chrono::high_resolution_clock::now();
                this->kernel_runtime_data_.periods_count++;
                this->kernel_runtime_data_.period = std::chrono::duration_cast<std::chrono::microseconds>(start_time - last_time).count() * ms_to_ms;
                last_time = start_time;

                if (this->busmanager_.ErrorFlag)
                {
                    this->logger_->error("Error occurred, Bitbot Encos kernel is stopping!");
                    this->kernel_config_data_.stop_flag = true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    break;
                }
                this->HandleEvents();
                this->KernelLoopTask();

                this->KernelPrivateLoopEndTask();

                end_time = std::chrono::high_resolution_clock::now();

                auto time_cost = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                this->kernel_runtime_data_.process_time = std::chrono::duration_cast<std::chrono::microseconds>(time_cost).count() * ms_to_ms;

                auto sleep_time = std::chrono::microseconds(this->run_period - 60) - time_cost; //-60为了修正一些误差
                if (sleep_time > std::chrono::microseconds(0)) [[likely]]
                {
                    std::this_thread::sleep_for(sleep_time);
                }
                else
                {
                    this->logger_->warn("program time out!");
                }
            }
        }

    private:
        void PrintWelcomeMessage()
        {
            std::string line0 = "\033[32m================================================================================== \033[0m";
            std::string line1 = "\033[32m|| BBBB   III  TTTTT  BBBB    OOO   TTTTT     EEEEE  N   N   CCCC   OOO    SSSS  ||\033[0m";
            std::string line2 = "\033[32m|| B   B   I     T    B   B  O   O    T       E      NN  N  C      O   O  S      ||\033[0m";
            std::string line3 = "\033[32m|| BBBB    I     T    BBBB   O   O    T       EEEEE  N N N  C      O   O   SSS   ||\033[0m";
            std::string line4 = "\033[32m|| B   B   I     T    B   B  O   O    T       E      N  NN  C      O   O       S ||\033[0m";
            std::string line5 = "\033[32m|| BBBB   III    T    BBBB    OOO     T       EEEEE  N   N   CCCC   OOO    SSSS  ||\033[0m";
            std::string line6 = "\033[32m===================================================================================\033[0m";

            std::cout << std::endl
                << std::endl;
            std::cout << "\033[31mWelcome to use Bitbot Encos. Make Bitbot Everywhere! \033[0m" << std::endl;
            std::cout << line0 << std::endl;
            std::cout << line1 << std::endl;
            std::cout << line2 << std::endl;
            std::cout << line3 << std::endl;
            std::cout << line4 << std::endl;
            std::cout << line5 << std::endl;
            std::cout << line6 << std::endl
                << std::endl;
        }
        // function
    private:
        bool is_init;
        int run_period; // run period in micro second

        JointAutoZero* joint_auto_zero__ = nullptr; // joint auto zero class
    };
};