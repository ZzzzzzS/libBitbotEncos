@mainpage

# BitbotEncos

Bitbot Encos是Bitbot机器人控制框架的一个实例。该实例提供Encos电机相关的驱动程序使用户可以快速在安装了Encos电机和EtherCAT转CAN卡的机器人实物上部署自己的控制算法。

> 该Bitbot实例基于Bitbot机器人软件框架设计，关于Bitbot框架的更多详细信息请参阅其[官方网站](https://bitbot.lmy.name/)。

# 硬件配置

BitbotEncos支持的硬件配置包括[Encos电机](http://encos.cn/)，[Yesense IMU](https://www.yesense.com/yis320)，和带有CAN总线的智能电池。BitbotEncos使用Encos推出的EtherCAT转CAN板卡来管理CAN总线设备。关于该板卡的详细信息和电机接线说明请参阅[Bitbot Encos 驱动板说明](./doc/BitbotEncosBusConfig.md)章节。

BitbotEncos的硬件连接关系如下图所示，其中slave0, slave1指的是Bitbot Encos驱动板，PC选用的是配备有以太网功能的计算机，且安装有具备实时内核的linux操作系统。

![Bitbot Encos电气连接关系](./doc/Hardware.svg)

# 软件配置

Bitbot Encos需要在具备实时内核的linux操作系统上运行，已在Ubuntu24.04上测试。Bitbot Encos无需任何依赖，仅需在[发布页面](https://github.com/ZzzzzzS/libBitbotEncos/releases)下载即可开始使用。

注意，soem需要操作rawsocket，因此程序需要root权限或者赋予capability才能运行。可以在生成最终可执行程序的``CMakeList.txt``的``add_executable``函数后添加
```cmake
add_custom_command(TARGET main_app POST_BUILD COMMAND sudo setcap cap_net_admin,cap_net_raw=eip $<TARGET_FILE:main_app> )
```
来自动为可执行文件添加capability，实现无需root权限运行。该命令中的``main_app``为最终可执行项目的名称，请按需替换成用户定义的名称。**注意，该命令使用了sudo命令，请确保当前用户组属于sudo组，且sudo指令无需输入密码！** 否则编译过程将会卡死。用户也可在构建完成后手动执行``sudo setcap cap_net_admin,cap_net_raw=eip ./main_app``或直接使用root用户运行程序来解决rawsocket的读写问题。

关于soem无root读写rawsocket详情，请参阅[soem-issue83](https://github.com/OpenEtherCATsociety/SOEM/issues/83)。关于linux读写rawsocket的问题请参阅[Using Linux Raw Sockets](https://squidarth.com/networking/systems/rc/2018/05/28/using-raw-sockets.html)。关于如何使sudo命令免密码请参阅[这里](https://cn.linux-terminal.com/?p=2065)。

# 配置说明

和[其它Bitbot实例](https://github.com/limymy/bitbot-mujoco-demo)一样，BitbotEncos需要根据硬件电气连接情况设置配置文件才能正确运行。关于配置文件的详细信息请参阅[Bitbot Encos的配置文件](./doc/BitbotEncosConfig.md)章节，或访问<https://bitbot.lmy.name/docs/bitbot-config_file>获取配置文件的详细信息。

# 电机控制说明

Encos电机支持位置控制，速度控制，力矩/电流控制，和运动模式4种控制模式。关于电机控制的详细说明请参阅[Encos电机模式简介](./doc/BitbotEncosMotorMotion.md)。关于的电机相关API请参阅@ref bitbot::EncosJoint "EncosJoint"。

# API

* Bitbot内核接口: <https://bitbot.lmy.name/docs/bitbot-programming>
* 电机控制：@ref bitbot::EncosJoint "EncosJoint"
* IMU控制：@ref bitbot::YesenseIMU "YesenseIMU"
* 电池控制：TBD...
