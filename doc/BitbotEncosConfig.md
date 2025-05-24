# Bitbot Encos的配置文件

Bitbot通过配置文件对系统参数及硬件设备进行配置与管理。配置文件使用 xml 格式。关于Bitbot配置文件的整体信息请参阅 <https://bitbot.lmy.name/docs/bitbot-config_file>.
这里仅介绍``bus``和``Encos``节点下的相关信息。

# Bitbot Encos配置文件示例

``` xml
<?xml version="1.0" encoding="utf-8" standalone="no"?>
<bitbot>
    <logger path="./log" level="info" />
    <backend port="12888" settings_file="setting.json" />
    <Encos NetWorkCardName="EtherCAT0" BusFrequency="1000" />
    <bus>
        <device id="2" type="EncosJoint" name="L2" mode="motion" enable="1"
            motor_direction="-1"  kp="0" kd="3" slave_id="1"
            kp_range="500" kd_range="5" vel_range="18" pos_range="12.5"
            torque_range="150" current_range="70"
        />
        <device id="8" type="YesenseIMU" name="imu" dev="/dev/ttyIMU" />
    </bus>
    <zero>
        <zero_group reset_period="5">
            <resetter name="L1" joint_lower_limit="-1.57" joint_upper_limit="1.57"
            joint_reset_torque="5" joint_reset_velocity="3.14"/>
            <resetter name="R1" joint_lower_limit="-1.57" joint_upper_limit="1.57"
            joint_reset_torque="5" joint_reset_velocity="3.14"/>
        </zero_group>
    </zero>
</bitbot>

```

在这个示例配置中，我们设置了由一个IMU和一个Encos电机组成的硬件系统。其中Encos节点下指定了``EtherCAT``网卡名称为``EtherCAT0``，总线频率为1000Hz。bus节点下指定了imu设备的读写路径为``/dev/ttyIMU``；指定了电机的``CAN ID``为2，``slave_id``为1。

# Bitbot Encos的配置文件详细说明

## Encos节点

* **EtherCAT：** 指定EtherCAT网卡名称，该名称可通过``ifconfig``查看。
* **BusFrequency：** 指定EtherCAT总线读写频率，注意出于硬件限制，该频率最大为1000Hz

## bus/device节点

### IMU type

* **dev：** 指定IMU设备读写文件路径。

### Encos type

* **id：** 指定了Encos电机CAN总线ID，需要与电机实际设置的CAN总线ID相符。

* **mode：** 设定了电机的运动模式，可选项包括``position``,``velocity``,``torque``和``motion``。关于运动模式的详细说明请参阅[电机运动模式](./BitbotEncosMotorMotion.md)章节。

* **enable：** 设置电机是否使能，电机只有在设置为使能状态时才能上电，上电后才能运动。电机在未使能未上电时仍能获取当前状态和设置零点。

* **motor_direction：** 设置电机旋转方向，1为正方向，-1为负方向。

* **slave_id：** 设置电机EtherCAT从机ID，该ID需要与电机实际连接的从站ID相符，从站ID从0开始递增。

* **kp：** 设置电机运动模式下位置环比例系数。关于运动模式的详细说明请参阅[电机运动模式](./BitbotEncosMotorMotion.md)章节。

* **kd：** 设置电机运动模式下位置环微分系数。关于运动模式的详细说明请参阅[电机运动模式](./BitbotEncosMotorMotion.md)章节。

* **kp_range：** 设置电机运动模式位置环比例系数范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

* **kd_range：** 设置电机运动模式位置环微分系数范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

* **vel_range：** 设置电机速度范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

* **pos_range：** 设置电机位置范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

* **torque_range：** 设置电机力矩范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

* **current_range：** 设置电机电流范围，注意该参数需要与电机实际允许的范围一致，否则将导致错误或损坏电机！

## zero节点(仅在具备自动标零功能的关节上可用)

* **zero_group：** 分组设置零点的分组顺序。

* **reset_period：** 分组自动设置零点的工作时间。（设置过短可能导致关节未运动到限位，导致标定错误）。

* **resetter：** 标定关节。

* **name：** 关节名称。

* **joint_lower_limit：** 关节限位下限（rad）。

* **joint_upper_limit：** 关节限位上限（rad）。

* **joint_reset_torque：** 关节复位力矩（N/m）。

* **joint_reset_velocity：** 关节复位速度（rad/s）。
