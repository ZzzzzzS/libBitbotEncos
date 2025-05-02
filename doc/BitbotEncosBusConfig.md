# Bitbot Encos 驱动板说明
>
> 该文档节选自Encos电机官方文档，详细信息请参阅Encos官方文档。

# 接线说明及线序

![](https://opensource-doc-1253829354.cos.ap-beijing.myqcloud.com/libBitbotEncos/EncosBoard.png)

以上图为例，左侧为电源供电口，右侧为网线链接口，上下为CAN信号传输口，均使用1.25×4Pin 接口线进行连接。底部通过4Pin线进行串联，另一侧通过网线连接到主机口中（当串联时，ethercat板子需要注意网口输入以及输出）。

经过测试可串联20块板子保证使用，通过主站下发速度可达1Khz,内部CAN数据通过间隔50us依次下发，一块转接板可最多一次控制六个电机。转接板内设有6个通道，每个通道所对应一个电机，其中1、2、3通道对应转接板右侧的CAN1；4、5、6通道对应转接板左侧CAN2（以上图转接板放置方向为标准）所需代码、上位机资料链接：<https://pan.baidu.com/s/1l5z25D0bQwoyJS3KzZ1EAg?pwd=6smn>

[Bitbot Encos配置文件](./BitbotEncosConfig.md)中的``slave_id``指的是该转换板的从站ID，根据接线顺序从0递增。``id``指的是电机本身的CAN总线ID，需保证与电机设置的相同。

![EtherCAT-CAN串联示例图](https://opensource-doc-1253829354.cos.ap-beijing.myqcloud.com/libBitbotEncos/EncosBoardConn.png)

**注意：BitbotEncos优先使用每个EtherCAT转CAN从站的CAN1通道，即转换板右侧的1、2、3接口，因此在接入电机时需要保证CAN1通道插满后再接CAN2通道。** EtherCAT-CAN从站右侧接口4个接口以及左侧4个接口在电路板上是分别并联互通的，实际仅存在两个CAN通道(而不是6个)，因此仅需保证右侧接入3个电机后再接左侧，而不是将所有接口插满。
