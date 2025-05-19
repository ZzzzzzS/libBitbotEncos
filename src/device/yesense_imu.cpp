#include "device/yesense_imu.h"
#include "memory"
#include <stdio.h>     /*标准输入输出的定义*/
#include <stdlib.h>    /*标准函数库定义*/
#include <unistd.h>    /*UNIX 标准函数定义*/
#include <sys/types.h> /**/
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <sys/time.h>
#include <string.h>
#include <getopt.h>

namespace bitbot
{
    YesenseIMU::YesenseIMU(const pugi::xml_node& imu_node)
        : Encos_VirtualBusDevice(imu_node) // TODO: check the correctness of the constructor
    {
        this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::IMU);
        this->type_ = static_cast<uint32_t>(EncosDeviceType::Yesense_IMU);
        this->monitor_header_.headers = { "roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z","IMU_temp" };
        this->monitor_data_.resize(monitor_header_.headers.size());
        // TODO: get name and speed from xml
        ConfigParser::ParseAttribute2s(this->dev, imu_node.attribute("dev"));

        fd = open(dev.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
        if (fd < 0)
        {
            printf("Can't Open Serial Port!\n");
            printf("这下搞砸了吧，杂鱼哥哥现在竟然连IMU都找不到了，到底行不行呀，小杂鱼~♥\n");
            exit(0);
        }

        // save to oldtio
        // tcgetattr(fd, &oldtio);
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
        newtio.c_cflag &= ~CSTOPB;
        newtio.c_cflag &= ~PARENB;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSAFLUSH, &newtio);
        // tcgetattr(fd, &oldtio);

        memset(buffer, 0, sizeof(buffer));
    }

    YesenseIMU::~YesenseIMU()
    {
        close(fd);
    }

    float YesenseIMU::GetRoll()
    {
        return this->imu_data_.runtime.roll.load() * d2r;
    }
    float YesenseIMU::GetPitch()
    {
        return this->imu_data_.runtime.pitch.load() * d2r;
    }
    float YesenseIMU::GetYaw()
    {
        return imu_data_.runtime.yaw.load() * d2r;
    }
    float YesenseIMU::GetAccX()
    {
        return this->imu_data_.runtime.a_x;
    }
    float YesenseIMU::GetAccY()
    {
        return this->imu_data_.runtime.a_y;
    }
    float YesenseIMU::GetAccZ()
    {
        return this->imu_data_.runtime.a_z;
    }
    float YesenseIMU::GetGyroX()
    {
        return this->imu_data_.runtime.w_x.load() * d2r;
    }
    float YesenseIMU::GetGyroY()
    {
        return this->imu_data_.runtime.w_y.load() * d2r;
    }
    float YesenseIMU::GetGyroZ()
    {
        return this->imu_data_.runtime.w_z.load() * d2r;
    }
    float YesenseIMU::GetIMUTemperature()
    {
        return this->imu_data_.runtime.IMU_temp.load();
    }


    void YesenseIMU::UpdateRuntimeData()
    {
        // his->monitor_header_.headers = { "roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z" };
        this->monitor_data_[0] = this->imu_data_.runtime.roll.load();
        this->monitor_data_[1] = this->imu_data_.runtime.pitch.load();
        this->monitor_data_[2] = this->imu_data_.runtime.yaw.load();
        this->monitor_data_[3] = this->imu_data_.runtime.a_x.load();
        this->monitor_data_[4] = this->imu_data_.runtime.a_y.load();
        this->monitor_data_[5] = this->imu_data_.runtime.a_z.load();
        this->monitor_data_[6] = this->imu_data_.runtime.w_x.load();
        this->monitor_data_[7] = this->imu_data_.runtime.w_y.load();
        this->monitor_data_[8] = this->imu_data_.runtime.w_z.load();
        this->monitor_data_[9] = this->imu_data_.runtime.IMU_temp.load();
    }

    void YesenseIMU::ReadOnce() // call this function in the bus loop
    {
        nread = read(fd, buffer, RX_BUF_LEN);
        if (nread > 0)
        {
            // printf("nread = %d\n", nread);
            if (g_recv_buf_idx + nread >= RX_BUF_LEN * 2)
            {
                printf("buffer overflow!\n");
                nread = RX_BUF_LEN * 2 - g_recv_buf_idx;
            }
            memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);
            g_recv_buf_idx += nread;
        }

        cnt = g_recv_buf_idx;
        pos = 0;
        if (cnt < YIS_OUTPUT_MIN_BYTES)
        {
            return;
        }

        while (cnt > (unsigned int)0)
        {
            int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info);
            if (analysis_done == ret) /*未查找到帧头*/
            {
                pos++;
                cnt--;
            }
            else if (data_len_err == ret)
            {
                break;
            }
            else if (crc_err == ret || analysis_ok == ret) /*删除已解析完的完整一帧*/
            {
                output_data_header_t* header = (output_data_header_t*)(g_recv_buf + pos);
                unsigned int frame_len = header->len + YIS_OUTPUT_MIN_BYTES;
                cnt -= frame_len;
                pos += frame_len;
                // memcpy(g_recv_buf, g_recv_buf + pos, cnt);

                if (analysis_ok == ret)
                {
                    this->UpdateImuDataFromBus();
                }
            }
        }

        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
        tcflush(fd, TCIFLUSH);
    }

    void YesenseIMU::UpdateImuDataFromBus()
    {
        this->imu_data_.runtime.a_x = g_output_info.accel.x;
        this->imu_data_.runtime.a_y = g_output_info.accel.y;
        this->imu_data_.runtime.a_z = g_output_info.accel.z;
        this->imu_data_.runtime.w_x = g_output_info.angle_rate.x;
        this->imu_data_.runtime.w_y = g_output_info.angle_rate.y;
        this->imu_data_.runtime.w_z = g_output_info.angle_rate.z;
        this->imu_data_.runtime.roll = g_output_info.attitude.roll;
        this->imu_data_.runtime.pitch = g_output_info.attitude.pitch;
        this->imu_data_.runtime.yaw = g_output_info.attitude.yaw;
        this->imu_data_.runtime.IMU_temp = g_output_info.sensor_temp;
    }
};