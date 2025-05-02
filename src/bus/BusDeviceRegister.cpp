#include "bitbot_kernel/device/device_factory.hpp"
#include "device/yesense_imu.h"
#include "device/Encos_joint.h"

namespace bitbot
{
    void EncosRegisterDeviceWarper()
    {
        static DeviceRegistrar<EncosDevice, EncosJoint> JointReg(static_cast<uint32_t>(EncosDeviceType::Encos_JOINT), "EncosJoint");
        static DeviceRegistrar<EncosDevice, YesenseIMU> ImuReg(static_cast<uint32_t>(EncosDeviceType::Yesense_IMU), "YesenseIMU");
    }
} // namespace bitbot
