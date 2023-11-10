// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

// include IMU base interface
#include "imu_interface.h"

// include sensor API headers
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MPU6050.h"

class GY87IMU : public IMUInterface
{
private:
    const float accel_scale_ = 1 / 16384.0;
    const float gyro_scale_ = 1 / 131.0;

    MPU6050 accelerometer_;
    MPU6050 gyroscope_;

    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;

public:
    GY87IMU()
    {
    }

    bool startSensor() override
    {
        Wire.begin();
        bool ret;
        accelerometer_.initialize();
        ret = accelerometer_.testConnection();
        if (!ret)
            return false;

        gyroscope_.initialize();
        ret = gyroscope_.testConnection();
        if (!ret)
            return false;

        return true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        int16_t ax, ay, az;

        accelerometer_.getAcceleration(&ax, &ay, &az);

        accel_.x = ax * (double)accel_scale_ * g_to_accel_;
        accel_.y = ay * (double)accel_scale_ * g_to_accel_;
        accel_.z = az * (double)accel_scale_ * g_to_accel_;

        return accel_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        int16_t gx, gy, gz;

        gyroscope_.getRotation(&gx, &gy, &gz);

        gyro_.x = gx * (double)gyro_scale_ * DEG_TO_RAD;
        gyro_.y = gy * (double)gyro_scale_ * DEG_TO_RAD;
        gyro_.z = gz * (double)gyro_scale_ * DEG_TO_RAD;

        return gyro_;
    }
};
#endif