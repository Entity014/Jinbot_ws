#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// include the header of your new driver here similar to default_imu.h
#include "default_imu.h"

// now you can create a config constant that you can use in lino_base_config.h
#ifdef USE_GY87_IMU
// pass your built in class to IMU macro
#define IMU GY87IMU
#endif

#endif