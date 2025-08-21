////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gci_sensors/io.h"
#include "gci_sensors/typedefs.h"

#if defined __cplusplus
extern "C" {
#endif

// #define ICM42688P_BUFFER_SIZE 14

typedef enum {
  // ICM42688_ODR_32KHZ = 0x01,
  // ICM42688_ODR_16KHZ = 0x02,
  ICM42688_ODR_8KHZ  = 0x03,
  ICM42688_ODR_4KHZ  = 0x04,
  ICM42688_ODR_2KHZ  = 0x05,
  ICM42688_ODR_1KHZ  = 0x06,
  ICM42688_ODR_200HZ = 0x07,
  ICM42688_ODR_100HZ = 0x08,
  ICM42688_ODR_50HZ  = 0x09,
  ICM42688_ODR_500HZ = 0xff,
} icm42688p_odr_t;

typedef enum {
  ICM42688_ACCEL_16G = 0x00,
  ICM42688_ACCEL_8G  = 0x20,
  ICM42688_ACCEL_4G  = 0x40,
  ICM42688_ACCEL_2G  = 0x60,
} icm42688p_accel_t;

typedef enum {
  ICM42688_GYRO_2000DPS = 0x00,
  ICM42688_GYRO_1000DPS = 0x20,
  ICM42688_GYRO_500DPS  = 0x40,
  ICM42688_GYRO_250DPS  = 0x60,
} icm42688p_gyro_t;

typedef enum {
  ICM42688_NF_1449HZ=0,
  ICM42688_NF_680HZ=0,
  ICM42688_NF_329HZ=0,
  ICM42688_NF_162HZ=0,
  ICM42688_NF_80HZ=0,
  ICM42688_NF_40HZ=0,
  ICM42688_NF_20HZ=0,
  ICM42688_NF_10HZ=0,
} icm42688p_nf_t;

typedef enum {
  ICM42688_UIF_ORDER_1=0,
  ICM42688_UIF_ORDER_2=1,
  ICM42688_UIF_ORDER_3=2,
} icm42688p_uif_t;

typedef struct {
  comm_interface_t *comm;
  // uint8_t buffer[ICM42688P_BUFFER_SIZE];
  float acal[12];
  float gcal[12];
  float ascale;
  float gscale;

  // change these from defaults before calling init()
  icm42688p_odr_t odr;
  icm42688p_accel_t accel;
  icm42688p_gyro_t gyro;

} icm42688p_io_t;

icm42688p_io_t *icm42688p_create(interface_t type, uint8_t port,
                                 uint8_t addr_cs);
int icm42688p_init(icm42688p_io_t *hw);
int icm42688p_read(icm42688p_io_t *hw, imuf_t *imu);
// int icm42688p_calibration(icm42688p_io_t *hw, imuf_t *imu);
// int icm42688p_dump(icm42688p_io_t *hw);
// int icm42688p_calibrate(icm42688p_io_t *hw, uint16_t num_pts);

#if defined __cplusplus
}
#endif