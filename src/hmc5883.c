
#include <stdint.h>
#include <stdlib.h> // calloc
#include <stdio.h>
#include "gci_sensors/hmc5883.h"

// FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
// -----------------------------------------------
//  MODE CONTROL (MODE)
// 	Standby			0x00
// 	Continuous		0x01

// OUTPUT DATA RATE (ODR)
// 	10Hz        	0x00
// 	50Hz        	0x04
// 	100Hz       	0x08
// 	200Hz       	0x0C

// FULL SCALE (RNG)
// 	2G          	0x00
// 	8G          	0x10

// OVER SAMPLE RATIO (OSR)
// 	512         	0x00
// 	256         	0x40
// 	128         	0x80
// 	64          	0xC0 

// HMC5883L I2C address and register definitions

// #define HMC5883L_CONFIG_A_REG 0x00
// #define HMC5883L_CONFIG_B_REG 0x01
// #define HMC5883L_MODE_REG 0x02
// #define HMC5883L_DATA_X_MSB 0x03
// #define HMC5883L_DATA_X_MSB 0x00
// #define HMC5883L_STATUS_REG 0x09
#define HMC5883L_DATA_X_LSB_REG 0x00
#define HMC5883L_STATUS_REG 0x06
#define HMC5883L_TEMPERATURE_REG 0x07
#define HMC5883L_CTRL1_REG 0x09
#define HMC5883L_CTRL2_REG 0x0A
#define QMC5883L_PERIOD_REG 0x0B
#define HMC5883L_WHO_AM_I_REG 0x0D // 0x10 // ID REG A really, holds ASCII H

#define HMC5883L_ADDR 0x0D // 0x1E
#define HMC5883L_WHO_AM_I 0xFF // 0x68 // ASCII H
#define HMC5883L_SOFT_RESET (1 << 7)

// typedef enum: uint8_t {
//   HMC5883_MODE_NORMAL = 0, // <= default
//   HMC5883_MODE_POS_BIAS = 1,
//   HMC5883_MODE_NEG_BIAS = 2
// } hmc5883_mode_t;

typedef enum: uint8_t {
  HMC5883_MODE_STANDBY = 0b00, // <= default
  HMC5883_MODE_CONTINUOUS = 0b01
} hmc5883_mode_t;

// typedef enum: uint8_t {
//   HMC5883_ODR_15HZ = (4 << 2),
//   HMC5883_ODR_30HZ = (5 << 2),
//   HMC5883_ODR_75HZ = (6 << 2), // <= default
// } hmc5883_odr_t;

typedef enum: uint8_t {
  HMC5883_ODR_10HZ = (1 << 2),
  HMC5883_ODR_50HZ = (2 << 2),
  HMC5883_ODR_100HZ = (3 << 2), // <= default
  HMC5883_ODR_200HZ = (4 << 2), // <= default
} hmc5883_odr_t;

// typedef enum: uint8_t {
//   HMC5883_1_SAMP_AVE = (0 << 5),
//   HMC5883_2_SAMP_AVE = (1 << 5), // <= default
//   HMC5883_4_SAMP_AVE = (2 << 5),
//   HMC5883_8_SAMP_AVE = (3 << 5),
// } hmc5883_sample_t;

typedef enum: uint8_t {
  HMC5883_512_OSR = (0 << 6),
  HMC5883_256_OSR = (1 << 6), // <= default
  HMC5883_128_OSR = (2 << 6),
  HMC5883_64_OSR = (3 << 6),
} hmc5883_sample_t;

// #define HMC5883_BUFFER_SIZE 6


hmc5883_io_t *hmc5883_init(interface_t type, uint8_t port, hmc5883_range_t range) {
  int32_t ok;
  uint8_t cmd;
  hmc5883_io_t *hw = (hmc5883_io_t *)calloc(1, sizeof(hmc5883_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, HMC5883L_ADDR, I2C_INTERFACE);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  hw->calibrated = false;

  // switch (range) {
  //   case HMC5883_0_88:
  //     hw->scale = 0.73f;
  //     break;
  //   case HMC5883_1_3:
  //     hw->scale = 0.92f;
  //     break;
  //   case HMC5883_1_9:
  //     hw->scale = 1.22f;
  //     break;
  //   case HMC5883_2_5:
  //     hw->scale = 1.52f;
  //     break;
  //   case HMC5883_4_0:
  //     hw->scale = 2.27f;
  //     break;
  //   case HMC5883_4_7:
  //     hw->scale = 2.56f;
  //     break;
  //   case HMC5883_5_6:
  //     hw->scale = 3.03f;
  //     break;
  //   case HMC5883_8_1:
  //     hw->scale = 4.35f;
  //     break;
  // }

  switch (range) {
    case HMC5883_2GAUSS:
      hw->scale = 1.0f/12000.0f;
      break;
    case HMC5883_8GAUSS:
      hw->scale = 1.0f; // / 3000.0f;
      break;
    default:
      return NULL;
  }

  ok = comm->read(comm->config, HMC5883L_WHO_AM_I_REG, &cmd, 1);
  if ((cmd != HMC5883L_WHO_AM_I) || (ok < 0)) return NULL;
  printf(">> whoami success! 0x%x\n", cmd);

  // ODR can vary from 0.75 Hz - 75 Hz, default to fastest
  // Config A: 2 (0x01) samples averaged, 75 Hz (0x06) output rate, normal measurement
  // cmd = HMC5883_2_SAMP_AVE | HMC5883_ODR_75HZ | HMC5883_MODE_NORMAL;
  // ok  = comm->write(comm->config, HMC5883L_CONFIG_A_REG, &cmd, 1);
  // if (ok < 0) return NULL;

  cmd = (1 << 7); // HMC5883L_SOFT_RESET;
  ok  = comm->write(comm->config, HMC5883L_CTRL2_REG, &cmd, 1);
  if (ok < 0) return NULL;

  sleep_ms(1000);

  // WTF
  cmd = 0x01; // what is this? See pg 18 datasheet
  ok  = comm->write(comm->config, 0x0B, &cmd, 1);
  if (ok < 0) return NULL;

  // cmd = HMC5883_512_OSR(0x00) | HMC5883_ODR_200HZ(0x0C) | HMC5883_MODE_CONTINUOUS(0x01) | range;
  cmd = 0x1D; // 512_OSR | 200HZ | CONT | 8G
  ok  = comm->write(comm->config, HMC5883L_CTRL1_REG, &cmd, 1);
  if (ok < 0) return NULL;
  printf(">> wrote ok: %d  cmd: 0x%x\n", ok, cmd);

  ok  = comm->read(comm->config, HMC5883L_CTRL1_REG, &cmd, 1);
  printf(">> read ok: %d  reg 0x%x: 0x%x\n", ok, HMC5883L_CTRL1_REG, cmd);

  // Config B: Set gain to 1.3 Ga (1090 LSB/Gauss)
  // cmd = (1 << 6); // auto incr?
  cmd = 0x00;
  ok  = comm->write(comm->config, HMC5883L_CTRL2_REG, &cmd, 1);
  if (ok < 0) return NULL;

  sleep_ms(1);

  return hw;
}

hmc5883_io_t *hmc5883_i2c_init(uint8_t port, hmc5883_range_t range) {
  return hmc5883_init(I2C_INTERFACE, port, range);
}

hmc5883_io_t *hmc5883_spi_init(uint8_t port, hmc5883_range_t range) {
  return hmc5883_init(SPI_INTERFACE, port, range);
}

hmc5883_t hmc5883_read(hmc5883_io_t *hw) {
  hmc5883_t ret;
  hw->ok                 = false;
  comm_interface_t *comm = hw->comm;
  // uint8_t *buf           = hw->buf;
  uint8_t buf[9]; // [XL,XM,YL,YM,ZL,ZM,STATUS,TL,TM]
  int32_t ok;
  // uint8_t data[2];

  // ok = comm->read(comm->config, HMC5883L_STATUS_REG, data, 1);
  // printf(">> ok: %d   STATUS_REG: %u\n", ok, (uint32_t)data[0]);

  // ok = comm->read(comm->config, HMC5883L_DATA_X_LSB_REG, buf, HMC5883_BUFFER_SIZE);
  ok = comm->read(comm->config, HMC5883L_DATA_X_LSB_REG, buf, 9);
  // for (int i=0; i<6; ++i) ok = comm->read(comm->config, HMC5883L_DATA_X_LSB_REG + i, &buf[i], 1);
  // printf(">> ok: %d\n", ok);
  if (ok < 0) return ret;

  // Combine MSB and LSB
  ret.x = hw->scale * (float)(((int16_t)buf[1] << 8) | buf[0]);
  ret.y = hw->scale * (float)(((int16_t)buf[3] << 8) | buf[2]);
  ret.z = hw->scale * (float)(((int16_t)buf[5] << 8) | buf[4]);

  printf(">> ok: %d   STATUS_REG: %u  overflow: %u\n", ok, (uint32_t)buf[6], buf[6] >> 1);

  ret.temperature = (float)(((int16_t)buf[8] << 8) | buf[7]) / 100.0f;

  hw->ok = true;

  return ret;
}