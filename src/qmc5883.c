
#include "gci_sensors/qmc5883.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // calloc
#include <string.h> // memcpy

// FROM QST QMC5883L Datasheet
// -----------------------------------------------
//  MODE CONTROL (MODE)
// 	Standby			0x00
// 	Continuous		0x01

// OUTPUT DATA ODR (ODR)
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

#define QMC5883L_DATA_X_LSB_REG  0x00
#define QMC5883L_STATUS_REG      0x06
#define QMC5883L_TEMPERATURE_REG 0x07
#define QMC5883L_CTRL1_REG       0x09
#define QMC5883L_CTRL2_REG       0x0A
#define QMC5883L_PERIOD_REG      0x0B
#define QMC5883L_WHO_AM_I_REG    0x0D // 0x10 // ID REG A really, holds ASCII H

#define QMC5883L_ADDR       0x0D // 0x1E
#define QMC5883L_WHO_AM_I   0xFF // 0x68 // ASCII H
#define QMC5883L_SOFT_RESET (1 << 7)

typedef enum : uint8_t {
  QMC5883_MODE_STANDBY    = 0b00, // <= default
  QMC5883_MODE_CONTINUOUS = 0b01
} qmc5883_mode_t;

typedef enum : uint8_t {
  QMC5883_ODR_10HZ  = (0 << 2),
  QMC5883_ODR_50HZ  = (1 << 2),
  QMC5883_ODR_100HZ = (2 << 2), // <= default
  QMC5883_ODR_200HZ = (3 << 2), // <= default
} qmc5883_odr_t;

typedef enum : uint8_t {
  QMC5883_512_OSR = (0 << 6),
  QMC5883_256_OSR = (1 << 6), // <= default
  QMC5883_128_OSR = (2 << 6),
  QMC5883_64_OSR  = (3 << 6),
} qmc5883_sample_t;

qmc5883_io_t *qmc5883_create(uint8_t port) {
  qmc5883_io_t *hw = (qmc5883_io_t *)calloc(1, sizeof(qmc5883_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, QMC5883L_ADDR, I2C_INTERFACE);
  if (comm == NULL) {
    free(hw);
    return NULL;
  }

  hw->comm = comm;
  
  float m[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  memcpy(hw->mcal, m, 12 * sizeof(float));

  return hw;
}

int qmc5883_init(qmc5883_io_t *hw, qmc5883_range_t range) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  comm_interface_t *comm = hw->comm;
  uint8_t cmd;

  hw->calibrated = false;

  switch (range) {
  case QMC5883_2GAUSS:
    hw->scale = 1.0f / 12000.0f;
    break;
  case QMC5883_8GAUSS:
    hw->scale = 1.0f / 3000.0f;
    break;
  default:
    return GCIS_ERROR_PARAM;
  }

  if (comm->read(comm->config, QMC5883L_WHO_AM_I_REG, &cmd, 1) < 0) return -1;
  if (cmd != QMC5883L_WHO_AM_I) return GCIS_ERROR_WHOAMI;
  // printf(">> whoami success! 0x%x\n", cmd);

  // ODR can vary from 0.75 Hz - 75 Hz, default to fastest
  // Config A: 2 (0x01) samples averaged, 75 Hz (0x06) output rate, normal measurement
  // cmd = QMC5883_2_SAMP_AVE | QMC5883_ODR_75HZ | QMC5883_MODE_NORMAL;
  // ok  = comm->write(comm->config, QMC5883L_CONFIG_A_REG, &cmd, 1);
  // if (ok < 0) return NULL;

  cmd = QMC5883L_SOFT_RESET;
  if (comm->write(comm->config, QMC5883L_CTRL2_REG, &cmd, 1) < 0) return -1;

  sleep_ms(10);

  int8_t cfg[] = {
      0x1D, // 09h: 512_OSR | 200HZ | CONT | 8G
      0x00, // 0Ah
      0x01, // 0Bh: WTF? See pg 18 datasheet
  };
  if (comm->write(comm->config, 0x09, cfg, sizeof(cfg)) < 0) return -1;

  // // WTF
  // // cmd = 0x01; // what is this? See pg 18 datasheet
  // if (comm->write(comm->config, 0x0B, &cmd, 1) < 0) return -1;

  // ok = comm->read(comm->config, 0x0B, &cmd, 1);
  // printf(">> read ok: %d  reg %s: 0x%x\n", ok, "0x0B", cmd);

  // // cmd = QMC5883_512_OSR(0x00) | QMC5883_ODR_200HZ(0x0C) | QMC5883_MODE_CONTINUOUS(0x01) | range;
  // // cmd = QMC5883_512_OSR | QMC5883_ODR_10HZ | QMC5883_MODE_CONTINUOUS | QMC5883_8GAUSS;
  // cmd = 0x1D; // 512_OSR | 200HZ | CONT | 8G
  // if (comm->write(comm->config, QMC5883L_CTRL1_REG, &cmd, 1) < 0) return -1;
  // // printf(">> wrote ok: %d  cmd: 0x%x\n", ok, cmd);

  // // ok = comm->read(comm->config, QMC5883L_CTRL1_REG, &cmd, 1);
  // // printf(">> read ok: %d  reg 0x%x: 0x%x\n", ok, QMC5883L_CTRL1_REG, cmd);

  // // Config B: Set gain to 1.3 Ga (1090 LSB/Gauss)
  // // cmd = (1 << 6); // auto incr? - no one sets this
  // cmd = 0x00;
  // ok  = comm->write(comm->config, QMC5883L_CTRL2_REG, &cmd, 1);
  // if (ok < 0) return NULL;

  // // comm->read(comm->config, 0x04, &cmd, 1);
  // comm->read(comm->config, 0x05, &cmd, 1); // disable lockout?

  // sleep_ms(1);

  return 0;
}

// qmc5883_io_t *qmc5883_i2c_init(uint8_t port, qmc5883_range_t range) {
//   return qmc5883_init(I2C_INTERFACE, port, range);
// }

// qmc5883_io_t *qmc5883_spi_init(uint8_t port, qmc5883_range_t range) {
//   return qmc5883_init(SPI_INTERFACE, port, range);
// }

int qmc5883_read(qmc5883_io_t *hw, vec3f_t *ret) {
  comm_interface_t *comm = hw->comm;
  uint8_t buf[9]; // [XL,XH,YL,YH,ZL,ZH,STATUS,TL,TH]
  // int32_t ok;
  uint8_t status = 0;

  ret->x = 0.0f;
  ret->y = 0.0f;
  ret->z = 0.0f;

  if (comm->read(comm->config, 0x05, &status, 1) < 0) return -1;
  printf(">> start ZH: %u\n", status);

  if (comm->read(comm->config, QMC5883L_STATUS_REG, &status, 1) < 0) return -1;
  printf(">> STATUS_REG: %u\n", (uint32_t)status);

  if (comm->read(comm->config, 0x05, &status, 1) < 0) return -1;
  printf(">> start ZH: %u\n", status);

  // ok = comm->read(comm->config, QMC5883L_DATA_X_LSB_REG, buf, QMC5883_BUFFER_SIZE);
  if (comm->read(comm->config, QMC5883L_DATA_X_LSB_REG, buf, 7) < 0) return -1;
  // for (int i=0; i<6; ++i) ok = comm->read(comm->config, QMC5883L_DATA_X_LSB_REG + i, &buf[i], 1);
  // printf(">> ok: %d\n", ok);
  // if (ok < 0) return ret;

  // Combine MSB and LSB
  ret->x = hw->scale * (float)(((int16_t)buf[1] << 8) | buf[0]);
  ret->y = hw->scale * (float)(((int16_t)buf[3] << 8) | buf[2]);
  ret->z = hw->scale * (float)(((int16_t)buf[5] << 8) | buf[4]);

  printf(">> STATUS_REG: %u  overflow: %u\n", (uint32_t)buf[6], buf[6] >> 1);

  // ret.temperature = (float)(((int16_t)buf[8] << 8) | buf[7]) / 100.0f;

  // hw->ok = true;

  return 0;
}

// vec3f_t qmc5883_read(qmc5883_io_t *hw) {
//   vec3f_t ret;
//   hw->ok                 = false;
//   comm_interface_t *comm = hw->comm;
//   // uint8_t *buf           = hw->buf;
//   uint8_t buf[9]; // [XL,XH,YL,YH,ZL,ZH,STATUS,TL,TH]
//   int32_t ok;
//   // uint8_t data[2];
//   uint8_t status = 0;

//   comm->read(comm->config, 0x05, &status, 1);
//   printf(">> start ZH: %u\n", status);

//   ok = comm->read(comm->config, QMC5883L_STATUS_REG, &status, 1);
//   printf(">> start ok: %d   STATUS_REG: %u\n", ok, (uint32_t)status);

//   comm->read(comm->config, 0x05, &status, 1);
//   printf(">> start ZH: %u\n", status);

//   // ok = comm->read(comm->config, QMC5883L_DATA_X_LSB_REG, buf, QMC5883_BUFFER_SIZE);
//   ok = comm->read(comm->config, QMC5883L_DATA_X_LSB_REG, buf, 7);
//   // for (int i=0; i<6; ++i) ok = comm->read(comm->config, QMC5883L_DATA_X_LSB_REG + i, &buf[i], 1);
//   // printf(">> ok: %d\n", ok);
//   if (ok < 0) return ret;

//   // Combine MSB and LSB
//   ret.x = hw->scale * (float)(((int16_t)buf[1] << 8) | buf[0]);
//   ret.y = hw->scale * (float)(((int16_t)buf[3] << 8) | buf[2]);
//   ret.z = hw->scale * (float)(((int16_t)buf[5] << 8) | buf[4]);

//   printf(">> ok: %d   STATUS_REG: %u  overflow: %u\n", ok, (uint32_t)buf[6], buf[6] >> 1);

//   // ret.temperature = (float)(((int16_t)buf[8] << 8) | buf[7]) / 100.0f;

//   hw->ok = true;

//   return ret;
// }

// QMC5883L I2C address and register definitions

// #define QMC5883L_CONFIG_A_REG 0x00
// #define QMC5883L_CONFIG_B_REG 0x01
// #define QMC5883L_MODE_REG 0x02
// #define QMC5883L_DATA_X_MSB 0x03
// #define QMC5883L_DATA_X_MSB 0x00
// #define QMC5883L_STATUS_REG 0x09

// typedef enum: uint8_t {
//   QMC5883_MODE_NORMAL = 0, // <= default
//   QMC5883_MODE_POS_BIAS = 1,
//   QMC5883_MODE_NEG_BIAS = 2
// } qmc5883_mode_t;

// typedef enum: uint8_t {
//   QMC5883_ODR_15HZ = (4 << 2),
//   QMC5883_ODR_30HZ = (5 << 2),
//   QMC5883_ODR_75HZ = (6 << 2), // <= default
// } qmc5883_odr_t;

// typedef enum: uint8_t {
//   QMC5883_1_SAMP_AVE = (0 << 5),
//   QMC5883_2_SAMP_AVE = (1 << 5), // <= default
//   QMC5883_4_SAMP_AVE = (2 << 5),
//   QMC5883_8_SAMP_AVE = (3 << 5),
// } qmc5883_sample_t;