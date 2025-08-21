
#include "gci_sensors/lsm6dsox.h"
#include <stdio.h>  // printf
#include <stdlib.h> // calloc
#include <string.h> // memcpy

#define WHO_AM_I      0x6C // 01101100 108
#define TEMP_SCALE    (1.0f / 256.0f)
#define INT_DRDY_XL   0x01 // accel data ready INT pin
#define INT_DRDY_G    0x02 // gyro data ready INT pin
#define INT_DRDY_TEMP 0x04 // temperature data ready INT pin

#define REG_FIFO_CTRL1 0x07
#define REG_WHO_AM_I   0x0F // 01101100
#define REG_CTRL1_XL   0x10 // Accel settings
#define REG_CTRL3_C    0x12 // interrupt stuff, reboot
#define REG_STATUS     0x1E
#define REG_OUT_TEMP_L 0x20 // termperature

////////////////////////////////////////////////////////////////////////////////

lsm6dsox_io_t *lsm6dsox_create(interface_t type, uint8_t port, uint8_t addr_cs) {
  lsm6dsox_io_t *hw = (lsm6dsox_io_t *)calloc(1, sizeof(lsm6dsox_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
  if (comm == NULL) {
    free(hw);
    return NULL;
  }
  hw->comm = comm;

  float sm[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  memcpy(hw->acal, sm, 12 * sizeof(float));
  memcpy(hw->gcal, sm, 12 * sizeof(float));

  return hw;
}

int lsm6dsox_init(lsm6dsox_io_t *hw, lsm6dsox_xl_range_t accel_range, lsm6dsox_g_range_t gyro_range, lsm6dsox_odr_t odr) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  int err                = 0;
  uint8_t data           = 0;
  comm_interface_t *comm = hw->comm;

  // printf("> check whoami\n");
  if (comm->read(comm->config, REG_WHO_AM_I, &data, 1) < 0) return -1;
  // printf("> whoami: %u = %u    num read: %d bytes\n", data, WHO_AM_I, err);
  if (data != WHO_AM_I) return GCIS_ERROR_WHOAMI;

  // reset everything to default
  data = 0x01; // CTRL3_C SW_RESET ... sets it to default values
  if (comm->write(comm->config, 0x12, &data, 1) < 0) return -1;
  sleep_ms(30);
  // printf("> reset done\n");

  if (accel_range == LSM6DSOX_XL_2_G) hw->a_scale = 2.0f / 32768.0f;
  else if (accel_range == LSM6DSOX_XL_4_G) hw->a_scale = 4.0f / 32768.0f;
  else if (accel_range == LSM6DSOX_XL_8_G) hw->a_scale = 8.0f / 32768.0f;
  else if (accel_range == LSM6DSOX_XL_16_G) hw->a_scale = 16.0f / 32768.0f;
  else return GCIS_ERROR_PARAM;

  if (gyro_range == LSM6DSOX_G_125_DPS) hw->g_scale = 125.0f / 32768.0f;
  else if (gyro_range == LSM6DSOX_G_250_DPS) hw->g_scale = 250.0f / 32768.0f;
  else if (gyro_range == LSM6DSOX_G_500_DPS) hw->g_scale = 500.0f / 32768.0f;
  else if (gyro_range == LSM6DSOX_G_1000_DPS) hw->g_scale = 1000.0f / 32768.0f;
  else if (gyro_range == LSM6DSOX_G_2000_DPS) hw->g_scale = 2000.0f / 32768.0f;
  else return GCIS_ERROR_PARAM;

  // Assume Mode 1: sensor -> I2C/SPI -> uC
  //
  // REG        | DEFAULT
  // -----------|---------
  // FUNC_CFG   | 0x00
  // PIN_CTRL   | 0x3F
  // -----------|---------
  // FIFO_CTRL1 | 0x00
  // FIFO_CTRL2 | 0x00
  // FIFO_CTRL3 | 0x00
  // FIFO_CTRL4 | 0x00
  // BDR_REG1   | 0x00
  // BDR_REG2   | 0x00
  // INT1_CTRL  | 0x00
  // INT2_CTRL  | 0x00
  // -----------|---------
  // WHO_AM_I   | 0x6C
  // -----------|---------
  // CTRL1_XL   | 0x00
  // CTRL2_G    | 0x00
  // CTRL3_C    | 0x04
  // CTRL4_C    | 0x00
  // CTRL5_C    | 0x00
  // CTRL6_C    | 0x00
  // CTRL7_G    | 0x00
  // CTRL8_XL   | 0x00
  // CTRL9_XL   | 0xD0
  // CTRL10_C   | 0x00

#define INT1_DRDY_G  0x02 // 0x02: G ready
#define INT2_DRDY_XL 0x01 // 0x01: XL ready, 0x00: disable INT2
  // reg 07-0E
  uint8_t blk0[8] = {
      0x00,        // FIFO 1 (default)
      0x00,        // FIFO 2 (default)
      0x00,        // FIFO 3 (default)
      0x00,        // FIFO 4 (default)
      0x00,        // counter bdr reg 1 (default)
      0x00,        // counter bdr reg 2 (default)
      INT1_DRDY_G, // INT1 - DRDY Gyro
      INT2_DRDY_XL // INT2 - DRDY Accel
  };

  // GYRO Filtering (Fig 19)
  // You can bypass HPF and LPF1, but not LPF2
  // You cannot set LPF2, see table 18
  //
  // ODR [Hz] Cutoff [Hz]
  // 104      33
  // 208      66.8
  // 416      135.9
  // 833      295.5
  // 1667    1108.1
  //
  //              |HP_EN_G|            |LPF1_SEL_G|
  // GYRO ADC -+->|  0    | --------+->|    0     | ---------+-> LPF2 -> SPI/I2C
  //           |  |       |         |  |          |          |
  //           +->|  1    | -> HPF -+  |    1     | -> LPF1 -+
  //
  //
  // ACCEL Filtering (Fig 16 & 17)
  // The ACCEL has an analog anti-alias LPF (AALPF) right before the ADC
  // and then goes into a digital LPF1 controled by ODR_XL and LPF2 is
  // controlled by HPCF_XL
  //
  //                          |LPF2_XL_EN|
  // AALPF -> ADC -> LPF1 -+->|    0     | ---------+-> SPI/I2C
  //                       |  |          |          |
  //                       +->|    1     | -> LPF2 -+

#define HP_EN_G    0x00 // G disable HPF
#define LPF1_SEL_G 0x00 // G turn off LPF1, only use LPF2
#define LPF2_XL_EN 0x00 // turn off LPF2 for XL
#define IF_INC     0x04 // 0x04: auto incr pointer on read reg
#define G_BW       0x03 // 0x03: FTYPE, LPF1 highest BW - Don't care, disabled
#define BDU        0x40 // 0x00: continuous, 0x40: not updated until read
#define PP_OD      0x00 // 0x00: push-pull mode ... this is what I want
#define I3C_DIS    0x02 // 0x02: disable I3C
#define DEN        0xD0 // 0xD0: DEN ... don't use, but is default
#define TS_DIS     0x00 // timestamp EN: 0x20, DIS: 0x00

  const uint8_t I2C_EN = (comm->type == I2C_INTERFACE) ? 0 : 4; // 0: enable, 4: disable

  // reg 10-19
  uint8_t blk1[10] = {
      odr | accel_range | LPF2_XL_EN, // ctrl 1 xl - ODR | XL_RANGE | LPF2_XL_EN(0)
      odr | gyro_range,               // ctrl 2 g - ODR | G_RANGE
      BDU | IF_INC | PP_OD,           // ctrl 3 c - BDU | PP_OD | SIM_4WIRE(0) | IF_INC
      0x02 | I2C_EN | LPF1_SEL_G,     // ctrl 4 c - DISABLE_I2C(0|4) | LFP1_SEL_G
      0x00,                           // ctrl 5 c (default)
      G_BW,                           // ctrl 6 c - XL_HM_MODE(0) | LPF1_G_BW(0x03) - turned off LPF1
      HP_EN_G,                        // ctrl 7 g (default) HPF disable(0)
      0x00,                           // ctrl 8 xl (default) - LPF_XL(0) ODR/2, XL_FS_MODE(0) = 2G-16G
      DEN | I3C_DIS,                  // ctrl 9 xl - DISABLE_I3C(2)
      TS_DIS,                         // ctrl 10 c (default) TIMESTAMP_DISABLED
  };

  if (comm->write(comm->config, 0x07, blk0, sizeof(blk0)) < 0) return -1;
  if (comm->write(comm->config, REG_CTRL1_XL, blk1, sizeof(blk1)) < 0) return -1;

  // printf("> lsm6dsox_init end:  hw: %d  comm: %d\n", hw, hw->comm);
  // lsm6dsox_dump(hw);

  return 0;
}

void lsm6dsox_dump(lsm6dsox_io_t *hw) {
  // if (hw == NULL) return GCIS_ERROR_IO_NULL;
  uint8_t buff[10];
  comm_interface_t *comm = hw->comm;

  if (comm->read(comm->config, 0x07, buff, 8) < 0) {
    printf("*** Error reading FIFO_CTRL_REG ***\n");
    return;
  }

  for (int i = 0; i < 8; ++i) {
    printf("> reg[%#x]: 0x%02x\n", 7 + i, buff[i]);
  }

  printf("---------------------------\n");

  if (comm->read(comm->config, 0x10, buff, 10) < 0) {
    printf("*** Error reading CTRL1_XL_REG ***\n");
    return;
  }
  for (int i = 0; i < 10; ++i) {
    printf("> reg[0x%02x]: 0x%02x\n", 0x10 + i, buff[i]);
  }
}

// accel - g's, gyro - dps, temp - C
int lsm6dsox_read(lsm6dsox_io_t *hw, imuf_t *imu) {
  if (hw == NULL || imu == NULL) return GCIS_ERROR_IO_NULL;
  // printf("lsm6dsox_read hw: %d  comm: %d\n", hw, hw->comm);

  uint8_t *buff = hw->buff;
  float as      = hw->a_scale;
  float gs      = hw->g_scale;

  comm_interface_t *comm = hw->comm;

  memset(buff, 0, LSM6DSOX_BUFFER_SIZE);
  memset(imu, 0, sizeof(imuf_t));

  if (comm->read(comm->config, REG_OUT_TEMP_L, buff, LSM6DSOX_BUFFER_SIZE) < 0) return -1;

  // for (int i = 0; i < LSM6DSOX_BUFFER_SIZE; ++i)
  //   printf("buff[%d]: %u\n", i, buff[i]);

  imu->temperature = cov_bb2f(buff[0], buff[1]) * TEMP_SCALE + 25.0f;

  imu->g.x = gs * cov_bb2f(buff[2], buff[3]);
  imu->g.y = gs * cov_bb2f(buff[4], buff[5]);
  imu->g.z = gs * cov_bb2f(buff[6], buff[7]);

  imu->a.x = as * cov_bb2f(buff[8], buff[9]);
  imu->a.y = as * cov_bb2f(buff[10], buff[11]);
  imu->a.z = as * cov_bb2f(buff[12], buff[13]);

  return 0;
}

// void lsm6dsox_calibrate(lsm6dsox_io_t *hw, imuf_t *data) {
//   // if (hw == NULL) return GCIS_ERROR_IO_NULL;
//   float *acal = hw->acal;
//   float *gcal = hw->gcal;
//   // imuf_t ret;

//   vec3f_t m = data->a;
//   // accel = A * accel_meas - bias
//   data->a.x = acal[0] * m.x + acal[1] * m.y + acal[2] * m.z - acal[3];
//   data->a.y = acal[4] * m.x + acal[5] * m.y + acal[6] * m.z - acal[7];
//   data->a.z = acal[8] * m.x + acal[9] * m.y + acal[10] * m.z - acal[11];

//   m = data->g;
//   // gyro = A * gyro_meas - bias
//   data->g.x = gcal[0] * m.x + gcal[1] * m.y + gcal[2] * m.z - gcal[3];
//   data->g.y = gcal[4] * m.x + gcal[5] * m.y + gcal[6] * m.z - gcal[7];
//   data->g.z = gcal[8] * m.x + gcal[9] * m.y + gcal[10] * m.z - gcal[11];
// }

// int lsm6dsox_read_calibrated(lsm6dsox_io_t *hw, imuf_t *ret) {
//   if (hw == NULL) return GCIS_ERROR_IO_NULL;
//   if (lsm6dsox_read(hw, ret) < 0) return -1;
//   lsm6dsox_calibrate(hw, ret);
//   return 0;
// }

// void lsm6dsox_set_cal(lsm6dsox_io_t *hw, float a[12], float g[12]) {
//   // if (hw == NULL) return GCIS_ERROR_IO_NULL;
//   const uint32_t size = 12 * sizeof(float);
//   memcpy(hw->acal, a, size);
//   memcpy(hw->gcal, g, size);
// }

// int lsm6dsox_i2c_init(lsm6dsox_io_t *hw, uint8_t port, uint8_t addr, lsm6dsox_xl_range_t accel_range, lsm6dsox_g_range_t gyro_range, lsm6dsox_odr_t odr) {
//   if (hw == NULL) hw = lsm6dsox_create(I2C_INTERFACE, port, addr);
//   if (hw == NULL) return -1;
//   return lsm6dsox_init(hw, accel_range, gyro_range, odr);
// }

// int lsm6dsox_spi_init(lsm6dsox_io_t *hw, uint8_t port, pin_t cs, lsm6dsox_xl_range_t accel_range, lsm6dsox_g_range_t gyro_range, lsm6dsox_odr_t odr) {
//   if (hw == NULL) hw = lsm6dsox_create(SPI_INTERFACE, port, cs);
//   if (hw == NULL) return -1;
//   printf("> lsm6dsox_spi_init:  hw: %d  comm: %d\n", hw, hw->comm);
//   return lsm6dsox_init(hw, accel_range, gyro_range, odr);
// }

// int lsm6dsox_spi_int_init(lsm6dsox_io_t *hw, uint8_t port, pin_t cs, pin_t interrupt, void (*func)(void), lsm6dsox_odr_t odr) {
//   if (hw == NULL) hw = lsm6dsox_create(SPI_INTERFACE, port, cs);
//   // setup interrupt
//   gpio_set_irq_enabled(interrupt, GPIO_IRQ_EDGE_RISE, true);
//   gpio_add_raw_irq_handler(interrupt, func);
//   irq_set_enabled(IO_IRQ_BANK0, true);
//   return lsm6dsox_init(hw, LSM6DSOX_XL_8_G, LSM6DSOX_G_2000_DPS, odr);
// }

// MSB 10000101 LSB = 128 + 4 + 1 = 133
// bool lsm6dsox_reboot(lsm6dsox_io_t *hw) {
//   comm_interface_t *comm = hw->comm;
//   uint8_t cmd            = 133;
//   // return writeRegister(REG_CTRL3_C, 133);
//   if (comm->write(comm->config, REG_CTRL3_C, &cmd, 1) < 0) return false;
//   return true;
// }

// int32_t lsm6dsox_available(lsm6dsox_io_t *hw) {
//   // TDA: temperature
//   // GDA: gyro
//   // XLDA: accel
//   //                             4   2    1
//   // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
//   uint8_t val            = 0;
//   comm_interface_t *comm = hw->comm;
//   // readRegister(REG_STATUS, &val);
//   if (comm->read(comm->config, REG_STATUS, &val, 1) < 0) return -1;
//   return val;
// }

// bool lsm6dsox_ready(lsm6dsox_io_t *hw) {
//   // int32_t val = lsm6dsox_available(hw);
//   return (lsm6dsox_available(hw) < 3) ? false : true;
// }

// void lsm6dsox_free(lsm6dsox_io_t *hw) {
//   free(hw->comm);
//   free(hw);
// }

// static inline float cov(uint8_t lo, uint8_t hi) {
//   return (float)((int16_t)((uint16_t)hi << 8) | lo);
// }

// // accel - g's, gyro - dps, temp - C
// imuf_t lsm6dsox_read(lsm6dsox_io_t *hw) {
//   int32_t err;
//   imuf_t ret;
//   uint8_t *buff          = hw->buff;
//   float as               = hw->a_scale;
//   float gs               = hw->g_scale;
//   comm_interface_t *comm = hw->comm;

//   memset(buff, 0, LSM6DSOX_BUFFER_SIZE);

//   err = comm->read(comm->config, REG_OUT_TEMP_L, buff, LSM6DSOX_BUFFER_SIZE);
//   // err = comm->read(comm->config, 0x22, buff, LSM6DSOX_BUFFER_SIZE);
//   if (err < 0) {
//      = false;
//     return ret;
//   }

//   // for (int i = 0; i < LSM6DSOX_BUFFER_SIZE; ++i)
//   //   printf("buff[%d]: %u\n", i, buff[i]);

//   ret.temperature = cov_bb2f(buff[0], buff[1]) * TEMP_SCALE + 25.0f;

//   ret.g.x = gs * cov_bb2f(buff[2], buff[3]);
//   ret.g.y = gs * cov_bb2f(buff[4], buff[5]);
//   ret.g.z = gs * cov_bb2f(buff[6], buff[7]);

//   ret.a.x = as * cov_bb2f(buff[8], buff[9]);
//   ret.a.y = as * cov_bb2f(buff[10], buff[11]);
//   ret.a.z = as * cov_bb2f(buff[12], buff[13]);

//   // ret.temperature = (float)((int16_t)((uint16_t)buff[1] << 8) | buff[0]) * TEMP_SCALE + 25.0f;

//   // ret.g.x = gs * (float)((int16_t)((uint16_t)buff[3] << 8) | buff[2]);
//   // ret.g.y = gs * (float)((int16_t)((uint16_t)buff[5] << 8) | buff[4]);
//   // ret.g.z = gs * (float)((int16_t)((uint16_t)buff[7] << 8) | buff[6]);

//   // ret.a.x = as * (float)((int16_t)((int16_t)buff[9] << 8) | buff[8]);
//   // ret.a.y = as * (float)((int16_t)((int16_t)buff[11] << 8) | buff[10]);
//   // ret.a.z = as * (float)((int16_t)((int16_t)buff[13] << 8) | buff[12]);

//   // ret.g.x = gs * (float)(((int16_t)buff[1] << 8) | buff[0]);
//   // ret.g.y = gs * (float)(((int16_t)buff[3] << 8) | buff[2]);
//   // ret.g.z = gs * (float)(((int16_t)buff[5] << 8) | buff[4]);

//   // ret.a.x = as * (float)(((int16_t)buff[7] << 8) | buff[6]);
//   // ret.a.y = as * (float)(((int16_t)buff[9] << 8) | buff[8]);
//   // ret.a.z = as * (float)(((int16_t)buff[11] << 8) | buff[10]);

//    = true;

//   return ret;
// }

// #define REG_FIFO_CTRL4 0x0A
// #define REG_INT1_CTRL  0x0D // Set interrupts for INT1 pin
// #define REG_INT2_CTRL  0x0E // Set interrupts for INT2 pin
// #define REG_CTRL2_G    0x11 // Gyro settings hz and dps
// #define REG_CTRL4_C    0x13
// #define REG_CTRL5_C    0x14
// #define REG_CTRL6_C    0x15 // Accel perf mode and Gyro LPF
// #define REG_CTRL7_G    0x16 // Gyro filtering
// #define REG_CTRL8_XL   0x17 // Accel filtering
// #define REG_CTRL9_XL   0x18 // Accel filtering
// #define REG_CTRL10_C   0x19 // tiimestamp
// #define REG_OUTX_L_G   0x22 // gyro
// #define REG_OUTX_L_A   0x28 // accel
// #define REG_TIMESTAMP0 0x40 // 4B timestamp
// #define IF_INC       0x04
// #define XL_FS_MODE   0x02 // new mode, default 0
// #define TIMESTAMP_EN 0x20
// #define LPF2_XL_EN   0x02 // output from LPF2 second
// filtering stage selected
// (not default)
// #define H_LACTIVE    0x20 // 0-high, 1-low - don't set this

// lsm6dsox_io_t *lsm6dsox_init(interface_t type, uint8_t port, uint8_t addr_cs, uint8_t accel_range, uint8_t gyro_range, uint8_t odr) {
//   // uint8_t id;
//   int32_t ok;
//   uint8_t cmd;
//   lsm6dsox_io_t *hw = (lsm6dsox_io_t *)calloc(1, sizeof(lsm6dsox_io_t));
//   if (hw == NULL) return NULL;

//   // comm_interface_t *comm = (comm_interface_t*) calloc(1, sizeof(comm_interface_t));
//   comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
//   if (comm == NULL) return NULL;

//   hw->comm = comm;

//   // printf("read whoami\n");

//   // lsm6dsox_io_t *hw = (lsm6dsox_io_t *)calloc(1, sizeof(lsm6dsox_io_t));
//   // if (hw == NULL) return NULL;
//   // hw->comm.i2c  = (port == 0) ? i2c0 : i2c1;
//   // hw->comm.addr = addr;
//   // readRegister(REG_WHO_AM_I, &id);
//   // if (!(id == WHO_AM_I)) return ERROR_WHOAMI;
//   ok = comm->read(comm->config, REG_WHO_AM_I, &cmd, 1);
//   if ((cmd != WHO_AM_I) || (ok < 0)) return NULL; // ERROR_WHOAMI;
//   // printf("correct answer\n");

//   // hw->calibrated = false;
//   float acal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
//   memcpy(hw->acal, acal, 12 * sizeof(float));
//   float gcal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
//   memcpy(hw->gcal, gcal, 12 * sizeof(float));

//   // reset memory
//   // MSB [ BOOT BDU H_LACTIVE PP_OD SIM IF_INC 0 SW_RESET ] LSB
//   // if (!writeRegister(REG_CTRL3_C, 0x84)) return 99;
//   // cmd = 0x84;
//   // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
//   // if (ok < 0) return NULL;

//   // Set the Accelerometer control register to work at 104 Hz, 4 g,and in
//   // bypass mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
//   // datasheet)
//   // kill LFP2 (default)
//   switch (accel_range) {
//   case XL_2_G:
//     hw->a_scale = 2.0f / 32768.0f;
//     break;
//   case XL_4_G:
//     hw->a_scale = 4.0f / 32768.0f;
//     break;
//   case XL_8_G:
//     hw->a_scale = 8.0f / 32768.0f;
//     break;
//   case XL_16_G:
//     hw->a_scale = 16.0f / 32768.0f;
//     break;
//   default:
//     return NULL;
//   }

//   // uint8_t regs[12] = {
//   //   odr | accel_range, // REG_CTRL1_XL
//   //   odr | gyro_range,  // REG_CTRL2_G
//   //   IF_INC,            // REG_CTRL3_C
//   //   0x08,              // REG_FIFO_CTRL4
//   //   0x00, // 5
//   //   0x00, // 6
//   //   0x00, // REG_CTRL7_G
//   //   0x00, // REG_CTRL8_XL
//   //   0x00, // REG_CTRL9_XL
//   //   0x00, // REG_CTRL10_C
//   //   INT_DRDY_XL, // REG_INT1_CTRL
//   //   INT_DRDY_G // REG_INT2_CTRL
//   // }

//   // if (!writeRegister(REG_CTRL1_XL, odr | accel_range))
//   //   return ERROR_CTRL1_XL;
//   cmd = odr | accel_range;
//   ok  = comm->write(comm->config, REG_CTRL1_XL, &cmd, 1);
//   if (ok < 0) return NULL;

//   if (gyro_range == G_125_DPS) hw->g_scale = 125.0f / 32768.0f;
//   else if (gyro_range == G_250_DPS) hw->g_scale = 250.0f / 32768.0f;
//   else if (gyro_range == G_500_DPS) hw->g_scale = 500.0f / 32768.0f;
//   else if (gyro_range == G_1000_DPS) hw->g_scale = 1000.0f / 32768.0f;
//   else if (gyro_range == G_2000_DPS) hw->g_scale = 2000.0f / 32768.0f;
//   else return NULL;

//   // if (!writeRegister(REG_CTRL2_G, odr | gyro_range)) return ERROR_CTRL2_G;
//   cmd = odr | gyro_range;
//   ok  = comm->write(comm->config, REG_CTRL2_G, &cmd, 1);
//   if (ok < 0) return NULL;

//   // auto-increament during multi-byte reads
//   // continous sampling BDU = 0
//   // if (!writeRegister(REG_CTRL3_C, IF_INC)) return ERROR_CTRL3_C;
//   cmd = IF_INC;
//   ok  = comm->write(comm->config, REG_CTRL3_C, &cmd, 1);
//   if (ok < 0) return NULL;

//   // disable fifo
//   // LSM6DSOX_FIFO_CTRL4 bypassmode (0)
//   // if (!writeRegister(REG_FIFO_CTRL4, 0x00)) return ERROR_DISABLE_FIFO;
//   // cmd = 0x00;
//   // ok = comm->write(comm->config, REG_FIFO_CTRL4, &cmd, 1);
//   // if (ok < 0) return NULL;
//   // uint8_t DRDY_MASK = 0x08;
//   // if (!writeRegister(REG_FIFO_CTRL4, DRDY_MASK)) return ERROR_CTRL4_C;
//   cmd = 0x08;
//   ok  = comm->write(comm->config, REG_FIFO_CTRL4, &cmd, 1);
//   if (ok < 0) return NULL;

//   // set gyroscope power mode to high performance and bandwidth to 16 MHz
//   // if (!writeRegister(REG_CTRL7_G, 0x00)) return ERROR_CTRL7_G;
//   cmd = 0x00;
//   ok  = comm->write(comm->config, REG_CTRL7_G, &cmd, 1);
//   if (ok < 0) return NULL;

//   // Set LPF and HPF config register
//   // LPF ODR/2, HPCF_XL = 0, LPF2_XL_EN = 0
//   // disable HPF, HP_REF_MODE_XL = 0x00
//   // if (!writeRegister(REG_CTRL8_XL, 0x00)) return ERROR_CTRL8_XL;
//   cmd = 0x00;
//   ok  = comm->write(comm->config, REG_CTRL8_XL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // disable I3C
//   // LSM6DSOX_CTRL9_XL
//   // LSM6DSOX_I3C_BUS_AVB
//   // uint8_t val = 0xD0; // 0b1110000; // these are default
//   // if (!writeRegister(REG_CTRL9_XL, val)) return ERROR_CTRL9_XL;

//   // take default of REG_CTRL9_XL
//   //
//   // cmd = 0xD1;
//   // ok  = comm->write(comm->config, REG_CTRL9_XL, &cmd, 1);
//   // if (ok < 0) return NULL;

//   // disable timestamp ... using interrupts now
//   // enable timestamp
//   // if (!writeRegister(REG_CTRL10_C, TIMESTAMP_EN))
//   //   return ERROR_CTRL10_C;
//   // cmd = TIMESTAMP_EN;
//   // ok  = comm->write(comm->config, REG_CTRL10_C, &cmd, 1);
//   // if (ok < 0) return NULL;

//   // enable INT1 and INT2 pins when data is ready
//   // if (!writeRegister(REG_INT1_CTRL, INT_DRDY_XL))
//   //   return ERROR_ENABLE_INT_ACCEL; // accel
//   cmd = INT_DRDY_XL;
//   ok  = comm->write(comm->config, REG_INT1_CTRL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // if (!writeRegister(REG_INT2_CTRL, INT_DRDY_G))
//   //   return ERROR_ENABLE_INT_GYRO; // gyro
//   cmd = INT_DRDY_G;
//   ok  = comm->write(comm->config, REG_INT2_CTRL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // printf("complete lsm6dsox\n");

//   return hw;
// }

// lsm6dsox_io_t *lsm6dsox_spi_init(
//                   uint8_t port,
//                   // uint8_t miso, uint8_t mosi, uint8_t clk,
//                   uint8_t cs,
//                   uint8_t accel_range, uint8_t gyro_range,
//                   uint8_t odr) {
//   uint8_t id;
//   int32_t ok;
//   uint8_t cmd;

//   // printf("setup lsm6dox i2c\n");

//   lsm6dsox_io_t *hw = (lsm6dsox_io_t *)calloc(1, sizeof(lsm6dsox_io_t));
//   if (hw == NULL) return NULL;

//   comm_interface_t *comm = comm_interface_init(port, cs, SPI_INTERFACE);
//   if (comm == NULL) return NULL;

//   hw->comm = comm;

//   // comm_interface_t *comm = (comm_interface_t*) calloc(1, sizeof(comm_interface_t));
//   // if (comm == NULL) return NULL;

//   // spi_config_t *config = (spi_config_t*) calloc(1, sizeof(spi_config_t));
//   // if (config == NULL) return NULL;
//   // config->spi  = (port == 0) ? spi0 : spi1;
//   // config->cs_pin = cs;

//   // hw->comm = comm;
//   // hw->comm->read = spi_read;
//   // hw->comm->write = spi_write;
//   // hw->comm->config = config;

//   // printf("i2c setup done\n");

//   return lsm6dsox_init(hw, accel_range, gyro_range, odr);
// }

// lsm6dsox_i2c_t *lsm6dsox_i2c_init(uint8_t port, uint8_t addr, uint8_t accel_range, uint8_t gyro_range, uint8_t odr) {
//   uint8_t id;
//   int32_t ok;
//   uint8_t cmd;

//   lsm6dsox_i2c_t *hw = (lsm6dsox_i2c_t *)calloc(1, sizeof(lsm6dsox_i2c_t));
//   if (hw == NULL) return NULL;
//   hw->i2c  = (port == 0) ? i2c0 : i2c1;
//   hw->addr = addr;
//   // readRegister(REG_WHO_AM_I, &id);
//   // if (!(id == WHO_AM_I)) return ERROR_WHOAMI;
//   ok = gci_i2c_read(hw->i2c, hw->addr, REG_WHO_AM_I, &id, 1);
//   if ((id != WHO_AM_I) || (ok < 0)) return NULL; // ERROR_WHOAMI;

//   float acal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
//   memcpy(hw->acal, acal, 12 * sizeof(float));
//   float gcal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
//   memcpy(hw->gcal, gcal, 12 * sizeof(float));

//   // reset memory
//   // MSB [ BOOT BDU H_LACTIVE PP_OD SIM IF_INC 0 SW_RESET ] LSB
//   // if (!writeRegister(REG_CTRL3_C, 0x84)) return 99;
//   // cmd = 0x84;
//   // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
//   // if (ok < 0) return NULL;

//   // Set the Accelerometer control register to work at 104 Hz, 4 g,and in
//   // bypass mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
//   // datasheet)
//   // kill LFP2 (default)
//   switch (accel_range) {
//   case XL_2_G:
//     hw->a_scale = 2.0f / 32768.0f;
//     break;
//   case XL_4_G:
//     hw->a_scale = 4.0f / 32768.0f;
//     break;
//   case XL_8_G:
//     hw->a_scale = 8.0f / 32768.0f;
//     break;
//   case XL_16_G:
//     hw->a_scale = 16.0f / 32768.0f;
//     break;
//   default:
//     return NULL;
//   }

//   // if (!writeRegister(REG_CTRL1_XL, odr | accel_range))
//   //   return ERROR_CTRL1_XL;
//   cmd = odr | accel_range;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL1_XL, &cmd, 1);
//   if (ok < 0) return NULL;

//   if (gyro_range == G_125_DPS) hw->g_scale = 125.0f / 32768.0f;
//   else if (gyro_range == G_250_DPS) hw->g_scale = 250.0f / 32768.0f;
//   else if (gyro_range == G_500_DPS) hw->g_scale = 500.0f / 32768.0f;
//   else if (gyro_range == G_1000_DPS) hw->g_scale = 1000.0f / 32768.0f;
//   else if (gyro_range == G_2000_DPS) hw->g_scale = 2000.0f / 32768.0f;
//   else return NULL;

//   // if (!writeRegister(REG_CTRL2_G, odr | gyro_range)) return ERROR_CTRL2_G;
//   cmd = odr | gyro_range;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL2_G, &cmd, 1);
//   if (ok < 0) return NULL;

//   // auto-increament during multi-byte reads
//   // continous sampling BDU = 0
//   // if (!writeRegister(REG_CTRL3_C, IF_INC)) return ERROR_CTRL3_C;
//   cmd = IF_INC;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
//   if (ok < 0) return NULL;

//   // disable fifo
//   // LSM6DSOX_FIFO_CTRL4 bypassmode (0)
//   // if (!writeRegister(REG_FIFO_CTRL4, 0x00)) return ERROR_DISABLE_FIFO;
//   // cmd = 0x00;
//   // ok = gci_i2c_write(hw->i2c, hw->addr, REG_FIFO_CTRL4, &cmd, 1);
//   // if (ok < 0) return NULL;
//   // uint8_t DRDY_MASK = 0x08;
//   // if (!writeRegister(REG_FIFO_CTRL4, DRDY_MASK)) return ERROR_CTRL4_C;
//   cmd = 0x08;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_FIFO_CTRL4, &cmd, 1);
//   if (ok < 0) return NULL;

//   // set gyroscope power mode to high performance and bandwidth to 16 MHz
//   // if (!writeRegister(REG_CTRL7_G, 0x00)) return ERROR_CTRL7_G;
//   cmd = 0x00;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL7_G, &cmd, 1);
//   if (ok < 0) return NULL;

//   // Set LPF and HPF config register
//   // LPF ODR/2, HPCF_XL = 0, LPF2_XL_EN = 0
//   // disable HPF, HP_REF_MODE_XL = 0x00
//   // if (!writeRegister(REG_CTRL8_XL, 0x00)) return ERROR_CTRL8_XL;
//   cmd = 0x00;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL8_XL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // disable I3C
//   // LSM6DSOX_CTRL9_XL
//   // LSM6DSOX_I3C_BUS_AVB
//   // uint8_t val = 0xD0; // 0b1110000; // these are default
//   // if (!writeRegister(REG_CTRL9_XL, val)) return ERROR_CTRL9_XL;
//   cmd = 0xD1;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL9_XL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // enable timestamp
//   // if (!writeRegister(REG_CTRL10_C, TIMESTAMP_EN))
//   //   return ERROR_CTRL10_C;
//   cmd = TIMESTAMP_EN;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL10_C, &cmd, 1);
//   if (ok < 0) return NULL;

//   // enable INT1 and INT2 pins when data is ready
//   // if (!writeRegister(REG_INT1_CTRL, INT_DRDY_XL))
//   //   return ERROR_ENABLE_INT_ACCEL; // accel
//   cmd = INT_DRDY_XL;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_INT1_CTRL, &cmd, 1);
//   if (ok < 0) return NULL;

//   // if (!writeRegister(REG_INT2_CTRL, INT_DRDY_G))
//   //   return ERROR_ENABLE_INT_GYRO; // gyro
//   cmd = INT_DRDY_G;
//   ok  = gci_i2c_write(hw->i2c, hw->addr, REG_INT2_CTRL, &cmd, 1);
//   if (ok < 0) return NULL;

//   return hw;
// }

// lsm6dsox_i2c_t *lsm6dsox_i2c_init_defaults(uint8_t port) {
//   return lsm6dsox_i2c_init(
//       port,
//       LSM6DSOX_ADDRESS,
//       XL_4_G,
//       G_2000_DPS,
//       ODR_104_HZ);
// }

// // MSB 10000101 LSB = 128 + 4 + 1 = 133
// bool lsm6dsox_reboot(lsm6dsox_i2c_t *hw) {
//   uint8_t cmd = 133;
//   // return writeRegister(REG_CTRL3_C, 133);
//   int32_t ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
//   if (ok < 0) return false;
//   return true;
// }

// // const lsm6dsox_raw_t read_raw() {
// //   lsm6dsox_raw_t ret;
// //   ret.ok = false;

// //   // Serial.println(ready());
// //   if (!ready(lsm6dsox_ready)) return ret;

// //   if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;

// //   ret.a.x  = block.regs.a.x;
// //   ret.a.y  = block.regs.a.y;
// //   ret.a.z  = block.regs.a.z;
// //   ret.g.x  = block.regs.g.x;
// //   ret.g.y  = block.regs.g.y;
// //   ret.g.z  = block.regs.g.z;
// //   ret.temperature = block.regs.temperature; // 52Hz, pg13, Table 4

// //   if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;

// //   // pg 13, Table 4, temp ODR is ~52Hz
// //   ret.timestamp = block.timestamp; // 25 usec per count
// //   ret.ok = true;

// //   return ret;
// // }

// // accel - g's, gyro - dps, temp - C
// imuf_t lsm6dsox_read(lsm6dsox_i2c_t *hw) {
//   int32_t ok;
//   imuf_t ret;
//   ret.ok = false;
//   printf("start read\n");
//   if (lsm6dsox_ready(hw) == false) return ret;
//   printf("ready\n");

//   // if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;
//   ok = gci_i2c_read(hw->i2c, hw->addr, REG_OUT_TEMP_L, hw->block.b, LSM6DSOX_BUFFER_SIZE);
//   if (ok < 0) return ret;
//   printf("read data\n");

//   ret.a.x = hw->a_scale * (float)hw->block.regs.a.x;
//   ret.a.y = hw->a_scale * (float)hw->block.regs.a.y;
//   ret.a.z = hw->a_scale * (float)hw->block.regs.a.z;
//   ret.g.x = hw->g_scale * (float)hw->block.regs.g.x;
//   ret.g.y = hw->g_scale * (float)hw->block.regs.g.y;
//   ret.g.z = hw->g_scale * (float)hw->block.regs.g.z;
//   // ret.temperature = hw->block.regs.temperature; // 52Hz, pg13, Table 4
//   // pg 13, Table 4, temp ODR is ~52Hz
//   ret.temperature = (float)(hw->block.regs.temperature) * TEMP_SCALE + 25.0f;

//   // if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;
//   ok = gci_i2c_read(hw->i2c, hw->addr, REG_TIMESTAMP0, hw->block.b, 4);
//   if (ok < 0) return ret;

//   printf("read timestamp\n");

//   ret.timestamp_us = hw->block.timestamp * 25; // 25 usec per count
//   ret.ok           = true;

//   return ret;
// }

// // accel - g's, gyro - dps, temp - C
// imuf_t lsm6dsox_read_cal(lsm6dsox_i2c_t *hw) {
//   const imuf_t m = lsm6dsox_read(hw);
//   if (m.ok == false) return m;
//   float *acal = hw->acal;
//   float *gcal = hw->gcal;

//   imuf_t ret;
//   ret.ok = false;

//   // accel = A * accel_meas - bias
//   ret.a.x = acal[0] * m.a.x + acal[1] * m.a.y + acal[2] * m.a.z - acal[3];
//   ret.a.y = acal[4] * m.a.x + acal[5] * m.a.y + acal[6] * m.a.z - acal[7];
//   ret.a.z = acal[8] * m.a.x + acal[9] * m.a.y + acal[10] * m.a.z - acal[11];

//   // gyro = A * gyro_meas - bias
//   ret.g.x = gcal[0] * m.g.x + gcal[1] * m.g.y + gcal[2] * m.g.z - gcal[3];
//   ret.g.y = gcal[4] * m.g.x + gcal[5] * m.g.y + gcal[6] * m.g.z - gcal[7];
//   ret.g.z = gcal[8] * m.g.x + gcal[9] * m.g.y + gcal[10] * m.g.z - gcal[11];

//   ret.timestamp_us = m.timestamp_us; // 25 usec per count
//   ret.temperature  = m.temperature;
//   ret.ok           = true;

//   return ret;
// }

// int32_t lsm6dsox_available(lsm6dsox_i2c_t *hw) {
//   // TDA: temperature
//   // GDA: gyro
//   // XLDA: accel
//   //                             4   2    1
//   // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
//   uint8_t val = 0;
//   // readRegister(REG_STATUS, &val);
//   int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &val, 1);
//   printf("avail/ready: %d %d\n", ok, (int)val);
//   if (ok < 0) return ok;
//   return val;
// }

// bool lsm6dsox_ready(lsm6dsox_i2c_t *hw) {
//   // int32_t val = lsm6dsox_available(hw);
//   return (lsm6dsox_available(hw) < 3) ? false : true;
// }

// const lsm6dsox_raw_t read_raw() {
//   lsm6dsox_raw_t ret;
//   ret.ok = false;

//   // Serial.println(ready());
//   if (!ready(lsm6dsox_ready)) return ret;

//   if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;

//   ret.a.x  = block.regs.a.x;
//   ret.a.y  = block.regs.a.y;
//   ret.a.z  = block.regs.a.z;
//   ret.g.x  = block.regs.g.x;
//   ret.g.y  = block.regs.g.y;
//   ret.g.z  = block.regs.g.z;
//   ret.temperature = block.regs.temperature; // 52Hz, pg13, Table 4

//   if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;

//   // pg 13, Table 4, temp ODR is ~52Hz
//   ret.timestamp = block.timestamp; // 25 usec per count
//   ret.ok = true;

//   return ret;
// }

// accel - g's, gyro - dps, temp - C
// imuf_t lsm6dsox_read_cal(lsm6dsox_io_t *hw) {
//   const imuf_t m = lsm6dsox_read(hw);
//   if (m.ok == false) return m;
//   float *acal = hw->acal;
//   float *gcal = hw->gcal;

//   imuf_t ret;
//   ret.ok = false;

//   // accel = A * accel_meas - bias
//   ret.a.x = acal[0] * m.a.x + acal[1] * m.a.y + acal[2] * m.a.z - acal[3];
//   ret.a.y = acal[4] * m.a.x + acal[5] * m.a.y + acal[6] * m.a.z - acal[7];
//   ret.a.z = acal[8] * m.a.x + acal[9] * m.a.y + acal[10] * m.a.z - acal[11];

//   // gyro = A * gyro_meas - bias
//   ret.g.x = gcal[0] * m.g.x + gcal[1] * m.g.y + gcal[2] * m.g.z - gcal[3];
//   ret.g.y = gcal[4] * m.g.x + gcal[5] * m.g.y + gcal[6] * m.g.z - gcal[7];
//   ret.g.z = gcal[8] * m.g.x + gcal[9] * m.g.y + gcal[10] * m.g.z - gcal[11];

//   ret.timestamp_us = m.timestamp_us; // 25 usec per count
//   ret.temperature  = m.temperature;
//   ret.ok           = true;

//   return ret;
// }