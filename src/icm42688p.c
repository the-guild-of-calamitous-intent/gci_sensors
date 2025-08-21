#include <stdio.h>
#include <string.h>

#include "gci_sensors/icm42688p.h"

// ICM-42688 register addresses
#define GYRO_CONFIG_STATIC2_REG 0x0B
#define GYRO_CONFIG_STATIC3_REG 0x0C
#define GYRO_CONFIG_STATIC4_REG 0x0D
#define GYRO_CONFIG_STATIC5_REG 0x0E
#define ACCEL_CONFIG_STATIC2_REG 0x03
#define ACCEL_CONFIG_STATIC3_REG 0x04
#define ACCEL_CONFIG_STATIC4_REG 0x05


#define DEVICE_CONFIG_REG 0x11 // SPI MODEE, RESET
#define INT_CONFIG_REG    0x14 // setup INT1 INT2
#define TEMP_DATA1_REG    0x1D
#define PWR_MGMT0_REG     0x4E
#define GYRO_CONFIG0_REG  0x4F
#define ACCEL_CONFIG0_REG 0x50
#define GYRO_CONFIG1_REG 0x51
#define GYRO_ACCEL_CONFIG0_REG 0x52
#define ACCEL_CONFIG1_REG 0x53
#define INT_CONFIG0_REG   0x63
#define INT_SOURCE0_REG   0x65
#define WHO_AM_I_REG      0x75

#define WHO_AM_I      0x47
#define OSC_ON        0x10
#define ACCEL_GYRO_ON (0x0F | OSC_ON)
// #define ACCEL_16G     0x01
// #define GYRO_2000DPS  0x00
// #define ODR_100HZ     0x06
#define SW_RESET      0x01

icm42688p_io_t *icm42688p_create(interface_t type, uint8_t port, uint8_t addr_cs) {
  // printf(">> Create LIS2MDL start\n");
  icm42688p_io_t *hw = (icm42688p_io_t *)calloc(1, sizeof(icm42688p_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
  if (comm == NULL) {
    free(hw);
    return NULL;
  }

  hw->comm = comm;
  hw->accel = ICM42688_ACCEL_4G;
  hw->gyro  = ICM42688_GYRO_2000DPS;
  hw->odr   = ICM42688_ODR_100HZ;

  float sm[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  memcpy(hw->acal, sm, 12 * sizeof(float));
  memcpy(hw->gcal, sm, 12 * sizeof(float));

  return hw;
}

//
// +y
//  ^
//  |
// +-----+
// |o    |--> +x
// | +z  |
// +-----+
//
// ADC -> Notch_Gyro -> Anti-Alias -> UI_Filter -> Out
// GYRO_NF_DIS: notch 0-enable, 1-disable
// AAF_DIS: anti-alias 0-endable, 1-disable
// ACCEL/GYRO_UI_FILT_ORD:
//  00: 1st order
//  01: 2nd order
//  10: 3rd order
//  11: reserved
//
// Low Noise (LN)
int icm42688p_init(icm42688p_io_t *hw) {
  if (hw == NULL) return GCIS_ERROR_IO_NULL;
  const comm_interface_t *comm = hw->comm;

  uint8_t who_am_i;
  if (comm->read(comm->config, WHO_AM_I_REG, &who_am_i, 1) < 0) return -1;
  if (who_am_i != WHO_AM_I) {
    printf("ICM-42688 WHO_AM_I error: 0x%02X (expected 0x%02X)\n", who_am_i, WHO_AM_I);
    return GCIS_ERROR_WHOAMI;
  }

  uint8_t cmd = SW_RESET;
  if (comm->write(comm->config, DEVICE_CONFIG_REG, &cmd, 1) < 0) return -1;
  sleep_ms(5);

  icm42688p_accel_t accel = hw->accel;
  icm42688p_gyro_t gyro   = hw->gyro;
  icm42688p_odr_t odr     = hw->odr;

  switch (accel) {
  case ICM42688_ACCEL_16G:
    hw->ascale = 1.0f / 2048.0f;
    break;
  case ICM42688_ACCEL_8G:
    hw->ascale = 1.0f / 4096.0f;
    break;
  case ICM42688_ACCEL_4G:
    hw->ascale = 1.0f / 8192.0f;
    break;
  case ICM42688_ACCEL_2G:
    hw->ascale = 1.0f / 16384.0f;
    break;
  default:
    return GCIS_ERROR_PARAM;
  }

  switch (gyro) {
  case ICM42688_GYRO_2000DPS:
    hw->gscale = 1.0f / 16.4f;
    break;
  case ICM42688_GYRO_1000DPS:
    hw->gscale = 1.0f / 32.8f;
    break;
  case ICM42688_GYRO_500DPS:
    hw->gscale = 1.0f / 65.5f;
    break;
  case ICM42688_GYRO_250DPS:
    hw->gscale = 1.0f / 131.0f;
    break;
  default:
    return GCIS_ERROR_PARAM;
  }

  // datasheet, 5.3, pg 28
  // AAF range 42-3979Hz, anything over ODR 8KHz is wasted?
  // Picked numbers for AAF BW ~ODR/2
  // ACCEL values in 0x03-0x05
  // GYRO values in 0x0C-0x0E
  uint16_t deltsqr;
  uint8_t delt, bitshift;
  switch (odr) {
    case ICM42688_ODR_8KHZ:
      delt = 63;
      deltsqr = 3968;
      bitshift = 3;
      break;
    case ICM42688_ODR_4KHZ:
      delt = 37;
      deltsqr = 1376;
      bitshift = 4;
      break;
    case ICM42688_ODR_2KHZ:
      delt = 21;
      deltsqr = 440;
      bitshift = 6;
      break;
    case ICM42688_ODR_1KHZ:
      delt = 11;
      deltsqr = 122;
      bitshift = 8;
      break;
    case ICM42688_ODR_500HZ:
      delt = 6;
      deltsqr = 36;
      bitshift = 10;
      break;
    case ICM42688_ODR_200HZ:
      delt = 2;
      deltsqr = 4;
      bitshift = 13;
      break;
    case ICM42688_ODR_100HZ:
      deltsqr = 1;
      delt = 1;
      bitshift = 15;
      break;
    case ICM42688_ODR_50HZ:
      deltsqr = 1;
      delt = 1;
      bitshift = 15;
      break;
    default:
      return GCIS_ERROR_PARAM;
  }

  const uint8_t drdy_int_clear = 0x20; // UI_DRDY_INT_CLEAR[5:4] 10: Clear on Sensor Reg Read
  const uint8_t drdy_int1_en = 0x80; // UI_DRDY_INT1_EN
  const uint8_t INT_SETUP = 0x10 | 0x08 | 0x02 | 0x01; // INT1/2: push-pull, Act High
  const uint8_t notch_dis = 0x01; // disable notch filter
  uint8_t config[][2] = {
    {PWR_MGMT0_REG, ACCEL_GYRO_ON},
    {GYRO_CONFIG0_REG, gyro | odr},
    {ACCEL_CONFIG0_REG, accel | odr},
    {GYRO_CONFIG1_REG, 0x1A}, // UI Filter 1st Order
    {GYRO_ACCEL_CONFIG0_REG, 0x11}, // default accel/gyro UI filter ODR/4
    {ACCEL_CONFIG1_REG, (2 << 1)}, // UI_FILTER 2nd Ord, DEC2 3rd Ord
    // INT Setup ------------------------------------------
    {INT_CONFIG_REG, INT_SETUP}, 
    {INT_CONFIG0_REG, drdy_int_clear},
    {INT_SOURCE0_REG, drdy_int1_en},
    // GYRO Filter ----------------------------------------
    {GYRO_CONFIG_STATIC2_REG, notch_dis},
    {GYRO_CONFIG_STATIC3_REG, (delt & 0xC0)},
    {GYRO_CONFIG_STATIC4_REG, (deltsqr & 0xFF)},
    {GYRO_CONFIG_STATIC5_REG, (bitshift << 4) | ((deltsqr >> 8) & 0xFF)},
    // ACCEL Filter ---------------------------------------
    {ACCEL_CONFIG_STATIC2_REG, (delt &0xC0) << 1},
    {ACCEL_CONFIG_STATIC3_REG, deltsqr & 0xFF},
    {ACCEL_CONFIG_STATIC4_REG, (bitshift << 4) | ((deltsqr >> 8) & 0xFF)},
  };

  for (int i=0; i<sizeof(config)/2; ++i) {
    uint8_t reg = config[i][0];
    uint8_t cfg = config[i][1];
    if (comm->write(comm->config, reg, &cfg, 1) < 0) return -1;
  }
  sleep_ms(50); // wait for sensors to stabilize

  return 0;
}

static inline int16_t to_int16(uint8_t msb, uint8_t lsb) {
  return (int16_t)((uint16_t)msb << 8) | lsb;
}

int icm42688p_read(icm42688p_io_t *hw, imuf_t *imu) {
  const comm_interface_t *comm = hw->comm;
  static uint8_t data[14];
  if (comm->read(comm->config, TEMP_DATA1_REG, data, 14) < 0) return -1;

  imu->temperature = (float)to_int16(data[0], data[1]) / 132.48f + 25.0f;

  float scale = hw->ascale;
  imu->a.x = scale * (float)to_int16(data[2], data[3]); // X
  imu->a.y = scale * (float)to_int16(data[4], data[5]); // Y
  imu->a.z = scale * (float)to_int16(data[6], data[7]); // Z

  scale    = hw->gscale;
  imu->g.x = scale * (float)to_int16(data[8], data[9]);   // X
  imu->g.y = scale * (float)to_int16(data[10], data[11]); // Y
  imu->g.z = scale * (float)to_int16(data[12], data[13]); // Z

  return 0;
}

// void cm42688p_calibrate(icm42688p_io_t *hw, imuf_t *data) {
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