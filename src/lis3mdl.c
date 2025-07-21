#include "gci_sensors/lis3mdl.h"
#include <stdint.h>
#include <string.h> // memcpy

// static constexpr uint32_t READ_MAG      = 6;
// static constexpr uint32_t READ_MAG_TEMP = 8;



static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
static constexpr uint8_t REG_CTRL_REG1  = 0x20;
static constexpr uint8_t REG_CTRL_REG2  = 0x21;
static constexpr uint8_t REG_CTRL_REG3  = 0x22;
static constexpr uint8_t REG_CTRL_REG4  = 0x23;
static constexpr uint8_t REG_CTRL_REG5  = 0x24;
static constexpr uint8_t REG_STATUS_REG = 0x27;
static constexpr uint8_t REG_OUT_X_L    = 0x28;
// static constexpr uint8_t REG_OUT_X_H    = 0x29;
// static constexpr uint8_t REG_OUT_Y_L    = 0x2A;
// static constexpr uint8_t REG_OUT_Y_H    = 0x2B;
// static constexpr uint8_t REG_OUT_Z_L    = 0x2C;
// static constexpr uint8_t REG_OUT_Z_H    = 0x2D;
// static constexpr uint8_t REG_TEMP_OUT_L = 0x2E;
// static constexpr uint8_t REG_TEMP_OUT_H = 0x2F;
// static constexpr uint8_t REG_INT_CFG    = 0x30;
// static constexpr uint8_t REG_INT_SRC    = 0x31;
// static constexpr uint8_t REG_INT_THS_L  = 0x32;
// static constexpr uint8_t REG_INT_THS_H  = 0x33;

static constexpr uint8_t WHO_AM_I            = 0x3D;
static constexpr uint8_t STATUS_ZYXDA        = 0x08; // 0b00001000;
static constexpr uint8_t LIS3MDL_TEMP_EN     = 0x80; // chip default off
static constexpr uint8_t LIS3MDL_FAST_ODR_EN = 0x02;
static constexpr uint8_t LIS3MDL_BDU_EN      = 0x40; // chip default off

bool lis3mdl_ready(lis3mdl_io_t *hw) {
  comm_interface_t *comm = hw->comm;
  // constexpr uint8_t ZYXDA = 0x08; // 0b00001000;

  // uint8_t val             = readRegister(REG_STATUS_REG);
  // return val & ZYXDA;
  uint8_t val = 0;
  // readRegister(REG_STATUS_REG, &val);
  int32_t ok;
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS_REG, &val, 1);
  ok = comm->read(comm->config, REG_STATUS_REG, &val, 1);
  // printf("ready: %d %d\n", ok, (int)val);
  if (ok < 0) return false;
  return (val & STATUS_ZYXDA) > 0;
}

// RANGE_4GAUSS, ODR_155HZ
static lis3mdl_io_t *lis3mdl_init(interface_t type, uint8_t port, uint8_t addr_cs, lis3mdl_range_t range, lis3mdl_odr_t odr) {
  uint8_t id = 0;
  int32_t ok;
  uint8_t cmd;
  float sm[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  lis3mdl_io_t *hw = (lis3mdl_io_t *)calloc(1, sizeof(lis3mdl_io_t));
  if (hw == NULL) return NULL;

  // comm_interface_t *comm = (comm_interface_t*) calloc(1, sizeof(comm_interface_t));
  comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  memcpy(hw->sm, sm, 12 * sizeof(float));

  // 1 g = 0.0001 T = 0.1 mT = 100 uT = 100,000 nT
  // m (1E3) * (1E-4) => (1E-1) = 0.1
  // u (1E6) * (1E-4) => (1E2) = 100 <- this is the 100 below
  // pg8, table 3
  switch (range) {
  case LIS3MDL_RANGE_4GAUSS:
    hw->scale = 100.0f / 6842.0f;
    break;
  case LIS3MDL_RANGE_8GAUSS:
    hw->scale = 100.0f / 3421.0f;
    break;
  case LIS3MDL_RANGE_12GAUSS:
    hw->scale = 100.0f / 2281.0f;
    break;
  case LIS3MDL_RANGE_16GAUSS:
    hw->scale = 100.0f / 1711.0f;
    break;
  }

  // readRegister(REG_WHO_AM_I, &id);
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_WHO_AM_I, &id, 1);
  ok = comm->read(comm->config, REG_WHO_AM_I, &id, 1);
  if (ok < 0) return NULL;
  if (id != WHO_AM_I) return NULL; // ERROR_WHOAMI;

  // uint8_t reg1 = LIS3MDL_FAST_ODR_EN | LIS3MDL_TEMP_EN | (odr << 5);
  // if (!writeRegister(REG_CTRL_REG1, reg1))
  //   return ERROR_REG1; // enable x/y-axis, temp
  // uint8_t reg1 = LIS3MDL_FAST_ODR_EN | LIS3MDL_TEMP_EN | (odr << 5);
  cmd = LIS3MDL_FAST_ODR_EN | (odr << 5);
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG1, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG1, &cmd, 1);
  if (ok < 0) return NULL;

  // if (!writeRegister(REG_CTRL_REG2, range)) return ERROR_REG2; // set range
  cmd = range;
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG2, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG2, &cmd, 1);
  if (ok < 0) return NULL;

  // if (!writeRegister(REG_CTRL_REG3, 0x00))
  //   return ERROR_REG3; // continuous sampling
  cmd = 0x00;
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG3, &cmd, 1);
  if (ok < 0) return NULL;

  // uint8_t reg4 = (odr << 2);
  // if (!writeRegister(REG_CTRL_REG4, reg4)) return ERROR_REG4; // enable z-axis
  cmd = (odr << 2);
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG4, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG4, &cmd, 1);
  if (ok < 0) return NULL;

  // if (!writeRegister(REG_CTRL_REG5, 0x00))
  //   return NULL; //ERROR_REG5; // continuous sampling / no fast read
  cmd = 0x00;
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG5, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG5, &cmd, 1);
  if (ok < 0) return NULL;

  return hw;
}

lis3mdl_io_t *lis3mdl_i2c_init(uint8_t port, uint8_t addr, lis3mdl_range_t range, lis3mdl_odr_t odr) {
  return lis3mdl_init(I2C_INTERFACE, port, addr, range, odr);
}

lis3mdl_io_t *lis3mdl_spi_init(uint8_t port, pin_t cs, lis3mdl_range_t range, lis3mdl_odr_t odr) {
  return lis3mdl_init(SPI_INTERFACE, port, cs, range, odr);
}

// bool reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); } // reboot
// memory content bool reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2);
// }  // reset to default

bool lis3mdl_reboot(lis3mdl_io_t *hw) {
  int32_t ok;
  uint8_t cmd;
  comm_interface_t *comm = hw->comm;
  // if (!writeRegister(REG_CTRL_REG3, 0x03)) return false;
  cmd = 0x03;
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG3, &cmd, 1);
  if (ok < 0) return false;
  sleep_ms(100);
  // return writeRegister(REG_CTRL_REG3, 0x00);
  cmd = 0x00;
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
  ok = comm->write(comm->config, REG_CTRL_REG3, &cmd, 1);
  if (ok < 0) return false;

  return true;
}

const lis3mdl_t lis3mdl_read(lis3mdl_io_t *hw) {
  int32_t ok;
  lis3mdl_t ret          = {0.0f, 0.0f, 0.0f};
  comm_interface_t *comm = hw->comm;
  hw->ok                 = false;

  // if (lis3mdl_ready(hw) == false) return ret;

  // if (!readRegisters(REG_OUT_X_L, READ_MAG, buff.b)) return ret;
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_OUT_X_L, hw->buff.b, MAG_BUFFER_SIZE);
  ok = comm->read(comm->config, REG_OUT_X_L, hw->buff.b, MAG_BUFFER_SIZE);
  if (ok < 0) return ret;

  ret.x = hw->scale * (float)hw->buff.s.x; // gauss
  ret.y = hw->scale * (float)hw->buff.s.y;
  ret.z = hw->scale * (float)hw->buff.s.z;

  // BROKEN????
  // ((float_t)lsb / 8.0f) + (25.0f);
  // https://github.com/STMicroelectronics/lis3mdl-pid/blob/master/lis3mdl_reg.c#L113
  // ret.temperature = (float)(raw.temperature) / 8.0f + 25.0f;
  // static_cast<float>(raw.temperature) / 8.0f + 25.0f; // pg 9, Table 4
  // printf(">> temp raw: %u", uint(raw.temperature));

  // normalize mag readings
  // if (!ret.normalize()) return ret;
  hw->ok = true;

  return ret;
}

// const lis3mdl_t lis3mdl_read_cal(lis3mdl_i2c_t *hw) {
//   const lis3mdl_t m = lis3mdl_read(hw);
//   if (m.ok == false) return m;
//   float *sm = hw->sm;

//   lis3mdl_t ret;
//   ret.x  = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z - sm[3];
//   ret.y  = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z - sm[7];
//   ret.z  = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z - sm[11];
//   ret.ok = true;

//   return ret;
// }

// const lis3mdl_raw_t read_raw() {
//   lis3mdl_raw_t ret;
//   ret.ok = false;
//   int32_t ok;

//   if (!ready()) return ret;

//   // if (!readRegisters(REG_OUT_X_L, READ_MAG, buff.b)) return ret;

//   ok = gci_i2c_read(hw->i2c, hw->addr, REG_OUT_X_L, hw->buff.b, READ_MAG);
//   if (ok < 0) return ret;

//   ret.x = buff.s.x; // counts
//   ret.y = buff.s.y;
//   ret.z = buff.s.z;
//   // ret.temperature = buff.s.temp;
//   ret.ok = true;

//   return ret;
// }