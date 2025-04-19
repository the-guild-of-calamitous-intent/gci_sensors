
#include "lsm6dsox.h"
#include <string.h> // memcpy

static constexpr uint8_t WHO_AM_I = 0x6C; // 01101100

static constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;
static constexpr uint8_t REG_INT1_CTRL  = 0x0D;
static constexpr uint8_t REG_INT2_CTRL  = 0x0E;
static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
static constexpr uint8_t REG_CTRL1_XL   = 0x10; // Accel settings
static constexpr uint8_t REG_CTRL2_G    = 0x11; // Gyro settings hz and dps
static constexpr uint8_t REG_CTRL3_C    = 0x12; // interrupt stuff
static constexpr uint8_t REG_CTRL4_C    = 0x13;
static constexpr uint8_t REG_CTRL5_C    = 0x14;
static constexpr uint8_t REG_CTRL6_C    = 0x15; // Accel perf mode and Gyro LPF
static constexpr uint8_t REG_CTRL7_G    = 0x16; // Gyro filtering
static constexpr uint8_t REG_CTRL8_XL   = 0x17; // Accel filtering
static constexpr uint8_t REG_CTRL9_XL   = 0x18; // Accel filtering
static constexpr uint8_t REG_CTRL10_C   = 0x19; // tiimestamp

static constexpr uint8_t REG_STATUS = 0x1E;

static constexpr uint8_t REG_OUT_TEMP_L = 0x20; // termperature
static constexpr uint8_t REG_OUTX_L_G   = 0x22; // gyro
static constexpr uint8_t REG_OUTX_L_A   = 0x28; // accel
static constexpr uint8_t REG_TIMESTAMP0 = 0x40; // 4B timestamp

static constexpr uint8_t IF_INC       = 0x04;
static constexpr uint8_t XL_FS_MODE   = 0x02; // new mode, default 0
static constexpr uint8_t TIMESTAMP_EN = 0x20;
static constexpr uint8_t LPF2_XL_EN   = 0x02;  // output from LPF2 second
                                               // filtering stage selected
                                               // (not default)
static constexpr uint8_t INT_DRDY_XL   = 0x01; // accel data ready INT pin
static constexpr uint8_t INT_DRDY_G    = 0x02; // gyro data ready INT pin
static constexpr uint8_t INT_DRDY_TEMP = 0x04; // temperature data ready INT pin
// static constexpr uint8_t H_LACTIVE    = 0x20; // 0-high, 1-low - don't set this

static constexpr float LSM6DSOX_TIMESTEP_RES = 25e-6f;
static constexpr float TEMP_SCALE            = 1.0f / 256.0f;

lsm6dsox_i2c_t *lsm6dsox_i2c_init(uint8_t port, uint8_t addr, uint8_t accel_range, uint8_t gyro_range, uint8_t odr) {
  uint8_t id;
  int32_t ok;
  uint8_t cmd;

  lsm6dsox_i2c_t *hw = (lsm6dsox_i2c_t *)calloc(1, sizeof(lsm6dsox_i2c_t));
  hw->i2c            = (port == 0) ? i2c0 : i2c1;
  hw->addr           = addr;
  // readRegister(REG_WHO_AM_I, &id);
  // if (!(id == WHO_AM_I)) return ERROR_WHOAMI;
  ok = gci_i2c_read(hw->i2c, hw->addr, REG_WHO_AM_I, &id, 1);
  if (!(id == WHO_AM_I) || (ok < 0)) return NULL; // ERROR_WHOAMI;

  float acal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
  memcpy(hw->acal, acal, 12 * sizeof(float));
  float gcal[12] = {1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.};
  memcpy(hw->gcal, gcal, 12 * sizeof(float));

  // reset memory
  // MSB [ BOOT BDU H_LACTIVE PP_OD SIM IF_INC 0 SW_RESET ] LSB
  // if (!writeRegister(REG_CTRL3_C, 0x84)) return 99;

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in
  // bypass mode and enable ODR/4 low pass filter (check figure9 of LSM6DSOX's
  // datasheet)
  // kill LFP2 (default)
  switch (accel_range) {
  case ACCEL_RANGE_2_G:
    hw->a_scale = 2.0f / 32768.0f;
    break;
  case ACCEL_RANGE_4_G:
    hw->a_scale = 4.0f / 32768.0f;
    break;
  case ACCEL_RANGE_8_G:
    hw->a_scale = 8.0f / 32768.0f;
    break;
  case ACCEL_RANGE_16_G:
    hw->a_scale = 16.0f / 32768.0f;
    break;
  default:
    return NULL;
  }

  // if (!writeRegister(REG_CTRL1_XL, odr | accel_range))
  //   return ERROR_CTRL1_XL;

  cmd = odr | accel_range;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL1_XL, &cmd, 1);
  if (ok < 0) return NULL;

  if (gyro_range == GYRO_RANGE_125_DPS) hw->g_scale = 125.0f / 32768.0f;
  else if (gyro_range == GYRO_RANGE_250_DPS) hw->g_scale = 250.0f / 32768.0f;
  else if (gyro_range == GYRO_RANGE_500_DPS) hw->g_scale = 500.0f / 32768.0f;
  else if (gyro_range == GYRO_RANGE_1000_DPS) hw->g_scale = 1000.0f / 32768.0f;
  else if (gyro_range == GYRO_RANGE_2000_DPS) hw->g_scale = 2000.0f / 32768.0f;
  else return NULL;

  // if (!writeRegister(REG_CTRL2_G, odr | gyro_range)) return ERROR_CTRL2_G;
  cmd = odr | gyro_range;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL2_G, &cmd, 1);
  if (ok < 0) return NULL;

  // auto-increament during multi-byte reads
  // continous sampling BDU = 0
  // if (!writeRegister(REG_CTRL3_C, IF_INC)) return ERROR_CTRL3_C;
  cmd = IF_INC;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
  if (ok < 0) return NULL;

  // disable fifo
  // LSM6DSOX_FIFO_CTRL4 bypassmode (0)
  uint8_t DRDY_MASK = 0x08;
  // if (!writeRegister(REG_FIFO_CTRL4, 0x00)) return ERROR_DISABLE_FIFO;
  // if (!writeRegister(REG_FIFO_CTRL4, DRDY_MASK)) return ERROR_CTRL4_C;
  // cmd = IF_INC;
  ok = gci_i2c_write(hw->i2c, hw->addr, REG_FIFO_CTRL4, &DRDY_MASK, 1);
  if (ok < 0) return NULL;

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  // if (!writeRegister(REG_CTRL7_G, 0x00)) return ERROR_CTRL7_G;
  cmd = 0x00;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL7_G, &cmd, 1);
  if (ok < 0) return NULL;

  // Set LPF and HPF config register
  // LPF ODR/2, HPCF_XL = 0, LPF2_XL_EN = 0
  // disable HPF, HP_REF_MODE_XL = 0x00
  // if (!writeRegister(REG_CTRL8_XL, 0x00)) return ERROR_CTRL8_XL;
  cmd = 0x00;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL8_XL, &cmd, 1);
  if (ok < 0) return NULL;

  // disable I3C
  // LSM6DSOX_CTRL9_XL
  // LSM6DSOX_I3C_BUS_AVB
  // uint8_t val = 0xD0; // 0b1110000; // these are default
  // if (!writeRegister(REG_CTRL9_XL, val)) return ERROR_CTRL9_XL;

  // enable timestamp
  // if (!writeRegister(REG_CTRL10_C, TIMESTAMP_EN))
  //   return ERROR_CTRL10_C;
  cmd = TIMESTAMP_EN;
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL10_C, &cmd, 1);
  if (ok < 0) return NULL;

  // enable INT1 and INT2 pins when data is ready
  // if (!writeRegister(REG_INT1_CTRL, INT_DRDY_XL))
  //   return ERROR_ENABLE_INT_ACCEL; // accel

  // if (!writeRegister(REG_INT2_CTRL, INT_DRDY_G))
  //   return ERROR_ENABLE_INT_GYRO; // gyro

  return hw;
}

lsm6dsox_i2c_t *lsm6dsox_i2c_init_defaults(uint8_t port) {
  return lsm6dsox_i2c_init(port, LSM6DSOX_ADDRESS, ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_104_HZ);
}

// MSB 10000101 LSB = 128 + 4 + 1 = 133
bool lsm6dsox_reboot(lsm6dsox_i2c_t *hw) {
  uint8_t cmd = 133;
  // return writeRegister(REG_CTRL3_C, 133);
  int32_t ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL3_C, &cmd, 1);
  if (ok < 0) return false;
  return true;
}

// void set_accel_cal(float cal[12]) { memcpy(acal, cal, 12 * sizeof(float)); }
// void set_gyro_cal(float cal[12]) { memcpy(gcal, cal, 12 * sizeof(float)); }

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

lsm6dsox_t lsm6dsox_read(lsm6dsox_i2c_t *hw) { // accel - g's, gyro - dps, temp - C
  // const lsm6dsox_raw_t raw = read_raw();
  int32_t ok;
  lsm6dsox_t ret;
  ret.ok = false;
  // if (raw.ok == false) return ret;
  if (!lsm6dsox_ready(hw)) return ret;

  // if (!readRegisters(REG_OUT_TEMP_L, sizeof(block.b), block.b)) return ret;
  ok = gci_i2c_read(hw->i2c, hw->addr, REG_OUT_TEMP_L, hw->block.b, 14);
  if (ok < 0) return ret;

  ret.a.x = hw->a_scale * hw->block.regs.a.x;
  ret.a.y = hw->a_scale * hw->block.regs.a.y;
  ret.a.z = hw->a_scale * hw->block.regs.a.z;
  ret.g.x = hw->g_scale * hw->block.regs.g.x;
  ret.g.y = hw->g_scale * hw->block.regs.g.y;
  ret.g.z = hw->g_scale * hw->block.regs.g.z;
  // ret.temperature = hw->block.regs.temperature; // 52Hz, pg13, Table 4
  // pg 13, Table 4, temp ODR is ~52Hz
  ret.temperature = (float)(hw->block.regs.temperature) * TEMP_SCALE + 25.0f;

  // if (!readRegisters(REG_TIMESTAMP0, 4, block.b)) return ret;
  ok = gci_i2c_read(hw->i2c, hw->addr, REG_TIMESTAMP0, hw->block.b, 4);
  if (ok < 0) return ret;

  ret.timestamp_us = hw->block.timestamp * 25; // 25 usec per count
  ret.ok           = true;

  return ret;
}

lsm6dsox_t lsm6dsox_read_cal(lsm6dsox_i2c_t *hw) { // accel - g's, gyro - dps, temp - C
  const lsm6dsox_t m = lsm6dsox_read(hw);
  if (m.ok == false) return m;
  float *acal = hw->acal;
  float *gcal = hw->gcal;

  lsm6dsox_t ret;
  ret.ok = false;

  // accel = A * accel_meas - bias
  ret.a.x = acal[0] * m.a.x + acal[1] * m.a.y + acal[2] * m.a.z - acal[3];
  ret.a.y = acal[4] * m.a.x + acal[5] * m.a.y + acal[6] * m.a.z - acal[7];
  ret.a.z = acal[8] * m.a.x + acal[9] * m.a.y + acal[10] * m.a.z - acal[11];

  // gyro = A * gyro_meas - bias
  ret.g.x = gcal[0] * m.g.x + gcal[1] * m.g.y + gcal[2] * m.g.z - gcal[3];
  ret.g.y = gcal[4] * m.g.x + gcal[5] * m.g.y + gcal[6] * m.g.z - gcal[7];
  ret.g.z = gcal[8] * m.g.x + gcal[9] * m.g.y + gcal[10] * m.g.z - gcal[11];

  ret.timestamp_us = m.timestamp_us; // 25 usec per count
  ret.temperature  = m.temperature;
  ret.ok           = true;

  return ret;
}

bool lsm6dsox_ready(lsm6dsox_i2c_t *hw) {
  // TDA: temperature
  // GDA: gyro
  // XLDA: accel
  //                             4   2    1
  // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
  // return readRegister(REG_STATUS);
  // uint8_t val = readRegister(REG_STATUS) & 3;
  uint8_t val = 0;
  // readRegister(REG_STATUS, &val);

  int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &val, 1);
  if (ok < 0) return false;

  return val < 3 ? false : true;
  // return val == 3;
}

int32_t lsm6dsox_available(lsm6dsox_i2c_t *hw) {
  // TDA: temperature
  // GDA: gyro
  // XLDA: accel
  //                             4   2    1
  // STATUS_REG: MSB 0 0 0 0 0 TDA GDA XLDA LSB
  uint8_t val = 0;
  // readRegister(REG_STATUS, &val);
  int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &val, 1);
  if (ok < 0) return ok;
  return val;
}

// sox_regs_t getRegs() {
//   uint8_t reg;
//   sox_regs_t regs;
//   readRegister(REG_CTRL1_XL, &reg);
//   regs.CTRL1_XL = reg;
//   readRegister(REG_CTRL2_G);
//   regs.CTRL2_G  = reg;
//   readRegister(REG_CTRL3_C);
//   regs.CTRL3_C  = reg;
//   regs.CTRL4_C  = readRegister(REG_CTRL4_C);
//   regs.CTRL5_C  = readRegister(REG_CTRL5_C);
//   regs.CTRL6_C  = readRegister(REG_CTRL6_C);
//   regs.CTRL7_G  = readRegister(REG_CTRL7_G);
//   regs.CTRL8_XL = readRegister(REG_CTRL8_XL);
//   regs.CTRL9_XL = readRegister(REG_CTRL9_XL);
//   regs.CTRL10_C = readRegister(REG_CTRL10_C);
//   return regs;
// }
