// /**************************************\
//  * The MIT License (MIT)
//  * Copyright (c) 2022 Kevin Walchko
//  * see LICENSE for full details
// \**************************************/

#include "bmp390.h"
#include <math.h>

inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}

inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

static bool setODR(bmp390_i2c_t *hw, uint8_t odr) {
  uint8_t press_os, temp_os;
  uint32_t ok;
  uint8_t arg;

  // based oon sec 3.9.1, pg 26
  // and pg 14, section 3.4.1
  // WARNING: Double check these work if you change them,
  //          only certain combos work
  switch (odr) {
  case ODR_200_HZ:              // 24 cm
    press_os = OVERSAMPLING_1X; // 2.64 Pa
    temp_os  = OVERSAMPLING_1X; // 0.0050 C
    break;
  case ODR_100_HZ:              // 12 cm
    press_os = OVERSAMPLING_2X; // 1.32 Pa
    temp_os  = OVERSAMPLING_1X; // 0.0050 C
    break;
  case ODR_50_HZ:               // 3cm
    press_os = OVERSAMPLING_8X; // 0.33 Pa
    temp_os  = OVERSAMPLING_1X; // 0.0050 C
    break;
  case ODR_25_HZ:
    press_os = OVERSAMPLING_16X; // 0.17 Pa
    temp_os  = OVERSAMPLING_2X;  // 0.0025 C
    break;
  case ODR_12_5_HZ:
    press_os = OVERSAMPLING_32X;
    temp_os  = OVERSAMPLING_2X;
    break;
  default:
    return false;
  }
  // if (!writeRegister(REG_OSR, (temp_os << 3) | press_os)) return false;
  arg = ((temp_os << 3) | press_os);
  ok  = gci_i2c_write(hw->i2c, hw->addr, REG_OSR, &arg, 1);
  if (ok < 0) return false;
  // if (!writeRegister(REG_ODR, odr)) return false;
  ok = gci_i2c_write(hw->i2c, hw->addr, REG_ODR, &odr, 1);
  if (ok < 0) return false;

  return true;
}

static bool get_calib_data(bmp390_i2c_t *hw) {
  uint8_t tmp[LEN_CALIB_DATA];
  // bool ok = readRegisters(REG_CALIB_DATA, LEN_CALIB_DATA, tmp);
  // if (!ok) return false;
  int32_t ok =
      gci_i2c_read(hw->i2c, hw->addr, REG_CALIB_DATA, tmp, LEN_CALIB_DATA);
  if (ok < 0) return false;

  hw->calib.par_t1 = (float)to_16b(tmp[1], tmp[0]) / powf(2, -8);
  hw->calib.par_t2 = (float)to_16b(tmp[3], tmp[2]) / powf(2, 30);
  hw->calib.par_t3 = (float)tmp[4] / powf(2, 48);

  hw->calib.par_p1 =
      ((float)to_16b(tmp[6], tmp[5]) - powf(2, 14)) / powf(2, 20);
  hw->calib.par_p2 =
      ((float)to_16b(tmp[8], tmp[7]) - powf(2, 14)) / powf(2, 29);
  hw->calib.par_p3  = (float)tmp[9] / powf(2, 32);
  hw->calib.par_p4  = (float)tmp[10] / powf(2, 37);
  hw->calib.par_p5  = (float)to_16b(tmp[12], tmp[11]) / powf(2, -3);
  hw->calib.par_p6  = (float)to_16b(tmp[14], tmp[13]) / powf(2, 6);
  hw->calib.par_p7  = (float)tmp[15] / powf(2, 8);
  hw->calib.par_p8  = (float)tmp[16] / powf(2, 15);
  hw->calib.par_p9  = (float)to_16b(tmp[18], tmp[17]) / powf(2, 48);
  hw->calib.par_p10 = (float)tmp[19] / powf(2, 48);
  hw->calib.par_p11 = (float)tmp[20] / powf(2, 65);

  return true;
}

// bool sleep() {
//   // uint8_t op_mode = readRegister(REG_PWR_CTRL);
//   // keep bits 0-1, temp/press enable, mode = 00 (sleep)
//   // op_mode = op_mode & (0x01 | 0x02);
//   // return writeRegister(REG_PWR_CTRL, op_mode);
//   return writeRegister(REG_PWR_CTRL, 0x00); // sleep, disable temp/press
// }

static bool soft_reset(bmp390_i2c_t *hw) {
  // bool ok;
  // bool ok;
  int32_t ok;
  uint8_t arg;

  // Check for command ready status
  uint8_t cmd_rdy_status = 0;
  // if(!readRegister(REG_STATUS, &cmd_rdy_status)) return false;
  ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &cmd_rdy_status, 1);
  if (ok < 0) return false;

  // Device is ready to accept new command
  if (cmd_rdy_status & CMD_RDY_BIT) {
    // println("cmd_rdy_status is CMD_RDY");
    // Write the soft reset command in the sensor
    // datasheet, p 39, table 47, register ALWAYS reads 0x00
    // writeRegister(REG_CMD, SOFT_RESET);
    arg = SOFT_RESET;
    ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CMD, &arg, 1);
    if (ok < 0) return false;

    sleep_ms(10); // was 2 ... too quick?
    // Read for command error status
    uint8_t reg_err = 0;
    // if(!readRegister(REG_ERR, &reg_err)) return false;
    // if (reg_err & REG_CMD) return false;
    ok = gci_i2c_read(hw->i2c, hw->addr, REG_ERR, &reg_err, 1);
    if (reg_err & REG_CMD) return false;
    return true;
  }
  return false;
}

static float
compensate_temperature(bmp390_i2c_t *hw,
                       const uint32_t uncomp_temp) { // datasheet pg 55
  float pd1       = (float)uncomp_temp - hw->calib.par_t1;
  float pd2       = pd1 * hw->calib.par_t2;
  hw->calib.t_lin = pd2 + (pd1 * pd1) * hw->calib.par_t3;
  return (float)hw->calib.t_lin;
}

static float
compensate_pressure(bmp390_i2c_t *hw,
                    const uint32_t uncomp_press) { // datasheet pg 56
  float pd1, pd2, pd3, pd4, po1, po2, comp_press;
  const float up = (float)uncomp_press;
  pd1            = hw->calib.par_p6 * hw->calib.t_lin;
  pd2            = hw->calib.par_p7 * (hw->calib.t_lin * hw->calib.t_lin);
  pd3 =
      hw->calib.par_p8 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
  po1 = hw->calib.par_p5 + pd1 + pd2 + pd3;

  pd1 = hw->calib.par_p2 * hw->calib.t_lin;
  pd2 = hw->calib.par_p3 * (hw->calib.t_lin * hw->calib.t_lin);
  pd3 =
      hw->calib.par_p4 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
  po2        = up * (hw->calib.par_p1 + pd1 + pd2 + pd3);

  pd1        = up * up;
  pd2        = hw->calib.par_p9 + hw->calib.par_p10 * hw->calib.t_lin;
  pd3        = pd1 * pd2;
  pd4        = pd3 + up * up * up * hw->calib.par_p11;
  comp_press = po1 + po2 + pd4;

  return comp_press;
}

// odr = ODR_50_HZ,  iir = IIR_FILTER_COEFF_3
bmp390_i2c_t *bmp390_i2c_init(uint32_t port, uint8_t addr, uint8_t odr,
                              uint8_t iir) {
  bool ok;
  uint8_t id;
  uint32_t ret;
  bmp390_i2c_t *bmp390 = (bmp390_i2c_t *)calloc(1, sizeof(bmp390_i2c_t));
  bmp390->i2c          = (port == 0) ? i2c0 : i2c1;
  bmp390->addr         = addr;

  ret = gci_i2c_read(bmp390->i2c, bmp390->addr, REG_WHO_AM_I, &id, 1);

  if (!(id == WHO_AM_I) || (ret < 0)) return NULL; // ERROR_WHOAMI;
  if (!soft_reset(bmp390)) return NULL;            // ERROR_RESET;
  if (!get_calib_data(bmp390)) return NULL;        // ERROR_CAL_DATA;
  if (!setODR(bmp390, odr)) return NULL;           // ERROR_ODR;

  /*
  IIR Filter
  Figure 6, pg 16, datasheet
      Off:  1 step delay
  coeff 1: 10 step delay
  coeff 3: 20 step delay
  coeff 7: 40 step delay

  Table 6, p 14
              OSR    Pa   1   3   7 15 31 63 127
  ULP          x1  2.64   2 1.2 0.8
  LP           x2  1.32 1.5 0.9 0.5
  Standard     x4  0.66 1.1 0.7 0.4
  High Res     x8  0.33 0.9 0.6 0.3
  UH Res      x16  0.17
  Highest Res x32 0.085
  */
  // if (!writeRegister(REG_IIR_FILTER, iir)) return ERROR_IIR_FILTER;
  ret = gci_i2c_write(bmp390->i2c, bmp390->addr, REG_IIR_FILTER, &iir, 1);
  if (ret < 0) return NULL; // ERROR_IIR_FILTER;

  // Enable interrupt pin
  // int_od: 0 = push-pull
  // int_latch: 0 = disable
  // 1 = enable pressure/temperature interrupt in INT_STATUS reg
  uint8_t DRDY_EN      = (1 << 6);
  uint8_t INT_LEVEL_HI = (1 << 1); // 1 = active high
  // latch int pin and status reg ... do I need this?
  uint8_t INT_LATCH_EN = (1 << 2);
  uint8_t arg          = (DRDY_EN | INT_LEVEL_HI | INT_LATCH_EN);
  // ok = writeRegister(REG_INT_CTRL, DRDY_EN | INT_LEVEL_HI | INT_LATCH_EN);
  ret = gci_i2c_write(bmp390->i2c, bmp390->addr, REG_INT_CTRL, &arg, 1);
  if (ret < 0) return NULL; // ERROR_INT_PIN;

  uint8_t MODE_NORMAL = (0x03 << 4); // continous sampling
  uint8_t PRESS_EN    = 0x01;
  uint8_t TEMP_EN     = 0x02;
  arg                 = MODE_NORMAL | TEMP_EN | PRESS_EN;
  // ok = writeRegister(REG_PWR_CTRL, MODE_NORMAL | TEMP_EN | PRESS_EN);
  ret = gci_i2c_write(bmp390->i2c, bmp390->addr, REG_PWR_CTRL, &arg, 1);
  if (ret < 0) return NULL; // ERROR_PWR_MODE;

  // return NO_ERROR;
  return bmp390;
}

const bmp390_t bmp390_read(bmp390_i2c_t *hw) {
  bmp390_t ret = {0.0f, 0.0f, false};

  if (!bmp390_ready(hw)) return ret;

  // bool ok = readRegisters(REG_DATA, LEN_P_T_DATA, buffer);
  int32_t ok;
  ok = gci_i2c_read(hw->i2c, hw->addr, REG_DATA, hw->buffer, LEN_P_T_DATA);
  if (ok < 0) return ret;

  uint32_t press  = to_24b(hw->buffer);
  uint32_t temp   = to_24b(&hw->buffer[3]);

  ret.temperature = compensate_temperature(hw, temp); // do temp 1st!!!
  ret.pressure    = compensate_pressure(hw, press);

  // value?
  // bool ok  = readRegisters(REG_SENSORTIME, 3, buffer);
  // if (!ok) return ret;
  // uint32_t time = to_24b(buffer);

  ret.ok = true;
  return ret;
}

// inline const bmp390_t read() { return read_raw(); }
bool bmp390_ready(bmp390_i2c_t *hw) {
  // constexpr uint8_t DATA_READY_BIT = BITS::b3;
  // if (((readRegister(REG_INT_STATUS) & DATA_READY_BIT) == 0)) return false;
  // return true;

  uint8_t reg;
  // if (!readRegister(REG_STATUS, &reg)) return false;
  int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &reg, 1);
  if (ok < 0) return false;
  // return (PRES_READY_BIT & reg) && (TEMP_READY_BIT & reg);
  return ((PRES_READY_BIT | TEMP_READY_BIT) & reg) > 0;
  // return true;
}

// inline bool reset() { return soft_reset(); }

// protected:
//   uint8_t buffer[LEN_P_T_DATA];
//   bmp3_reg_calib_data calib;