// /**************************************\
//  * The MIT License (MIT)
//  * Copyright (c) 2022 Kevin Walchko
//  * see LICENSE for full details
// \**************************************/
// // #pragma once


// // #if defined(__USE_SENSOR_LIS3MDL__)

#include <stdint.h>
#include <string.h> // memcpy
#include "lis3mdl.h"

static constexpr uint32_t READ_MAG = 6;
static constexpr uint32_t READ_MAG_TEMP = 8;

  bool lis3mdl_ready(lis3mdl_i2c_t *hw) {
    // constexpr uint8_t ZYXDA = 0x08; // 0b00001000;

    // uint8_t val             = readRegister(REG_STATUS_REG);
    // return val & ZYXDA;
    uint8_t val = 0;
    // readRegister(REG_STATUS_REG, &val);
    int32_t ok;
    ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS_REG, &val, 1);
    if (ok < 0) return false;
    return (val & STATUS_ZYXDA) > 0;
  }


// RANGE_4GAUSS, ODR_155HZ
  lis3mdl_i2c_t* lis3mdl_i2c_init(uint32_t port, uint8_t addr, Range range, Odr odr) {
    uint8_t id = 0;
    int32_t ok;
    uint8_t cmd;
    float sm[12] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

    lis3mdl_i2c_t *hw = NULL;
    hw       = (lis3mdl_i2c_t *)calloc(1, sizeof(lis3mdl_i2c_t));
    if (hw == NULL) return NULL;
    hw->i2c  = (port == 0) ? i2c0 : i2c1;
    hw->addr = addr;
    memcpy(hw->sm, sm, 12*sizeof(float));

    // readRegister(REG_WHO_AM_I, &id);

    ok = gci_i2c_read(hw->i2c, hw->addr, REG_WHO_AM_I, &id, 1);
    if (ok < 0) return NULL;
    if (id != WHO_AM_I) return NULL; // ERROR_WHOAMI;

    uint8_t reg1 = LIS3MDL_FAST_ODR_EN | LIS3MDL_TEMP_EN | (odr << 5);
    uint8_t reg4 = (odr << 2);

    // if (!writeRegister(REG_CTRL_REG1, reg1))
    //   return ERROR_REG1; // enable x/y-axis, temp

    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG1, &reg1, 1);
    if (ok < 0) return NULL; // ERROR_IIR_FILTER;

    // if (!writeRegister(REG_CTRL_REG2, range)) return ERROR_REG2; // set range
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG2, &range, 1);
    if (ok < 0) return NULL; // ERROR_IIR_FILTER;
    
    // if (!writeRegister(REG_CTRL_REG3, 0x00))
    //   return ERROR_REG3; // continuous sampling
    cmd = 0x00;
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
    if (ok < 0) return NULL; // ERROR_IIR_FILTER;
    
    // if (!writeRegister(REG_CTRL_REG4, reg4)) return ERROR_REG4; // enable z-axis
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG4, &reg4, 1);
    if (ok < 0) return NULL; // ERROR_IIR_FILTER;
    
    // if (!writeRegister(REG_CTRL_REG5, 0x00))
    //   return NULL; //ERROR_REG5; // continuous sampling / no fast read
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG5, &cmd, 1);
    if (ok < 0) return NULL; // ERROR_IIR_FILTER;

    return hw;
  }

  // bool reboot() { return writeBits(REG_CTRL_REG1, 0x01, 1, 3); } // reboot
  // memory content bool reset() { return writeBits(REG_CTRL_REG1, 0x01, 1, 2);
  // }  // reset to default

  bool lis3mdl_reboot(lis3mdl_i2c_t *hw) {
    int32_t ok;
    uint8_t cmd;
    // if (!writeRegister(REG_CTRL_REG3, 0x03)) return false;
    cmd = 0x03;
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
    if (ok < 0) return false;
    sleep_ms(100);
    // return writeRegister(REG_CTRL_REG3, 0x00);
    cmd = 0x00;
    ok = gci_i2c_write(hw->i2c, hw->addr, REG_CTRL_REG3, &cmd, 1);
    if (ok < 0) return false;
  }

  // void set_cal(float cal[12]) { memcpy(sm, cal, 12 * sizeof(float)); }

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

  const lis3mdl_t lis3mdl_read(lis3mdl_i2c_t *hw) {
    int32_t ok;
    lis3mdl_t ret = {0.0f,0.0f,0.0f,false};
    // ret.ok                  = false;
    // const lis3mdl_raw_t raw = read_raw();
    // if (raw.ok == false) return ret;

    if (!lis3mdl_ready(hw)) return ret;

    // if (!readRegisters(REG_OUT_X_L, READ_MAG, buff.b)) return ret;
    ok = gci_i2c_read(hw->i2c, hw->addr, REG_OUT_X_L, hw->buff.b, READ_MAG);
    if (ok < 0) return ret;

    // ret.x = static_cast<float>(raw.x); // gauss
    // ret.y = static_cast<float>(raw.y);
    // ret.z = static_cast<float>(raw.z);
    ret.x = (float)(ret.x); // gauss
    ret.y = (float)(ret.y);
    ret.z = (float)(ret.z);

    // BROKEN????
    // ((float_t)lsb / 8.0f) + (25.0f);
    // https://github.com/STMicroelectronics/lis3mdl-pid/blob/master/lis3mdl_reg.c#L113
    // ret.temperature = (float)(raw.temperature) / 8.0f + 25.0f;
    // static_cast<float>(raw.temperature) / 8.0f + 25.0f; // pg 9, Table 4
    // printf(">> temp raw: %u", uint(raw.temperature));

    // normalize mag readings
    // if (!ret.normalize()) return ret;
    ret.ok = true;

    return ret;
  }

  const lis3mdl_t lis3mdl_read_cal(lis3mdl_i2c_t *hw) {
    const lis3mdl_t m = lis3mdl_read(hw);
    if (m.ok == false) return m;
    float *sm = hw->sm;

    lis3mdl_t ret;
    ret.x = sm[0] * m.x + sm[1] * m.y + sm[2] * m.z - sm[3];
    ret.y = sm[4] * m.x + sm[5] * m.y + sm[6] * m.z - sm[7];
    ret.z = sm[8] * m.x + sm[9] * m.y + sm[10] * m.z - sm[11];
    // ret.temperature = m.temperature;
    ret.ok = true;

    return ret;
  }



// protected:
//   // scale and bias
//   float sm[12]{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  // union {
  //   struct {
  //     int16_t x, y, z, temp;
  //   } s;
  //   // int16_t s[4]; // signed short
  //   uint8_t b[8]; // bytes
  // } buff;

  // float scale;
// };

// } // namespace LIS3MDL

// #endif // use_sensor_lis3mdl

// 1 g = 0.0001 T = 0.1 mT = 100 uT = 100,000 nT
// m (1E3) * (1E-4) => (1E-1) = 0.1
// u (1E6) * (1E-4) => (1E2) = 100 <- this is the 100 below
// pg8, table 3
// switch (range) {
// case RANGE_4GAUSS:
//   scale = 100.0f / 6842.0f;
//   break;
// case RANGE_8GAUSS:
//   scale = 100.0f / 3421.0f;
//   break;
// case RANGE_12GAUSS:
//   scale = 100.0f / 2281.0f;
//   break;
// case RANGE_16GAUSS:
//   scale = 100.0f / 1711.0f;
//   break;
// }