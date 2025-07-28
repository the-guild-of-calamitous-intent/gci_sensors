#include "gci_sensors/bmp390.h"
#include <math.h>
#include <stdlib.h> // calloc
#include <string.h> // memcpy

constexpr uint8_t BMP390_OVERSAMPLING_1X  = 0x00;
constexpr uint8_t BMP390_OVERSAMPLING_2X  = 0x01;
constexpr uint8_t BMP390_OVERSAMPLING_4X  = 0x02;
constexpr uint8_t BMP390_OVERSAMPLING_8X  = 0x03;
constexpr uint8_t BMP390_OVERSAMPLING_16X = 0x04;
constexpr uint8_t BMP390_OVERSAMPLING_32X = 0x05;

static constexpr uint8_t REG_WHO_AM_I   = 0x00;
static constexpr uint8_t REG_ERR        = 0x02;
static constexpr uint8_t REG_STATUS     = 0x03;
static constexpr uint8_t REG_DATA       = 0x04;
static constexpr uint8_t REG_INT_STATUS = 0x11;
static constexpr uint8_t REG_INT_CTRL   = 0x19;
static constexpr uint8_t REG_PWR_CTRL   = 0x1B;
static constexpr uint8_t REG_OSR        = 0x1C;
static constexpr uint8_t REG_ODR        = 0x1D;
static constexpr uint8_t REG_IIR_FILTER = 0x1F;
static constexpr uint8_t REG_TIME       = 0x0C;
static constexpr uint8_t REG_CALIB_DATA = 0x31;
static constexpr uint8_t REG_CMD        = 0x7E;

static constexpr uint8_t WHO_AM_I            = 0x60;
static constexpr uint8_t CMD_RDY_BIT         = 0x10;
static constexpr uint8_t PRES_READY_BIT      = (1 << 5);
static constexpr uint8_t TEMP_READY_BIT      = (1 << 6);
static constexpr uint8_t PRES_TEMP_READY_BIT = (1 << 5) | (1 << 6);
static constexpr uint8_t SOFT_RESET          = 0xB6;
static constexpr uint8_t LEN_CALIB_DATA      = 21;

static constexpr uint8_t MODE_NORMAL  = (0x03 << 4); // continous sampling
static constexpr uint8_t PRESS_EN     = 0x01;
static constexpr uint8_t TEMP_EN      = 0x02;
static constexpr uint8_t DRDY_EN      = (1 << 6);
static constexpr uint8_t INT_LEVEL_HI = (1 << 1); // 1 = active high
// latch int pin and status reg ... do I need this?
static constexpr uint8_t INT_LATCH_EN = (1 << 2);


// FIXME: put this in io.h
static inline uint32_t to_24b(uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16;
}
// FIXME: replace with cov_bb2f ... like lsb/msb better
static inline uint16_t to_16b(uint8_t msb, uint8_t lsb) {
  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

static bool setODR(bmp390_io_t *hw, bmp390_odr_t odr) {
  uint8_t press_os, temp_os;
  int32_t ok;
  uint8_t arg;
  comm_interface_t *comm = hw->comm;

  // based oon sec 3.9.1, pg 26
  // and pg 14, section 3.4.1
  // WARNING: Double check these work if you change them,
  //          only certain combos work
  switch (odr) {
  case BMP390_ODR_200_HZ:              // 24 cm
    press_os = BMP390_OVERSAMPLING_1X; // 2.64 Pa
    temp_os  = BMP390_OVERSAMPLING_1X; // 0.0050 C
    break;
  case BMP390_ODR_100_HZ:              // 12 cm
    press_os = BMP390_OVERSAMPLING_2X; // 1.32 Pa
    temp_os  = BMP390_OVERSAMPLING_1X; // 0.0050 C
    break;
  case BMP390_ODR_50_HZ:               // 3cm
    press_os = BMP390_OVERSAMPLING_8X; // 0.33 Pa
    temp_os  = BMP390_OVERSAMPLING_1X; // 0.0050 C
    break;
  case BMP390_ODR_25_HZ:
    press_os = BMP390_OVERSAMPLING_16X; // 0.17 Pa
    temp_os  = BMP390_OVERSAMPLING_2X;  // 0.0025 C
    break;
  case BMP390_ODR_12_5_HZ:
    press_os = BMP390_OVERSAMPLING_32X;
    temp_os  = BMP390_OVERSAMPLING_2X;
    break;
  default:
    return false;
  }
  // if (!writeRegister(REG_OSR, (temp_os << 3) | press_os)) return false;
  arg = ((temp_os << 3) | press_os);
  // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_OSR, &arg, 1);
  ok = comm->write(comm->config, REG_OSR, &arg, 1);
  if (ok < 0) return false;
  // if (!writeRegister(REG_ODR, odr)) return false;
  // ok = gci_i2c_write(hw->i2c, hw->addr, REG_ODR, &odr, 1);
  ok = comm->write(comm->config, REG_ODR, &arg, 1);
  if (ok < 0) return false;

  return true;
}

static bool get_calib_data(bmp390_io_t *hw) {
  uint8_t tmp[LEN_CALIB_DATA];
  comm_interface_t *comm = hw->comm;
  // bool ok = readRegisters(REG_CALIB_DATA, LEN_CALIB_DATA, tmp);
  // if (!ok) return false;
  int32_t ok;
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_CALIB_DATA, tmp, LEN_CALIB_DATA);
  ok = comm->read(comm->config, REG_CALIB_DATA, tmp, LEN_CALIB_DATA);
  if (ok < 0) return false;

  hw->calib.par_t1 = (float)to_16b(tmp[1], tmp[0]) / powf(2, -8);
  hw->calib.par_t2 = (float)to_16b(tmp[3], tmp[2]) / powf(2, 30);
  hw->calib.par_t3 = (float)tmp[4] / powf(2, 48);

  hw->calib.par_p1  = ((float)to_16b(tmp[6], tmp[5]) - powf(2, 14)) / powf(2, 20);
  hw->calib.par_p2  = ((float)to_16b(tmp[8], tmp[7]) - powf(2, 14)) / powf(2, 29);
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

static bool soft_reset(bmp390_io_t *hw) {
  int32_t ok;
  uint8_t arg;
  comm_interface_t *comm = hw->comm;

  // Check for command ready status
  // uint8_t cmd_rdy_status = 0;
  // if(!readRegister(REG_STATUS, &cmd_rdy_status)) return false;
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &arg, 1);
  ok = comm->read(comm->config, REG_STATUS, &arg, 1);
  if (ok < 0) return false;

  // Device is ready to accept new command
  if (arg & CMD_RDY_BIT) {
    // println("cmd_rdy_status is CMD_RDY");
    // Write the soft reset command in the sensor
    // datasheet, p 39, table 47, register ALWAYS reads 0x00
    // writeRegister(REG_CMD, SOFT_RESET);
    arg = SOFT_RESET;
    // ok  = gci_i2c_write(hw->i2c, hw->addr, REG_CMD, &arg, 1);
    ok = comm->write(comm->config, REG_CMD, &arg, 1);
    if (ok < 0) return false;

    sleep_ms(10); // was 2 ... too quick?
    // Read for command error status
    arg = 0;
    // if(!readRegister(REG_ERR, &reg_err)) return false;
    // if (reg_err & REG_CMD) return false;
    // ok = gci_i2c_read(hw->i2c, hw->addr, REG_ERR, &arg, 1);
    ok = comm->read(comm->config, REG_ERR, &arg, 1);
    if ((arg & REG_CMD) || (ok < 0)) return false;
    return true;
  }
  return false;
}

// datasheet pg 55
static float compensate_temperature(bmp390_io_t *hw, const uint32_t uncomp_temp) {
  float pd1       = (float)uncomp_temp - hw->calib.par_t1;
  float pd2       = pd1 * hw->calib.par_t2;
  hw->calib.t_lin = pd2 + (pd1 * pd1) * hw->calib.par_t3;
  return (float)hw->calib.t_lin;
}

// datasheet pg 56
static float compensate_pressure(bmp390_io_t *hw, const uint32_t uncomp_press) {
  float pd1, pd2, pd3, pd4, po1, po2, prs;
  const float up = (float)uncomp_press;

  pd1 = hw->calib.par_p6 * hw->calib.t_lin;
  pd2 = hw->calib.par_p7 * (hw->calib.t_lin * hw->calib.t_lin);
  pd3 = hw->calib.par_p8 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
  po1 = hw->calib.par_p5 + pd1 + pd2 + pd3;
  pd1 = hw->calib.par_p2 * hw->calib.t_lin;
  pd2 = hw->calib.par_p3 * (hw->calib.t_lin * hw->calib.t_lin);
  pd3 = hw->calib.par_p4 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
  po2 = up * (hw->calib.par_p1 + pd1 + pd2 + pd3); //
  pd1 = up * up;                                   //
  pd2 = hw->calib.par_p9 + hw->calib.par_p10 * hw->calib.t_lin;
  pd3 = pd1 * pd2;
  pd4 = pd3 + up * up * up * hw->calib.par_p11; //
  prs = po1 + po2 + pd4;

  return prs;
}

static bmp390_io_t *bmp390_init(interface_t type, uint8_t port, uint8_t addr, bmp390_odr_t odr, bmp390_iir_t iir) {
  bmp390_io_t *hw = (bmp390_io_t *)calloc(1, sizeof(bmp390_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, addr, type);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  uint8_t id;
  int32_t ret;
  uint8_t arg;
  // bmp390_i2c_t *hw = (bmp390_i2c_t *)calloc(1, sizeof(bmp390_i2c_t));
  // if (hw == NULL) return NULL;
  // hw->i2c  = (port == 0) ? i2c0 : i2c1;
  // hw->addr = addr;

  // ret = gci_i2c_read(hw->i2c, hw->addr, REG_WHO_AM_I, &id, 1);
  ret = comm->read(comm->config, REG_WHO_AM_I, &id, 1);

  if (!(id == WHO_AM_I) || (ret < 0)) return NULL; // ERROR_WHOAMI;
  if (soft_reset(hw) == false) return NULL;        // ERROR_RESET;
  if (get_calib_data(hw) == false) return NULL;    // ERROR_CAL_DATA;
  if (setODR(hw, odr) == false) return NULL;       // ERROR_ODR;

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
  // ret = gci_i2c_write(hw->i2c, hw->addr, REG_IIR_FILTER, &iir, 1);
  ret = comm->write(comm->config, REG_IIR_FILTER, &iir, 1);
  if (ret < 0) return NULL; // ERROR_IIR_FILTER;

  // Enable interrupt pin
  // int_od: 0 = push-pull
  // int_latch: 0 = disable
  // 1 = enable pressure/temperature interrupt in INT_STATUS reg
  // uint8_t DRDY_EN      = (1 << 6);
  // uint8_t INT_LEVEL_HI = (1 << 1); // 1 = active high
  // // latch int pin and status reg ... do I need this?
  // uint8_t INT_LATCH_EN = (1 << 2);
  // ok = writeRegister(REG_INT_CTRL, DRDY_EN | INT_LEVEL_HI | INT_LATCH_EN);
  arg = (DRDY_EN | INT_LEVEL_HI | INT_LATCH_EN);
  // ret = gci_i2c_write(hw->i2c, hw->addr, REG_INT_CTRL, &arg, 1);
  ret = comm->write(comm->config, REG_INT_CTRL, &arg, 1);
  if (ret < 0) return NULL; // ERROR_INT_PIN;

  // uint8_t MODE_NORMAL = (0x03 << 4); // continous sampling
  // uint8_t PRESS_EN    = 0x01;
  // uint8_t TEMP_EN     = 0x02;
  // ok = writeRegister(REG_PWR_CTRL, MODE_NORMAL | TEMP_EN | PRESS_EN);
  arg = MODE_NORMAL | TEMP_EN | PRESS_EN;
  // ret = gci_i2c_write(hw->i2c, hw->addr, REG_PWR_CTRL, &arg, 1);
  ret = comm->write(comm->config, REG_PWR_CTRL, &arg, 1);
  if (ret < 0) return NULL; // ERROR_PWR_MODE;

  return hw;
}

bmp390_io_t *bmp390_i2c_init(uint8_t port, uint8_t addr, bmp390_odr_t odr, bmp390_iir_t iir) {
  return bmp390_init(I2C_INTERFACE, port, addr, odr, iir);
}

bmp390_io_t *bmp390_spi_init(uint8_t port, pin_t cs, bmp390_odr_t odr, bmp390_iir_t iir) {
  return bmp390_init(SPI_INTERFACE, port, cs, odr, iir);
}

const bmp390_t bmp390_read(bmp390_io_t *hw) {
  comm_interface_t *comm = hw->comm;
  bmp390_t ret           = {0.0f, 0.0f};
  hw->ok                 = false;
  // printf("start\n");

  if (!bmp390_ready(hw)) return ret;
  // printf("ready\n");

  // bool ok = readRegisters(REG_DATA, LEN_P_T_DATA, buffer);
  int32_t ok;
  // ok = gci_i2c_read(hw->i2c, hw->addr, REG_DATA, hw->buffer, LEN_P_T_DATA);
  ok = comm->read(comm->config, REG_DATA, hw->buffer, LEN_P_T_DATA);
  if (ok < 0) return ret;
  // printf("good read\n");

  uint32_t press = to_24b(hw->buffer);
  uint32_t temp  = to_24b(&hw->buffer[3]);

  ret.temperature = compensate_temperature(hw, temp); // do temp 1st!!!
  ret.pressure    = compensate_pressure(hw, press);

  // value?
  // bool ok  = readRegisters(REG_SENSORTIME, 3, buffer);
  // if (!ok) return ret;
  // uint32_t time = to_24b(buffer);

  hw->ok = true;
  return ret;
}

// inline const bmp390_t read() { return read_raw(); }
int32_t bmp390_available(bmp390_io_t *hw) {
  comm_interface_t *comm = hw->comm;
  // constexpr uint8_t DATA_READY_BIT = BITS::b3;
  // if (((readRegister(REG_INT_STATUS) & DATA_READY_BIT) == 0)) return false;
  // return true;

  uint8_t reg;
  // if (!readRegister(REG_STATUS, &reg)) return false;
  // int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &reg, 1);
  int32_t ok = comm->read(comm->config, REG_STATUS, &reg, 1);
  if (ok < 0) return ok;
  // return (PRES_READY_BIT & reg) && (TEMP_READY_BIT & reg);
  return reg;
  // return true;
}

bool bmp390_ready(bmp390_io_t *hw) {
  return (bmp390_available(hw) < PRES_TEMP_READY_BIT) ? false : true;
}

// typedef struct {
//   float po1;
//   float po2const;
//   float a, b;
// } comp_press_t;

// static float compensate_pressure_alt(bmp390_i2c_t *hw, const uint32_t uncomp_press) { // datasheet pg 56
//   float pd1, pd2, pd3, pd4, po1, po2, prs;
//   const float up = (float)uncomp_press;
//   comp_press_t cp;

//   // po1 - const
//   // po2 - up * const(hw->calib.par_p1 + pd1 + pd2 + pd3)
//   // pd4 - up^2 * const + up^3 * const
//   // pressure = po1 + po2 + pd

//   pd1    = hw->calib.par_p6 * hw->calib.t_lin;
//   pd2    = hw->calib.par_p7 * (hw->calib.t_lin * hw->calib.t_lin);
//   pd3    = hw->calib.par_p8 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
//   po1    = hw->calib.par_p5 + pd1 + pd2 + pd3; // const
//   cp.po1 = po1;

//   pd1         = hw->calib.par_p2 * hw->calib.t_lin;
//   pd2         = hw->calib.par_p3 * (hw->calib.t_lin * hw->calib.t_lin);
//   pd3         = hw->calib.par_p4 * (hw->calib.t_lin * hw->calib.t_lin * hw->calib.t_lin);
//   po2         = up * (hw->calib.par_p1 + pd1 + pd2 + pd3); // pd1, pd2, pd3
//   cp.po2const = hw->calib.par_p1 + pd1 + pd2 + pd3;

//   pd1  = up * up; //
//   pd2  = hw->calib.par_p9 + hw->calib.par_p10 * hw->calib.t_lin;
//   pd3  = pd1 * pd2;
//   pd4  = pd3 + up * up * up * hw->calib.par_p11; //
//   cp.a = pd3;
//   cp.b = hw->calib.par_p11;

//   prs = po1 + po2 + pd4;

//   return prs;
// }

// bool bmp390_ready(bmp390_i2c_t *hw){
//   // constexpr uint8_t DATA_READY_BIT = BITS::b3;
//   // if (((readRegister(REG_INT_STATUS) & DATA_READY_BIT) == 0)) return false;
//   // return true;

//   uint8_t reg;
//   // if (!readRegister(REG_STATUS, &reg)) return false;
//   int32_t ok = gci_i2c_read(hw->i2c, hw->addr, REG_STATUS, &reg, 1);
//   // return (PRES_READY_BIT & reg) && (TEMP_READY_BIT & reg);
//   return ((PRES_READY_BIT | TEMP_READY_BIT) & reg) > 0;
//   // return true;
// }

// inline bool reset() { return soft_reset(); }

// protected:
//   uint8_t buffer[LEN_P_T_DATA];
//   bmp3_reg_calib_data calib;

// bool sleep() {
//   // uint8_t op_mode = readRegister(REG_PWR_CTRL);
//   // keep bits 0-1, temp/press enable, mode = 00 (sleep)
//   // op_mode = op_mode & (0x01 | 0x02);
//   // return writeRegister(REG_PWR_CTRL, op_mode);
//   return writeRegister(REG_PWR_CTRL, 0x00); // sleep, disable temp/press
// }