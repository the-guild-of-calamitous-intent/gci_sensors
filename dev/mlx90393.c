#include <stdint.h>
#include <stdio.h>

#include "gci_sensors/mlx90393.h"

#define MLX90393_AXIS_ALL      (0x0E) // X+Y+Z axis bits for commands.
#define MLX90393_CONF1         (0x00) // Gain
#define MLX90393_CONF2         (0x01) // Burst, comm mode
#define MLX90393_CONF3         (0x02) // Oversampling, filter, res.
#define MLX90393_CONF4         (0x03) // Sensitivty drift.
#define MLX90393_GAIN_SHIFT    (4)    // Left-shift for gain bits.
#define MLX90393_HALL_CONF     (0x0C) // Hall plate spinning rate adj.
#define MLX90393_STATUS_OK     (0x00) // OK value for status response.
#define MLX90393_STATUS_SMMODE (0x08) // SM Mode status response.
#define MLX90393_STATUS_RESET  (0x01) // Reset value for status response.
#define MLX90393_STATUS_ERROR  (0xFF) // OK value for status response.
#define MLX90393_STATUS_MASK   (0xFC) // Mask for status OK checks.

// Register map.
typedef enum {
  MLX90393_REG_NOP = (0x00), // NOP.
  MLX90393_REG_SB  = (0x10), // Start burst mode.
  MLX90393_REG_SW  = (0x20), // Start wakeup on change mode.
  MLX90393_REG_SM  = (0x30), // Start single-meas mode.
  MLX90393_REG_RM  = (0x40), // Read measurement.
  MLX90393_REG_RR  = (0x50), // Read register.
  MLX90393_REG_WR  = (0x60), // Write register.
  MLX90393_REG_HS  = (0x70), // Memory store.
  MLX90393_REG_EX  = (0x80), // Exit mode.
  MLX90393_REG_HR  = (0xD0), // Memory recall.
  MLX90393_REG_RT  = (0xF0), // Reset.
} mlx90393_cmds_t;

// 10 types of commands (pg 22 datasheet)
// -------------------------------------------
// SB: burst mode
//   SDO <0x10>< 0x00 >
//   SDI < xx ><Status>
// SW: wake on change
//   SDO <0x20>< 0x00 >
//   SDI < xx ><Status>
// SM: single measurement
//   SDO <0x30>< 0x00 >
//   SDI < xx ><Status>
// EX: exit
//   note: wait 1 msec after issue
//   SDO <0x80>< 0x00 >
//   SDI < xx ><Status>
// HR: memory recall
//   SDO <0xD0>< 0x00 >
//   SDI < xx ><Status>
// HS: memory store
//   SDO <0x70>< 0x00 >
//   SDI < xx ><Status>
// RT: reset
//   note: issue EX first and wait 1 msec
//   SDO <0xF0>< 0x00 >
//   SDI < xx ><Status>
// RM: read measurement
//   SDO <0x40>< 0x00 >< 0x00>< 0x00><0><0>...
//   SDI < xx ><Status><T_MSB><T_LSB><X><Y>...
// RR: read register
//   SDO <0x50><(addr<<2)>< 0x00 >< 0 >< 0 >
//   SDI < xx ><    xx   ><Status><MSB><LSB>
// WR: write register
//   SDO <0x60><MSB><LSB><(addr<<2)>< 0x00 >
//   SDI < xx >< xx>< xx><    xx   ><Status>

// Subset I use
// SB, EX, RT: write_status
// RM: write_status, normal read
// RR: write_status

// Gain settings for CONF1 register.
typedef enum {
  MLX90393_GAIN_5X = (0x00),
  MLX90393_GAIN_4X,
  MLX90393_GAIN_3X,
  MLX90393_GAIN_2_5X,
  MLX90393_GAIN_2X,
  MLX90393_GAIN_1_67X,
  MLX90393_GAIN_1_33X,
  MLX90393_GAIN_1X
} mlx90393_gain_t;

// Resolution settings for CONF3 register.
typedef enum {
  MLX90393_RES_16,
  MLX90393_RES_17,
  MLX90393_RES_18,
  MLX90393_RES_19,
} mlx90393_resolution_t;

// Axis designator.
typedef enum mlx90393_axis {
  MLX90393_X,
  MLX90393_Y,
  MLX90393_Z
} mlx90393_axis_t;

// Digital filter settings for CONF3 register.
typedef enum {
  MLX90393_FILTER_0,
  MLX90393_FILTER_1,
  MLX90393_FILTER_2,
  MLX90393_FILTER_3,
  MLX90393_FILTER_4,
  MLX90393_FILTER_5,
  MLX90393_FILTER_6,
  MLX90393_FILTER_7,
} mlx90393_filter_t;

// Oversampling settings for CONF3 register.
typedef enum {
  MLX90393_OSR_0,
  MLX90393_OSR_1,
  MLX90393_OSR_2,
  MLX90393_OSR_3,
} mlx90393_oversampling_t;

static const float mlx90393_lsb_lookup[2][8][4][2] = {

    // HALLCONF = 0xC (default)
    {
        // GAIN_SEL = 0, 5x gain
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        // GAIN_SEL = 1, 4x gain
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        // GAIN_SEL = 2, 3x gain
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        // GAIN_SEL = 3, 2.5x gain
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        // GAIN_SEL = 4, 2x gain
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        // GAIN_SEL = 5, 1.667x gain
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        // GAIN_SEL = 6, 1.333x gain
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        // GAIN_SEL = 7, 1x gain
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    // HALLCONF = 0x0
    {
        // GAIN_SEL = 0, 5x gain
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        // GAIN_SEL = 1, 4x gain
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        // GAIN_SEL = 2, 3x gain
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        // GAIN_SEL = 3, 2.5x gain
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        // GAIN_SEL = 4, 2x gain
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        // GAIN_SEL = 5, 1.667x gain
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        // GAIN_SEL = 6, 1.333x gain
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        // GAIN_SEL = 7, 1x gain
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }};

// Lookup table for conversion time based on [DIF_FILT][OSR].
static const float mlx90393_tconv[8][4] = {
    // DIG_FILT = 0
    {1.27, 1.84, 3.00, 5.30},
    // DIG_FILT = 1
    {1.46, 2.23, 3.76, 6.84},
    // DIG_FILT = 2
    {1.84, 3.00, 5.30, 9.91},
    // DIG_FILT = 3
    {2.61, 4.53, 8.37, 16.05},
    // DIG_FILT = 4
    {4.15, 7.60, 14.52, 28.34},
    // DIG_FILT = 5
    {7.22, 13.75, 26.80, 52.92},
    // DIG_FILT = 6
    {13.36, 26.04, 51.38, 102.07},
    // DIG_FILT = 7
    {25.65, 50.61, 100.53, 200.37},
};

// #define CMD_READ_REGISTER 0x50
// #define CMD_WRITE_REGISTER 0x60
// read:  <0x50><(REG<<2)><DH><DL><0x00>
// write: <0x60><DH><DL><(REG<<2)><0x00>
mlx90393_io_t *mlx90393_init(interface_t type, uint8_t port, uint8_t addr_cs) {
  mlx90393_io_t *hw = (mlx90393_io_t *)calloc(1, sizeof(mlx90393_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, addr_cs, type);
  if (comm == NULL) return NULL;
  if (type == SPI_INTERFACE) {
    comm->read  = spi_read_status;
    comm->write = spi_write_status;
  }

  hw->comm = comm;

  hw->gain = 0;
  hw->res  = 0;

  uint8_t nop       = 0x00;
  int result        = 0;
  uint8_t config[3] = {
      0x00,     // bist diabled=0
      0x5C,     // gain_sel=5, hall=default // zseries=0,gain=5,hallconf=0
      0x00 << 2 // addr
  };

  result = comm->write(comm->config, 0x60, config, 3); // write
  if (result < 0) return NULL;
  // result = comm->read(comm->config, 0x00, &nop, 1);
  // if (result < 0) return NULL;

  config[0] = 0x02;      // osr2=0, res_z=1, res_y=0
  config[1] = 0xB4;      // res_y=1,res_x=01,dig_filt=101,osr=0
  config[2] = 0x02 << 2; // addr

  result = comm->write(comm->config, 0x60, config, 3); // write
  if (result < 0) return NULL;
  // result = comm->read(comm->config, 0x00, &nop, 1);
  // if (result < 0) return NULL;

  config[0] = 0x00; // enable all axes

  // start burst (0x10) and zyxt (0x0F) or zyx0 (0xE - no temp)
  result = comm->write(comm->config, 0x1E, config, 0); // burst mode
  if (result < 0) return NULL;
  // result = comm->read(comm->config, 0x00, &nop, 1);

  // if (!exitMode())
  //   return false;
  // uint8_t tx[1] = {MLX90393_REG_EX};
  // return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);

  // arg = MLX90393_REG_EX;
  // result = comm->write(comm->config, REG_CMD, &arg, 1);
  // if (result < 0) return NULL;

  // if (!reset())
  //   return false;
  // uint8_t tx[1] = {MLX90393_REG_RT};
  // if (transceive(tx, sizeof(tx), NULL, 0, 5) != MLX90393_STATUS_RESET)

  // /* Set gain and sensor config. */
  // if (!setGain(MLX90393_GAIN_1X)) {
  //   return false;
  // }
  // uint16_t data;
  // readRegister(MLX90393_CONF1, &data);
  // // mask off gain bits
  // data &= ~0x0070;
  // // set gain bits
  // data |= gain << MLX90393_GAIN_SHIFT;
  // writeRegister(MLX90393_CONF1, data);

  // arg = MLX90393_REG_EX;
  // result = comm->write(comm->config, REG_CMD, &arg, 1);
  // if (result < 0) return NULL;

  /* Set resolution. */
  // if (!setResolution(MLX90393_X, MLX90393_RES_16))
  //   return false;
  // if (!setResolution(MLX90393_Y, MLX90393_RES_16))
  //   return false;
  // if (!setResolution(MLX90393_Z, MLX90393_RES_16))
  //   return false;

  // /* Set oversampling. */
  // if (!setOversampling(MLX90393_OSR_3))
  //   return false;

  // /* Set digital filtering. */
  // if (!setFilter(MLX90393_FILTER_7))
  //   return false;

  // /* set INT pin to output interrupt */
  // if (!setTrigInt(false)) {
  //   return false;
  // }

  return hw;
}

mlx90393_io_t *mlx90393_spi_init(uint8_t port, pin_t cs) {
  return mlx90393_init(SPI_INTERFACE, port, cs);
}

vec3f_t mlx90393_read(mlx90393_io_t *hw) {
  comm_interface_t *comm = hw->comm;
  int result;
  int16_t xi, yi, zi;
  vec3f_t ret;
  uint8_t arg = 0x00;
  hw->ok      = false;
  // uint8_t tx[1] = {MLX90393_REG_RM | MLX90393_AXIS_ALL};
  // uint8_t rx[6] = {0};

  // result = comm->write(comm->config, MLX90393_REG_RM | MLX90393_AXIS_ALL, &arg, 1);
  // if (result < 0) return ret;

  // /* Read a single data sample. */
  // if (transceive(tx, sizeof(tx), rx, sizeof(rx), 0) != MLX90393_STATUS_OK) {
  //   return false;
  // }

  result = comm->read(comm->config, MLX90393_REG_RM | MLX90393_AXIS_ALL, hw->buffer, MLX90393_BUFFER_SIZE);
  // printf(">> mlx_90393_read: %d\n", result);
  if (result < 0) return ret;

  /* Convert data to uT and float. */
  xi = (int16_t)(((uint16_t)hw->buffer[0] << 8) | hw->buffer[1]);
  yi = (int16_t)(((uint16_t)hw->buffer[2] << 8) | hw->buffer[3]);
  zi = (int16_t)(((uint16_t)hw->buffer[4] << 8) | hw->buffer[5]);

  // if (_res_x == MLX90393_RES_18)
  //   xi -= 0x8000;
  // if (_res_x == MLX90393_RES_19)
  //   xi -= 0x4000;
  // if (_res_y == MLX90393_RES_18)
  //   yi -= 0x8000;
  // if (_res_y == MLX90393_RES_19)
  //   yi -= 0x4000;
  // if (_res_z == MLX90393_RES_18)
  //   zi -= 0x8000;
  // if (_res_z == MLX90393_RES_19)
  //   zi -= 0x4000;

  int _gain = hw->gain;
  int _res  = hw->res;

  ret.x = (float)xi * mlx90393_lsb_lookup[0][_gain][_res][0];
  ret.y = (float)yi * mlx90393_lsb_lookup[0][_gain][_res][0];
  ret.z = (float)zi * mlx90393_lsb_lookup[0][_gain][_res][1];

  hw->ok = true;

  return ret;
}

// /**
//  * Gets the current sensor resolution.
//  * @param axis  The axis to get.
//  * @return An enum containing the current resolution.
//  */
// enum mlx90393_resolution
// Adafruit_MLX90393::getResolution(enum mlx90393_axis axis) {
//   switch (axis) {
//   case MLX90393_X:
//     return _res_x;
//   case MLX90393_Y:
//     return _res_y;
//   case MLX90393_Z:
//     return _res_z;
//   }
//   // shouldn't get here, but to make compiler happy...
//   return _res_x;
// }

// /**
//  * Sets the digital filter.
//  * @param filter The digital filter setting.
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::setFilter(enum mlx90393_filter filter) {
//   _dig_filt = filter;

//   uint16_t data;
//   readRegister(MLX90393_CONF3, &data);

//   data &= ~0x1C;
//   data |= filter << 2;

//   return writeRegister(MLX90393_CONF3, data);
// }

// /**
//  * Gets the current digital filter setting.
//  * @return An enum containing the current digital filter setting.
//  */
// enum mlx90393_filter Adafruit_MLX90393::getFilter(void) { return _dig_filt; }

// /**
//  * Sets the oversampling.
//  * @param oversampling The oversampling value to use.
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::setOversampling(
//     enum mlx90393_oversampling oversampling) {
//   _osr = oversampling;

//   uint16_t data;
//   readRegister(MLX90393_CONF3, &data);

//   data &= ~0x03;
//   data |= oversampling;

//   return writeRegister(MLX90393_CONF3, data);
// }

// /**
//  * Gets the current oversampling setting.
//  * @return An enum containing the current oversampling setting.
//  */
// enum mlx90393_oversampling Adafruit_MLX90393::getOversampling(void) {
//   return _osr;
// }

// /**
//  * Sets the TRIG_INT pin to the specified function.
//  *
//  * @param state  'true/1' sets the pin to INT, 'false/0' to TRIG.
//  *
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::setTrigInt(bool state) {
//   uint16_t data;
//   readRegister(MLX90393_CONF2, &data);

//   // mask off trigint bit
//   data &= ~0x8000;

//   // set trigint bit if desired
//   if (state) {
//     /* Set the INT, highest bit */
//     data |= 0x8000;
//   }

//   return writeRegister(MLX90393_CONF2, data);
// }

// /**
//  * Begin a single measurement on all axes
//  *
//  * @return True on command success
//  */
// bool Adafruit_MLX90393::startSingleMeasurement(void) {
//   uint8_t tx[1] = {MLX90393_REG_SM | MLX90393_AXIS_ALL};

//   /* Set the device to single measurement mode */
//   uint8_t stat = transceive(tx, sizeof(tx), NULL, 0, 0);
//   if ((stat == MLX90393_STATUS_OK) || (stat == MLX90393_STATUS_SMMODE)) {
//     return true;
//   }
//   return false;
// }

// /**
//  * Performs a single X/Y/Z conversion and returns the results.
//  *
//  * @param x     Pointer to where the 'x' value should be stored.
//  * @param y     Pointer to where the 'y' value should be stored.
//  * @param z     Pointer to where the 'z' value should be stored.
//  *
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::readData(float *x, float *y, float *z) {
//   if (!startSingleMeasurement())
//     return false;
//   // See MLX90393 Getting Started Guide for fancy formula
//   // tconv = f(OSR, DIG_FILT, OSR2, ZYXT)
//   // For now, using Table 18 from datasheet
//   // Without +10ms delay measurement doesn't always seem to work
//   delay(mlx90393_tconv[_dig_filt][_osr] + 10);
//   return readMeasurement(x, y, z);
// }

// bool Adafruit_MLX90393::writeRegister(uint8_t reg, uint16_t data) {
//   uint8_t tx[4] = {
//       MLX90393_REG_WR,
//       (uint8_t)(data >> 8),   // high byte
//       (uint8_t)(data & 0xFF), // low byte
//       (uint8_t)(reg << 2)};   // the register itself, shift up by 2 bits!

//   /* Perform the transaction. */
//   return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
// }

// bool Adafruit_MLX90393::readRegister(uint8_t reg, uint16_t *data) {
//   uint8_t tx[2] = {
//       MLX90393_REG_RR,
//       (uint8_t)(reg << 2)}; // the register itself, shift up by 2 bits!

//   uint8_t rx[2];

//   /* Perform the transaction. */
//   if (transceive(tx, sizeof(tx), rx, sizeof(rx), 0) != MLX90393_STATUS_OK) {
//     return false;
//   }

//   *data = ((uint16_t)rx[0] << 8) | rx[1];

//   return true;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
//     @param  event Pointer to an Adafruit Unified sensor_event_t object that
//    we'll fill in
//     @returns True on successful read
// */
// /**************************************************************************/
// bool Adafruit_MLX90393::getEvent(sensors_event_t *event) {
//   /* Clear the event */
//   memset(event, 0, sizeof(sensors_event_t));

//   event->version = 1;
//   event->sensor_id = _sensorID;
//   event->type = SENSOR_TYPE_MAGNETIC_FIELD;
//   event->timestamp = millis();

//   return readData(&event->magnetic.x, &event->magnetic.y, &event->magnetic.z);
// }

// /**
//  * Performs a full read/write transaction with the sensor.
//  *
//  * @param txbuf     Pointer the the buffer containing the data to write.
//  * @param txlen     The number of bytes to write.
//  * @param rxbuf     Pointer to an appropriately large buffer where data read
//  *                  back will be written.
//  * @param rxlen     The number of bytes to read back (not including the
//  *                  mandatory status byte that is always returned).
//  *
//  * @return The status byte from the IC.
//  */
// uint8_t Adafruit_MLX90393::transceive(uint8_t *txbuf, uint8_t txlen,
//                                       uint8_t *rxbuf, uint8_t rxlen,
//                                       uint8_t interdelay) {
//   uint8_t status = 0;
//   uint8_t i;
//   uint8_t rxbuf2[rxlen + 2];

//   if (i2c_dev) {
//     /* Write stage */
//     if (!i2c_dev->write(txbuf, txlen)) {
//       return MLX90393_STATUS_ERROR;
//     }
//     delay(interdelay);

//     /* Read status byte plus any others */
//     if (!i2c_dev->read(rxbuf2, rxlen + 1)) {
//       return MLX90393_STATUS_ERROR;
//     }
//     status = rxbuf2[0];
//     for (i = 0; i < rxlen; i++) {
//       rxbuf[i] = rxbuf2[i + 1];
//     }
//   }

//   if (spi_dev) {
//     spi_dev->write_then_read(txbuf, txlen, rxbuf2, rxlen + 1, 0x00);
//     status = rxbuf2[0];
//     for (i = 0; i < rxlen; i++) {
//       rxbuf[i] = rxbuf2[i + 1];
//     }
//     delay(interdelay);
//   }

//   /* Mask out bytes available in the status response. */
//   return (status >> 2);
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the sensor_t device data, Adafruit Unified Sensor format
//     @param  sensor Pointer to an Adafruit Unified sensor_t object that we'll
//    fill in
// */
// /**************************************************************************/
// void Adafruit_MLX90393::getSensor(sensor_t *sensor) {
//   /* Clear the sensor_t object */
//   memset(sensor, 0, sizeof(sensor_t));

//   /* Insert the sensor name in the fixed length char array */
//   strncpy(sensor->name, "MLX90393", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name) - 1] = 0;
//   sensor->version = 1;
//   sensor->sensor_id = _sensorID;
//   sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
//   sensor->min_delay = 0;
//   sensor->min_value = -50000; // -50 gauss in uTesla
//   sensor->max_value = 50000;  // +50 gauss in uTesla
//   sensor->resolution = 0.15;  // 100/16-bit uTesla per LSB
// }

// /*!
//  *    @brief  Sets up the hardware and initializes hardware SPI
//  *    @param  cs_pin The arduino pin # connected to chip select
//  *    @param  theSPI The SPI object to be used for SPI connections.
//  *    @return True if initialization was successful, otherwise false.
//  */
// boolean Adafruit_MLX90393::begin_SPI(uint8_t cs_pin, SPIClass *theSPI) {
//   i2c_dev = NULL;
//   if (!spi_dev) {
//     _cspin = cs_pin;
//     spi_dev = new Adafruit_SPIDevice(cs_pin,
//                                      1000000,               // frequency
//                                      SPI_BITORDER_MSBFIRST, // bit order
//                                      SPI_MODE3,             // data mode
//                                      theSPI);
//   }
//   if (!spi_dev->begin()) {
//     return false;
//   }
//   return _init();
// }

// bool Adafruit_MLX90393::_init(void) {

//   if (!exitMode())
//     return false;

//   if (!reset())
//     return false;

//   /* Set gain and sensor config. */
//   if (!setGain(MLX90393_GAIN_1X)) {
//     return false;
//   }

//   /* Set resolution. */
//   if (!setResolution(MLX90393_X, MLX90393_RES_16))
//     return false;
//   if (!setResolution(MLX90393_Y, MLX90393_RES_16))
//     return false;
//   if (!setResolution(MLX90393_Z, MLX90393_RES_16))
//     return false;

//   /* Set oversampling. */
//   if (!setOversampling(MLX90393_OSR_3))
//     return false;

//   /* Set digital filtering. */
//   if (!setFilter(MLX90393_FILTER_7))
//     return false;

//   /* set INT pin to output interrupt */
//   if (!setTrigInt(false)) {
//     return false;
//   }

//   return true;
// }

// /**
//  * Perform a mode exit
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::exitMode(void) {
//   uint8_t tx[1] = {MLX90393_REG_EX};

//   /* Perform the transaction. */
//   return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
// }

// /**
//  * Perform a soft reset
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::reset(void) {
//   uint8_t tx[1] = {MLX90393_REG_RT};

//   /* Perform the transaction. */
//   if (transceive(tx, sizeof(tx), NULL, 0, 5) != MLX90393_STATUS_RESET) {
//     return false;
//   }
//   return true;
// }

// /**
//  * Sets the sensor gain to the specified level.
//  * @param gain  The gain level to set.
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::setGain(mlx90393_gain_t gain) {
//   _gain = gain;

//   uint16_t data;
//   readRegister(MLX90393_CONF1, &data);

//   // mask off gain bits
//   data &= ~0x0070;
//   // set gain bits
//   data |= gain << MLX90393_GAIN_SHIFT;

//   return writeRegister(MLX90393_CONF1, data);
// }

// /**
//  * Gets the current sensor gain.
//  *
//  * @return An enum containing the current gain level.
//  */
// mlx90393_gain_t Adafruit_MLX90393::getGain(void) {
//   uint16_t data;
//   readRegister(MLX90393_CONF1, &data);

//   // mask off gain bits
//   data &= 0x0070;

//   return (mlx90393_gain_t)(data >> 4);
// }

// /**
//  * Sets the sensor resolution to the specified level.
//  * @param axis  The axis to set.
//  * @param resolution  The resolution level to set.
//  * @return True if the operation succeeded, otherwise false.
//  */
// bool Adafruit_MLX90393::setResolution(enum mlx90393_axis axis,
//                                       enum mlx90393_resolution resolution) {

//   uint16_t data;
//   readRegister(MLX90393_CONF3, &data);

//   switch (axis) {
//   case MLX90393_X:
//     _res_x = resolution;
//     data &= ~0x0060;
//     data |= resolution << 5;
//     break;
//   case MLX90393_Y:
//     _res_y = resolution;
//     data &= ~0x0180;
//     data |= resolution << 7;
//     break;
//   case MLX90393_Z:
//     _res_z = resolution;
//     data &= ~0x0600;
//     data |= resolution << 9;
//     break;
//   }

//   return writeRegister(MLX90393_CONF3, data);
// }
