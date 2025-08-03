#include "gci_sensors/lis3mdl.h"
#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

#define SCL      17
#define SDA      16
#define I2C_PORT 0

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  int32_t speed = gcis_i2c_bus_init(I2C_PORT, GCIS_I2C_400KHZ, SDA, SCL);
  if (speed < 0) {
    printf("*** couldn't initialize i2c%d\n ***", I2C_PORT);
    sleep_ms(1000);
  }

  printf(">> i2c%u at %u bps\n", I2C_PORT, speed);
  printf(">> i2c SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  printf("/// I2C ACCEL/GYRO START ///\n");

  lsm6dsox_io_t *imu = NULL;
  while (true) {
    imu = lsm6dsox_i2c_init(
        I2C_PORT, LSM6DSOX_ADDRESS,
        LSM6DSOX_XL_4_G, LSM6DSOX_G_2000_DPS,
        LSM6DSOX_ODR_1660_HZ);
    if (imu != NULL && imu->ok) break;
    printf("imu error\n");
    sleep_ms(1000);
  }

  // lsm6dsox_dump(imu);
  // while(1) sleep_ms(1000);

  uint64_t prev = 0;

  while (true) {
    uint64_t now = time_us_64();
    lsm6dsox_t i = lsm6dsox_read(imu);
    if (imu->ok == false) {
      printf("*** Bad read ***\n");
      sleep_ms(1000);
      continue;
    }

    uint64_t delta = now - prev;

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    printf("Timestamp: %llu msec\n", now);
    printf("Delta: %llu usec   %llu msec   %.1fHz\n", delta, delta / 1000, 1E6 / (float)delta);

    prev = now;

    sleep_us(5);
    // sleep_ms(1);
  }

  gcis_i2c_free(I2C_PORT);

  return 0;
}

// odr 1660 hz

// Temperature: 26.082031 C
// Timestamp: 15261973 msec
// Delta: 1349 usec   1 msec   741.3Hz
// -----------------------------
// Accels: -0.011841 0.026123 1.038330 g
// Gyros: 0.061035 -0.976562 -0.122070 dps
// Temperature: 26.082031 C
// Timestamp: 15263292 msec
// Delta: 1319 usec   1 msec   758.2Hz
// -----------------------------
// Accels: -0.014893 0.033569 1.038818 g
// Gyros: 0.183105 -1.220703 -0.244141 dps
// Temperature: 26.097656 C
// Timestamp: 15264606 msec
// Delta: 1314 usec   1 msec   761.0Hz
// -----------------------------
// Accels: -0.010986 0.034912 1.022705 g
// Gyros: 0.244141 -1.342773 -0.366211 dps
// Temperature: 26.097656 C
// Timestamp: 15265896 msec
// Delta: 1290 usec   1 msec   775.2Hz
// -----------------------------
// Accels: -0.001465 0.032471 1.016724 g
// Gyros: 0.122070 -1.525879 -0.244141 dps
// Temperature: 26.097656 C
// Timestamp: 15267150 msec
// Delta: 1254 usec   1 msec   797.4Hz
// -----------------------------
// Accels: -0.005615 0.031128 1.021240 g
// Gyros: 0.061035 -1.220703 -0.244141 dps
// Temperature: 26.097656 C
// Timestamp: 15268448 msec
// Delta: 1298 usec   1 msec   770.4Hz
// -----------------------------
// Accels: -0.007568 0.035278 1.029663 g
// Gyros: 0.000000 -0.976562 -0.427246 dps
// Temperature: 26.097656 C
// Timestamp: 15269781 msec
// Delta: 1333 usec   1 msec   750.2Hz
// -----------------------------
// Accels: -0.016357 0.033203 1.034790 g
// Gyros: -0.061035 -1.098633 -0.305176 dps
// Temperature: 26.097656 C
// Timestamp: 15271093 msec
// Delta: 1312 usec   1 msec   762.2Hz
// -----------------------------
// Accels: -0.011597 0.031372 1.033813 g
// Gyros: -0.061035 -1.281738 -0.305176 dps
// Temperature: 26.097656 C
// Timestamp: 15272379 msec
// Delta: 1286 usec   1 msec   777.6Hz
// -----------------------------
// Accels: -0.009766 0.032104 1.026001 g
// Gyros: 0.000000 -1.586914 -0.305176 dps
// Temperature: 26.097656 C
// Timestamp: 15273610 msec
// Delta: 1231 usec   1 msec   812.3Hz

// -----------------------------
// Accels: -0.008667 0.029907 1.004150 g
// Gyros: 0.122070 -1.220703 -0.305176 dps
// Temperature: 26.152344 C
// Timestamp: 10550313 msec
// Delta: 1904 usec   1 msec   525.2Hz
// -----------------------------
// Accels: -0.007446 0.037476 1.000854 g
// Gyros: 0.122070 -1.342773 -0.244141 dps
// Temperature: 26.152344 C
// Timestamp: 10552226 msec
// Delta: 1913 usec   1 msec   522.7Hz
// -----------------------------
// Accels: -0.005981 0.023804 1.025269 g
// Gyros: -0.061035 -1.098633 -0.366211 dps
// Temperature: 26.152344 C
// Timestamp: 10554129 msec
// Delta: 1903 usec   1 msec   525.5Hz
// -----------------------------
// Accels: -0.015381 0.034790 1.036743 g
// Gyros: 0.122070 -0.793457 -0.183105 dps
// Temperature: 26.152344 C
// Timestamp: 10556029 msec
// Delta: 1900 usec   1 msec   526.3Hz
// -----------------------------
// Accels: -0.014648 0.035522 1.025513 g
// Gyros: 0.061035 -1.220703 -0.305176 dps
// Temperature: 26.152344 C
// Timestamp: 10557991 msec
// Delta: 1962 usec   1 msec   509.7Hz
// -----------------------------
// Accels: -0.000732 0.026123 1.025391 g
// Gyros: 0.000000 -1.708984 -0.244141 dps
// Temperature: 26.152344 C
// Timestamp: 10559844 msec
// Delta: 1853 usec   1 msec   539.7Hz
// -----------------------------
// Accels: -0.010986 0.033691 1.025513 g
// Gyros: 0.244141 -1.098633 -0.488281 dps
// Temperature: 26.152344 C
// Timestamp: 10561701 msec
// Delta: 1857 usec   1 msec   538.5Hz
// -----------------------------
// Accels: -0.015991 0.035034 1.031738 g
// Gyros: -0.061035 -0.976562 -0.366211 dps
// Temperature: 26.152344 C
// Timestamp: 10563633 msec
// Delta: 1932 usec   1 msec   517.6Hz
// -----------------------------
// Accels: -0.003662 0.023071 1.027954 g
// Gyros: -0.244141 -1.586914 -0.366211 dps
// Temperature: 26.160156 C
// Timestamp: 10565525 msec
// Delta: 1892 usec   1 msec   528.5Hz
// -----------------------------
// Accels: -0.008667 0.0294

// Temperature: 26.031250 C
// Timestamp: 22691286 msec
// Delta: 1286 usec   1 msec   777.6Hz
// -----------------------------
// Accels: -0.004395 0.032471 1.028931 g
// Gyros: -0.061035 -1.342773 -0.366211 dps
// Temperature: 26.031250 C
// Timestamp: 22692610 msec
// Delta: 1324 usec   1 msec   755.3Hz
// -----------------------------
// Accels: -0.001831 0.033813 1.025269 g
// Gyros: 0.000000 -1.464844 -0.244141 dps
// Temperature: 26.031250 C
// Timestamp: 22693927 msec
// Delta: 1317 usec   1 msec   759.3Hz
// -----------------------------
// Accels: -0.006592 0.036255 1.024414 g
// Gyros: -0.061035 -1.342773 -0.427246 dps
// Temperature: 26.031250 C
// Timestamp: 22695156 msec
// Delta: 1229 usec   1 msec   813.7Hz
// -----------------------------
// Accels: -0.005981 0.032837 1.025757 g
// Gyros: 0.000000 -1.342773 -0.244141 dps
// Temperature: 26.031250 C
// Timestamp: 22696460 msec
// Delta: 1304 usec   1 msec   766.9Hz
// -----------------------------
// Accels: -0.008789 0.030518 1.025024 g
// Gyros: 0.000000 -1.220703 -0.183105 dps
// Temperature: 26.031250 C
// Timestamp: 22697794 msec
// Delta: 1334 usec   1 msec   749.6Hz
// -----------------------------
// Accels: -0.004272 0.031006 1.020020 g
// Gyros: 0.061035 -1.281738 -0.244141 dps
// Temperature: 26.031250 C
// Timestamp: 22699107 msec
// Delta: 1313 usec   1 msec   761.6Hz
// -----------------------------
// Accels: -0.010498 0.031982 1.012695 g
// Gyros: 0.061035 -1.342773 -0.183105 dps
// Temperature: 26.031250 C
// Timestamp: 22700317 msec
// Delta: 1210 usec   1 msec   826.4Hz
// -----------------------------
// Accels: -0.007080 0.033813 1.012451 g
// Gyros: 0.061035 -1.403809 -0.244141 dps
// Temperature: 26.031250 C
// Timestamp: 22701577 msec
// Delta: 1260 usec   1 msec   793.7Hz
// -----------------------------
// Accels: -0.002930 0.030640 1.015015 g
// Gyros: 0.000000 -1.220703 -0.305176 dps
// Temperature: 26.031250 C
// Timestamp: 22702910 msec
// Delta: 1333 usec   1 msec   750.2Hz
// -----------------------------
// Accels: -0.006226 0.033691 1.021851 g
// Gyros: 0.122070 -1.159668 -0.305176 dps
// Temperature: 26.031250 C
// Timestamp: 22704155 msec
// Delta: 1245 usec