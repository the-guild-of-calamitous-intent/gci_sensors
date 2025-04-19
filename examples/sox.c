#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "lsm6dsox.h"
#include <picolibc.h>

constexpr pin_t i2c_scl = 1;
constexpr pin_t i2c_sda = 0;

int main() {
  stdio_init_all();
  wait_for_usb();

  int32_t speed = gci_i2c0_bus_init(I2C_400KHZ, i2c_sda, i2c_scl);

  printf(">> i2c instance: %u baud: %u\n", 0, speed);
  printf(">> i2c SDA: %u SCL: %u\n", i2c_sda, i2c_scl);
  bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C)); // compile info

  printf("/// Accel/Gyros START ///\n");

  lsm6dsox_i2c_t *imu = NULL;
  while (true) {
    imu = lsm6dsox_i2c_init(0, LSM6DSOX_ADDRESS, ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (imu != NULL) break;
    printf("imu error\n");
    sleep_ms(1000);
  }

  while (true) {
    lsm6dsox_t i = lsm6dsox_read(imu);
    if (i.ok == false) continue;

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    printf("Timestamp: %llu msec\n", i.timestamp_us);
  }
}