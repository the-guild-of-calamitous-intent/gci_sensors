#include "gci_sensors/lis3mdl.h"
#include "gci_sensors/lsm6dsox.h"
#include <hardware/gpio.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h> // wait for USB

constexpr pin_t RX             = 0;
constexpr pin_t CS             = 1;
constexpr pin_t SCK            = 2;
constexpr pin_t TX             = 3;
constexpr uint32_t BUFFER_SIZE = 10;

constexpr float M_PIf = 3.14159265358979323846f;

typedef union vector2_u {
  float v[2];
  struct {
    float x, y;
  };
} vector2_t;

typedef union vector3_u {
  float v[3];
  struct {
    float x, y, z;
  };
} vector3_t;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  vector3_t v = {1, 2, 3};
  v.v[0]      = 4.0f;

  uint32_t speed = gcis_spi_bus_init(0, 1000000, RX, TX, SCK);
  gcis_spi_init_cs(CS);

  printf(">> spi instance: %u baudrate: %u\n", 0, speed);
  printf(">> spi MISO: %u MOSI: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);

  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(CS, "SPI CS"));

  printf("/// ACCEL/GYRO START ///\n");

  lsm6dsox_io_t *imu = NULL;
  while (true) {
    imu = lsm6dsox_spi_init(
        0,
        // RX, TX, SCK,
        CS,
        ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS,
        RATE_208_HZ);
    if (imu != NULL) break;
    printf("imu error\n");
    sleep_ms(1000);
  }

  while (true) {
    lsm6dsox_t i = lsm6dsox_read(imu);
    if (imu->ok == false) {
      printf("*** Bad read ***\n");
      sleep_ms(1000);
      continue;
    }

    printf("-----------------------------\n");
    printf("Accels: %f %f %f g\n", i.a.x, i.a.y, i.a.z);
    printf("Gyros: %f %f %f dps\n", i.g.x, i.g.y, i.g.z);
    printf("Temperature: %f C\n", i.temperature);
    // printf("Timestamp: %llu msec\n", i.timestamp_us);

    sleep_ms(500);
  }
}