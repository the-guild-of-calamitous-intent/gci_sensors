#include <gci_sensors/lis2mdl.h>
#include <hardware/spi.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <tusb.h> // wait usb serial

#define SCL  17
#define SDA  16
#define PORT 0
#define SCK  2
#define TX   3 // SDO
#define RX   4 // SDI
#define CS   10

volatile bool mag_ready = false;
lis2mdl_io_t *mag = NULL;

int main() {
  stdio_init_all();
  while (!tud_cdc_connected()) sleep_ms(100);

#if 1
  uint32_t speed = gcis_spi0_init(GCIS_SPI_1MHZ, RX, TX, SCK);
  printf(">> SPI%u baudrate: %u\n", PORT, speed);
  printf(">> SDI: %u SDO: %u SCK: %u CS: %u\n", TX, RX, SCK, CS);
  bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

  mag = lis2mdl_create(SPI_INTERFACE, PORT, CS);
  bi_decl(bi_1pin_with_name(CS, "MAG CS"));
#else
  uint32_t speed = gcis_i2c_bus_init(PORT, GCIS_I2C_100KHZ, SDA, SCL);
  printf(">> I2C%u at %u bps\n", PORT, speed);
  printf(">> SDA: %u SCL: %u\n", SDA, SCL);
  bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C)); // compile info

  mag = lis2mdl_create(I2C_INTERFACE, PORT, LIS2MDL_ADDRESS);
#endif

  mag->odr = LIS2MDL_ODR_100;

  while (lis2mdl_init(mag) < 0) {
    printf("*** mag init error ***\n");
    sleep_ms(1000);
  }

  while (true) {
    sleep_ms(1000);

    vec3f_t m;
    if (lis2mdl_read(mag, &m) < 0) {
      printf("*** failed mag read ***\n");
      continue;
    }
    printf("LIS2MDL (mGauss): X=%.2f, Y=%.2f, Z=%.2f\n", m.x, m.y, m.z);
  }

  return 0;
}