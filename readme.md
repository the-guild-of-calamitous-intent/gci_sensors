# GCI Sensors

> Another re-write of my code base, but this time from
> C++ to C23. I am using C23 because it brings in some
> nice things like `constexpr` [1]

[1]: https://en.cppreference.com/w/c/language/constexpr.html

## Running

- [Pi Pico Pinout](https://pico.pinout.xyz/)
- [Pico Examples](https://github.com/raspberrypi/pico-examples)

```
flash-pico <filename>.uf2
screen /dev/tty.usbmodemxxxx # ctrl-a ctrl-d to exit
```

## Sensors

- Accel/Gyro
  - LSM6DSOX or LSM6DSO (no ML core)
- Magnetometer
  - LIS3MDL
  - QMC5883L
- GPS
  - PA1010D (I2C only)
- Pressure/Temperature
  - BMP390
  - LPS22

All sensors have a simple interface:

```c
// return: NULL - error
//         senosr_io_t - success
sensor_io_t* gcis_[spi,i2c]_init(uint8_t port, uint8_t address, ...);
sensor_t sensor_read(sensor_io_t* hw);
int32_t sensor_write(sensor_io_t* hw, uint8_t reg, uint8_t* buffer, uint8_t length);
// others as needed ...
```

## Units

| Sensor Type | Units           | Abbreviation
|-------------|-----------------|--------------|
| Accels      | gravity         | g
| Gyros       | degrees/sec     | dps
| Mags        | gauss           | gs
| Temperature | Celcius         | C
| Pressure    | Pascal          | Pa
| Altitude    | meter           | m
| Lat/Lon     | decimal degrees | deg
| Rate        | hertz           | Hz
| Time        | seconds         | sec

## Generic API

- Naming convention
  - `<sensor>_t` is the output, contains sensor readings
  - `<sensor>_io_t` is the generic information to interface
    with the sensor via I2C or SPI
    - Errors are captured in each `<sensor>_io_t`
      - `bool ok`: true-ok, false-error
      - `int errnum`: error number
  - Functions are generally named `return_type <sensor>_<action>(args)`
    - `<action>`s can be `init`, `read`, `write`, etc
    - `return_type` can be `int`, `void`, `<sensor>_t`, etc
    - `int` return is typically:
      - `0`: success
      - `>0`: success or baudrate or data read/written
      - `<0`: error

## ToDo

- [ ] Make I2C or SPI interface for all sensors
  - [ ] bmp390
  - [ ] lps22
  - [ ] lsm6dsox
  - [ ] pa1010d
  - [ ] qmc5883l
- [ ] Add a `<sensor>_setmode(...)` for sensors
- [ ] For completeness, add a `<sensor>_free(...)` for sensors
- [x] Remove external libraries
- [ ] Add linux support

# MIT License

**Copyright (c) 2022 The Guild of Calamitous Intent**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.