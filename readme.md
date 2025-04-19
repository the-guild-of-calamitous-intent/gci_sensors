# GCI Sensors

> Another re-write of my code base, but this time from
> C++ to C23. I am using C23 because it brings in some
> nice things like `constexpr`

## Sensors

- Accel/Gyro
  - LSM6DSOX
- Magnetometer
  - LIS3MDL
- GPS
  - PA1010D
- Pressure/Temperature
  - BMP390

All sensors basically have these funtions available to them:

```c
sensor_i2c_t* sensor_i2c_init(sensor_i2c_t*, uint32_t address, ...)
sensor_t sensor_i2c_read(sensor_i2c_t*)
bool sensor_reboot(sensor_i2c_t*)
bool sensor_ready(sensor_i2c_t*)
int32_t sensor_available(sensor_i2c_t*)
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