

## LIS3MDL

```c
busWriteRegister(dev, LIS3MDL_REG_CTRL_REG2, LIS3MDL_FS_4GAUSS);
busWriteRegister(dev, LIS3MDL_REG_CTRL_REG1, LIS3MDL_TEMP_EN | LIS3MDL_OM_ULTRA_HI_PROF | LIS3MDL_DO_80);
busWriteRegister(dev, LIS3MDL_REG_CTRL_REG5, LIS3MDL_BDU);
busWriteRegister(dev, LIS3MDL_REG_CTRL_REG4, LIS3MDL_ZOM_UHP);
busWriteRegister(dev, LIS3MDL_REG_CTRL_REG3, 0x00);
```

## QMC5883L

```c
busWriteRegister(dev, 0x0B, 0x01);
busWriteRegister(dev, QMC5883L_REG_CONF1, QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_OSR_512 | QMC5883L_RNG_8G);
```

## LSM6DSOX

```c
// Reset the device (wait 100ms before continuing config)
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure interrupt pin 1 for gyro data ready only
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_INT1_CTRL, LSM6DSO_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_INT2_CTRL, LSM6DSO_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    lsm6dsoWriteRegister(dev, LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR6664 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C, (LSM6DSO_VAL_CTRL3_C_BDU | LSM6DSO_VAL_CTRL3_C_H_LACTIVE | LSM6DSO_VAL_CTRL3_C_PP_OD | LSM6DSO_VAL_CTRL3_C_SIM | LSM6DSO_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // enable accelerometer high performane mode; set gyro LPF1 cutoff to 335.5hz
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL4_C, LSM6DSO_MASK_CTRL4_C, (LSM6DSO_VAL_CTRL4_C_I2C_DISABLE | LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // disable I2C interface; enable gyro LPF1
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL6_C, LSM6DSO_MASK_CTRL6_C, (LSM6DSO_VAL_CTRL6_C_XL_HM_MODE | LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ), 1);

    // Configure control register 9
    // disable I3C interface
    lsm6dsoWriteRegisterBits(dev, LSM6DSO_REG_CTRL9_XL, LSM6DSO_MASK_CTRL9_XL, LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE, 1);
```