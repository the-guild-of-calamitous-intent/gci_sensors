# LSM6DSOX or LSM6DSO

> NOTE: the SOX and SO are the same except the 
> SOX varient has some AIML stuff it can do.

Assume Mode 1: sensor -> I2C/SPI -> uC

REG        | DEFAULT
-----------|---------
FUNC_CFG   | 0x00
PIN_CTRL   | 0x3F
-----------|---------
FIFO_CTRL1 | 0x00
FIFO_CTRL2 | 0x00
FIFO_CTRL3 | 0x00
FIFO_CTRL4 | 0x00
BDR_REG1   | 0x00
BDR_REG2   | 0x00
INT1_CTRL  | 0x00
INT2_CTRL  | 0x00
-----------|---------
WHO_AM_I   | 0x6C
-----------|---------
CTRL1_XL   | 0x00
CTRL2_G    | 0x00
CTRL3_C    | 0x04
CTRL4_C    | 0x00
CTRL5_C    | 0x00
CTRL6_C    | 0x00
CTRL7_G    | 0x00
CTRL8_XL   | 0x00
CTRL9_XL   | 0xD0
CTRL10_C   | 0x00

GYRO Filtering (Fig 19)
You can bypass HPF and LPF1, but not LPF2
You cannot set LPF2, see table 18

LPF2 ODR [Hz] | Cutoff [Hz]
--------------|--------------
104           |   33
208           |   66.8
416           |   135.9
833           |   295.5
1660          |   1108.1

```
            |HP_EN_G|            |LPF1_SEL_G|
GYRO ADC -+->|  0    | --------+->|    0     | ---------+-> LPF2 -> SPI/I2C
          |  |       |         |  |          |          |
          +->|  1    | -> HPF -+  |    1     | -> LPF1 -+
```

ACCEL Filtering (Fig 16 & 17)
The ACCEL has an analog anti-alias LPF (AALPF) right before the ADC
and then goes into a digital LPF1 controled by ODR_XL and LPF2 is
controlled by HPCF_XL

```
                        |LPF2_XL_EN|
AALPF -> ADC -> LPF1 -+->|    0     | ---------+-> SPI/I2C
                      |  |          |          |
                      +->|    1     | -> LPF2 -+
```