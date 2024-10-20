# SMI330 Sensor Linux IIO Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)

## Introduction <a name=Intro></a>

The SMI330 is a highly integrated, low power inertial measurement unit (IMU) that combines precise acceleration and
angular rate (gyroscopic) measurement with intelligent on-chip motion-triggered interrupt features. The SMI330 integrates
a 16 bit digital, triaxial accelerometer with range configurable to +/- 2 g, +/- 4 g, +/- 8 g, +/- 16 g,
a 16 bit digital, triaxial gyroscope with range configurable to +/- 125 DPS, +/- 250 DPS , +/- 500 DPS,
a 16 bit digital temperature sensor for an operating temperature range -40 C ... +125 C

## Documentation <a name=Doc></a>

https://boschmemssolutions.github.io/iio/bosch_smi330_IIO.html

## License <a name=License></a>

See [LICENSE](drivers/iio/imu/smi330/LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* I2C
* SPI

## Architecture <a name=Architecture></a>

```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
                IMU-subsystem
                      |
                iio-subsystem
                      |
sensor_API <-- smi330_driver --> smi330_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```
