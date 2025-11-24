# SMI330 Linux IIO Driver

[![License](https://img.shields.io/badge/license-GPL%2FBSD-blue)](LICENSE)
[![Kernel](https://img.shields.io/badge/kernel-6.6%2B-orange)](https://kernel.org/)

A comprehensive Linux Industrial I/O (IIO) driver for the Bosch SMI330 6-axis Inertial Measurement Unit (IMU).

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)  
- [Hardware Specifications](#hardware-specifications)
- [Supported Platforms](#supported-platforms)
- [Building and Installation](#building-and-installation)
- [Device Tree Configuration](#device-tree-configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)
- [Architecture](#architecture)
- [License](#license)

## Introduction

The SMI330 is a highly integrated, low-power inertial measurement unit (IMU) that combines precise acceleration and angular rate (gyroscopic) measurements with intelligent on-chip motion-triggered interrupt features. This Linux IIO driver provides a standardized interface for accessing all sensor capabilities through the Industrial I/O subsystem.

## Features

### Core Functionality
- **3-axis accelerometer** with configurable ranges: ±2g, ±4g, ±8g, ±16g
- **3-axis gyroscope** with configurable ranges: ±125°/s, ±250°/s, ±500°/s
- **Temperature sensor** with operating range: -40°C to +125°C
- **Hardware FIFO** with configurable watermark levels
- **Multiple communication interfaces**: I2C and SPI

### Advanced Features
- Motion detection and no-motion detection
- Tilt detection
- Self-calibration (Component Re-Trim)
- Self-test functionality
- Soft reset capability
- Auto-operation mode with power management
- Configurable interrupt pins (INT1, INT2)
- Open-drain and push-pull interrupt configuration
- Wake-up source support for system suspend/resume

### Driver Features  
- Standard Linux IIO interface
- Sysfs attribute support
- Device tree integration
- Power management support with wake-up capability
- Multi-instance support
- Comprehensive error handling

## Hardware Specifications

| Parameter | Specification |
|-----------|---------------|
| Supply Voltage | 1.71V - 3.6V |
| Interface | I2C (up to 400kHz) / SPI (up to 10MHz) |
| Resolution | 16-bit ADC |
| Operating Temperature | -40°C to +125°C |
| Current Consumption | Low power modes available |

## Supported Platforms

### Tested Kernels
- Linux 6.6.y (Raspberry Pi)
- Linux 6.12.y (Raspberry Pi)

## Integration Guide

> 1. Clone: `git clone https://github.com/boschmemssolutions/SMI330-Linux-Driver-IIO.git`
> 2. Copy: `cp -r SMI330-Linux-Driver-IIO/drivers/iio/imu/smi330 linux/drivers/iio/imu/`
> 3. Kconfig: add `source "drivers/iio/imu/smi330/Kconfig"`
> 4. Makefile: add `obj-$(CONFIG_SMI330) += smi330/` (plus I2C/SPI objects)
> 5. Enable: `CONFIG_SMI330[,_I2C,_SPI]` + select interrupt mapping
> 6. Add DT node / overlay (`compatible = "bosch,smi330"`)
> 7. Build + deploy: `make Image modules dtbs` then install & boot; verify via `dmesg | grep smi330` & `ls /sys/bus/iio/devices`

See [Integration Guide](doc/IntegrationGuide.md) for a more comprehensive documentation.

## Documentation

- [Website](https://www.bosch-semiconductors.com/products/mems-sensors/adas/smi330/)

## Architecture

```
                    User Space Applications
                           |
    ┌─────────────────────────────────────────────────────┐
    │                 IIO Subsystem                       │
    ├─────────────────────────────────────────────────────┤
    │  sysfs interface  │  chardev  │  buffer interface   │
    └─────────────────────────────────────────────────────┘
                           |
    ┌─────────────────────────────────────────────────────┐
    │               SMI330 IIO Driver                     │
    │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐    │
    │  │ smi330_core │ │ smi330_i2c  │ │ smi330_spi  │    │
    │  └─────────────┘ └─────────────┘ └─────────────┘    │
    └─────────────────────────────────────────────────────┘
                           |
    ┌─────────────────────────────────────────────────────┐
    │           Hardware Abstraction Layer                │
    │        I2C Subsystem    │    SPI Subsystem          │
    └─────────────────────────────────────────────────────┘
                           |
                    SMI330 Hardware
```

## License

See [LICENSE](drivers/iio/imu/smi330/LICENSE.md) for detailed terms.
