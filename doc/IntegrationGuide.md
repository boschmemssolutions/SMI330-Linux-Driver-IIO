# SMI330 Linux IIO Driver – Integration Guide

## Table of Contents

1. [Scope of Release](#1-scope-of-release)
   - 1.1 [Driver Source Files](#11-driver-source-files)
   - 1.2 [Documentation Files](#12-documentation-files)

2. [Device Tree Integration](#2-device-tree-integration)
    - 2.1 [Device Tree Binding Properties](#21-device-tree-binding-properties)
    - 2.2 [I2C Integration Example](#22-i2c-integration-example)
    - 2.3 [SPI Integration Example](#23-spi-integration-example)
    - 2.4 [Interrupt Configuration Guidelines](#24-interrupt-configuration-guidelines)
    - 2.5 [Alternative I2C Addresses](#25-alternative-i2c-addresses)

3. [In-Tree Integration](#3-in-tree-integration)
    - 3.1 [Prerequisites](#31-prerequisites)
    - 3.2 [Integration](#32-integration)

4. [Runtime Interface & Operation](#4-runtime-interface--operation)
    - 4.1 [Basic Sensor Channels](#41-basic-sensor-channels)
    - 4.2 [Sensor Configuration](#42-sensor-configuration)
    - 4.3 [Buffer and Streaming Operations](#43-buffer-and-streaming-operations)
    - 4.4 [Advanced Features (if interrupt pins configured)](#44-advanced-features-if-interrupt-pins-configured)

5. [Verification and Testing](#5-verification-and-testing)
    - 5.1 [Driver Loading Verification](#51-driver-loading-verification)
    - 5.2 [Functional Testing](#52-functional-testing)
    - 5.3 [Troubleshooting Common Issues](#53-troubleshooting-common-issues)

6. [Known Limitations](#6-known-limitations)

7. [Documentation and Support Resources](#7-documentation-and-support-resources)
     - 7.1 [Technical Documentation](#71-technical-documentation)
     - 7.2 [Related Driver Projects](#72-related-driver-projects)
     - 7.3 [Support Channels](#73-support-channels)
     - 7.4 [Contributing and Feedback](#74-contributing-and-feedback)

---
## 1. Scope of Release

Includes all source, binding, and documentation assets required to add SMI330 to a Linux kernel build.

### 1.1 Driver Source Files
* **Core:** `drivers/iio/imu/smi330/smi330_core.c` – IIO device logic (channels, FIFO, events, power)
* **I2C bus:** `drivers/iio/imu/smi330/smi330_i2c.c` – Regmap I2C transport
* **SPI bus:** `drivers/iio/imu/smi330/smi330_spi.c` – Regmap SPI transport  
* **Header:** `drivers/iio/imu/smi330/smi330.h` – Registers, structures
* **Kconfig:** `drivers/iio/imu/smi330/Kconfig` – Config + interrupt mapping
* **Makefile:** `drivers/iio/imu/smi330/Makefile` – Build wiring

### 1.2 Documentation Files
* **DT binding:** `dts/binding/bosch,smi330.yaml`
* **Sysfs ABI:** `Documentation/sysfs-bus-iio-smi330`
* **Licenses:** `drivers/iio/imu/smi330/LICENSE-*` (BSD / GPL)

---
## 2. Device Tree Integration

Device Tree configuration is essential for hardware detection, bus configuration, and interrupt setup. The SMI330 driver follows standard Linux device tree conventions.

### 2.1 Device Tree Binding Properties
**Required:** `compatible = "bosch,smi330"`, `reg = <addr>` (I2C 0x68/0x69, or SPI CS)

**Bus:** SPI: `spi-max-frequency`; I2C: normal bus props.

**Optional:** `interrupts`, `interrupt-names`, `vdd-supply`, `vddio-supply`, `drive-open-drain`, `wakeup-source`.

### 2.2 I2C Integration Example

```dts
&i2c1 {
    status = "okay";
    clock-frequency = <400000>;

    smi330: imu@68 {
        compatible = "bosch,smi330";
        reg = <0x68>;  /* 0x69 if SDO pin is high */
        
        /* Power supplies */
        vdd-supply = <&vdd_3v3>;
        vddio-supply = <&vdd_3v3>;
        
        /* Interrupt configuration */
        interrupt-parent = <&gpio>;
        interrupts = <26 IRQ_TYPE_EDGE_RISING>,    /* INT1 */
                     <20 IRQ_TYPE_EDGE_RISING>;    /* INT2 */
        interrupt-names = "INT1", "INT2";
        
        /* Optional features */
        wakeup-source;          /* Enable wake-up capability */
        drive-open-drain;       /* Use open-drain outputs */
    };
};
```

### 2.3 SPI Integration Example

```dts
&spi0 {
    status = "okay";
    
    smi330: imu@0 {
        compatible = "bosch,smi330";
        reg = <0>;                          /* Chip select 0 */
        spi-max-frequency = <10000000>;     /* 10 MHz maximum */
        
        /* Power supplies */
        vdd-supply = <&vdd_3v3>;
        vddio-supply = <&vdd_3v3>;
        
        /* Interrupt configuration */
        interrupt-parent = <&gpio>;
        interrupts = <26 IRQ_TYPE_EDGE_RISING>,    /* INT1 */
                     <20 IRQ_TYPE_EDGE_RISING>;    /* INT2 */
        interrupt-names = "INT1", "INT2";
        
        /* Optional features */
        wakeup-source;
    };
};
```

### 2.4 Interrupt Configuration Guidelines
**Mapping:** Mapping of IRQ function to pin is configured in Kconfig; dual pins → INT1 data/FIFO, INT2 events (recommended).  
**Edge:** Prefer `IRQ_TYPE_EDGE_RISING`; others (falling / level) supported.  
**GPIO:** Must support IRQ; ensure not shared; for wake use wake‑capable controller.

**GPIO Pin Requirements:**
* Pins must support interrupt generation
* Ensure pins are not used by other peripherals  
* For wake-up functionality, interrupts must be capable of waking the system

**Example Single Interrupt Configuration:**
```dts
interrupts = <26 IRQ_TYPE_EDGE_RISING>;
interrupt-names = "INT1";
/* Configure Kconfig to map both features to INT1 or disable advanced features*/
```

### 2.5 Alternative I2C Addresses

SMI330 supports two I2C addresses based on SDO pin state:
```dts
/* Default address (SDO low or floating) */
reg = <0x68>;

/* Alternative address (SDO high) */  
reg = <0x69>;
```

---
## 3. In-Tree Integration
We recommend to cross-compile the kernel on your host-machine.

### 3.1 Prerequisites
**Kernel ≥6.6 (recommended)** with: `CONFIG_IIO`, `CONFIG_IIO_BUFFER`, `CONFIG_IIO_TRIGGERED_BUFFER`, `CONFIG_REGMAP`, `CONFIG_I2C` and/or `CONFIG_SPI`, `CONFIG_OF`, `CONFIG_GPIOLIB`.  
**Environment:** Linux environment (native or VM).  
**Hardware:** SMI330 on I2C or SPI; ≥1 interrupt (recommended); 1.71–3.6 V supplies.

**Step 1: Install Toolchain and dependencies**
```bash
sudo apt install crossbuild-essential-armhf # (for arm)
sudo apt install crossbuild-essential-arm64 # (for arm64)
sudo apt install git bc bison flex libssl-dev make libc6-dev libncurses5-dev
```

**Step 2: Clone Linux kernel**
```bash
git clone --depth=1 --branch <branch_name> https://github.com/raspberrypi/linux
```

### 3.2 Integration

**Step 1: Copy Driver Files**
```bash
# Copy the entire smi330 directory to the kernel tree
cp -r drivers/iio/imu/smi330 <kernel_source>/drivers/iio/imu/
```

**Step 2: Update Kernel Configuration**

Edit `<kernel_source>/drivers/iio/imu/Kconfig` and add:
```kconfig
source "drivers/iio/imu/smi330/Kconfig"
```

Edit `<kernel_source>/drivers/iio/imu/Makefile` and add:
```makefile
obj-y += smi330/
```

**Step 3: Configure Kernel Build**
```bash
cd <kernel_source>
make ARCH=<arch> CROSS_COMPILE=<toolchain>- <defconfig>
make ARCH=<arch> CROSS_COMPILE=<toolchain>- menuconfig

# Navigate to: Device Drivers -> Industrial I/O support -> Inertial measurement units
# Enable:
#   [*] Bosch Sensor SMI330 Inertial Measurement Unit  
#   [*] Bosch SMI330 I2C driver (if using I2C)
#   [*] Bosch SMI330 SPI driver (if using SPI)
#   Configure interrupt mapping based on your hardware setup
```

**Step 4: Build and Install**
```bash
# Build kernel, modules, and device trees
make -j$(nproc) ARCH=<arch> CROSS_COMPILE=<toolchain>- Image modules dtbs

# Install to target system
make ARCH=<arch> CROSS_COMPILE=<toolchain>- INSTALL_MOD_PATH=<target_rootfs> modules_install
# Copy Image and dtb files to target system
```

---
## 4. Runtime Interface & Operation

The SMI330 driver provides a comprehensive IIO (Industrial I/O) interface for sensor configuration and data access.

### 4.1 Basic Sensor Channels

**Raw Data Channels:**
```bash
# Accelerometer raw values (16-bit signed)
/sys/bus/iio/devices/iio:deviceX/in_accel_x_raw
/sys/bus/iio/devices/iio:deviceX/in_accel_y_raw  
/sys/bus/iio/devices/iio:deviceX/in_accel_z_raw

# Gyroscope raw values (16-bit signed)
/sys/bus/iio/devices/iio:deviceX/in_anglvel_x_raw
/sys/bus/iio/devices/iio:deviceX/in_anglvel_y_raw
/sys/bus/iio/devices/iio:deviceX/in_anglvel_z_raw

# Temperature raw value
/sys/bus/iio/devices/iio:deviceX/in_temp_raw
```

**Scale & Offset:**
```bash
# Conversion factors (read-only)
/sys/bus/iio/devices/iio:deviceX/in_accel_scale      # g/LSB  
/sys/bus/iio/devices/iio:deviceX/in_anglvel_scale    # °/s/LSB
/sys/bus/iio/devices/iio:deviceX/in_temp_scale       # °C/LSB
/sys/bus/iio/devices/iio:deviceX/in_temp_offset      # Temperature offset
```

**Data Conversion:**
```
Acceleration [g] = in_accel_x_raw * in_accel_scale
Angular Rate [°/s] = in_anglvel_x_raw * in_anglvel_scale  
Temperature [°C] = (in_temp_raw + in_temp_offset) * in_temp_scale
```

### 4.2 Sensor Configuration

**Power Mode Control:**
```bash
# Enable/configure power modes (0=Suspend, 3=Low Power, 4=Normal, 7=Performance)
echo 4 > /sys/bus/iio/devices/iio:deviceX/in_accel_en
echo 4 > /sys/bus/iio/devices/iio:deviceX/in_anglvel_en
```

**Sampling Frequency (ODR):**
```bash
# Set sampling frequency (Hz)
echo 100 > /sys/bus/iio/devices/iio:deviceX/in_accel_sampling_frequency
echo 100 > /sys/bus/iio/devices/iio:deviceX/in_anglvel_sampling_frequency

# Supported values: 0=0.78125, 1=1.5625, 3=3.125, 6=6.25, 12=12.5, 25, 50, 100, 200, 400, 800, 1600, 3200, 6400
```

**Measurement Ranges:**
```bash
# Configure accelerometer range via scale
echo 0.000244140 > /sys/bus/iio/devices/iio:deviceX/in_accel_scale  # ±8g
# Available scales: 0.000061035 (±2g), 0.000122070 (±4g), 0.000244140 (±8g), 0.000488281 (±16g)

# Configure gyroscope range via scale  
echo 0.007629395 > /sys/bus/iio/devices/iio:deviceX/in_anglvel_scale  # ±250°/s
# Available scales: 0.003814697 (±125°/s), 0.007629395 (±250°/s), 0.015258789 (±500°/s)
```

**Oversampling/Averaging:**
```bash
# Set oversampling ratio for noise reduction (1, 2, 4, 8, 16, 32, 64)
echo 8 > /sys/bus/iio/devices/iio:deviceX/in_accel_oversampling_ratio
echo 8 > /sys/bus/iio/devices/iio:deviceX/in_anglvel_oversampling_ratio
```

**Low-Pass Filtering:**
```bash
# Set low-pass filter -3dB frequency (as ratio of ODR: 1, 2, 4, etc.)
echo 2 > /sys/bus/iio/devices/iio:deviceX/in_accel_filter_low_pass_3db_frequency
echo 2 > /sys/bus/iio/devices/iio:deviceX/in_anglvel_filter_low_pass_3db_frequency
```

### 4.3 Buffer & Streaming Operations

**IIO Buffer Configuration:**
```bash
# Configure software buffer
echo 256 > /sys/bus/iio/devices/iio:deviceX/buffer0/length
echo 64 > /sys/bus/iio/devices/iio:deviceX/buffer0/watermark

# Enable/disable channels for buffered acquisition
echo 1 > /sys/bus/iio/devices/iio:deviceX/scan_elements/in_accel_x_en
echo 1 > /sys/bus/iio/devices/iio:deviceX/scan_elements/in_accel_y_en
echo 1 > /sys/bus/iio/devices/iio:deviceX/scan_elements/in_accel_z_en
echo 1 > /sys/bus/iio/devices/iio:deviceX/scan_elements/in_timestamp_en

# Enable buffer
echo 1 > /sys/bus/iio/devices/iio:deviceX/buffer0/enable
```

**Hardware FIFO Status:**
```bash
# Monitor hardware FIFO (read-only)
cat /sys/bus/iio/devices/iio:deviceX/hwfifo_watermark      # Current watermark level
cat /sys/bus/iio/devices/iio:deviceX/hwfifo_fill_level     # Current fill level
cat /sys/bus/iio/devices/iio:deviceX/hwfifo_enabled        # FIFO enable status
```

**Trigger Assignment (Operation Mode Change):**
```bash
# Assign data-ready trigger (default)
echo $(cat /sys/bus/iio/devices/triggerX/name) > $sysfs_dir/trigger/current_trigger

# Remove trigger (switch to hardware FIFO mode if interrupt configured)
echo "" > /sys/bus/iio/devices/iio:deviceX/trigger/current_trigger
```

### 4.4 Advanced Features (interrupt required)

**Motion Detection Events:**
```bash
# Enable any-motion detection on specific axes
echo 1 > /sys/bus/iio/devices/iio:deviceX/events/in_accel_x_roc_rising_en
echo 1 > /sys/bus/iio/devices/iio:deviceX/events/in_accel_y_roc_rising_en
echo 1 > /sys/bus/iio/devices/iio:deviceX/events/in_accel_z_roc_rising_en

# Configure any-motion parameters
echo 156 > /sys/bus/iio/devices/iio:deviceX/events/roc_rising_value      # Threshold
echo 0 > /sys/bus/iio/devices/iio:deviceX/events/roc_rising_period       # Duration
echo 0 > /sys/bus/iio/devices/iio:deviceX/events/roc_rising_timeout      # Wait time

# Enable no-motion detection  
echo 1 > /sys/bus/iio/devices/iio:deviceX/events/in_accel_x_roc_falling_en

# Enable tilt detection
echo 1 > /sys/bus/iio/devices/iio:deviceX/events/in_accel_change_either_en
echo 48 > /sys/bus/iio/devices/iio:deviceX/events/change_either_value     # Minimum tilt angle
```

**Self-Test and Calibration:**
```bash
# Perform self-test (returns "passed" or "failed")
cat /sys/bus/iio/devices/iio:deviceX/self_test_acc
cat /sys/bus/iio/devices/iio:deviceX/self_test_gyro

# Execute gyroscope self-calibration
echo 1 > /sys/bus/iio/devices/iio:deviceX/self_cal

# Perform soft reset
echo 1 > /sys/bus/iio/devices/iio:deviceX/soft_reset
```

**Auto-Operation Mode:**
```bash
# Enable auto-operation mode (0=accel, 1=gyro, 2=both, 3=disable)
echo 2 > /sys/bus/iio/devices/iio:deviceX/control_auto_op_mode

# Configure switching conditions
echo 3 > /sys/bus/iio/devices/iio:deviceX/set_auto_op_mode_cond  # Switch to alt on no-motion

# Configure alternative sensor settings
echo 3 > /sys/bus/iio/devices/iio:deviceX/alt_acc_mode     # Low power mode
echo 25 > /sys/bus/iio/devices/iio:deviceX/alt_acc_odr     # 25 Hz ODR
echo 4 > /sys/bus/iio/devices/iio:deviceX/alt_acc_avg_num  # 4x averaging

# Check current configuration status
cat /sys/bus/iio/devices/iio:deviceX/alt_status
```

---
## 5. Verification & Testing

### 5.1 Driver Loading Verification

**Step 1: Check Module Loading**
```bash
# Verify module is loaded
lsmod | grep smi330
# smi330_spi             16384  0
# smi330_core            32768  1 smi330_spi
```

**Step 2: IIO Device Detection**
```bash
# Find IIO device
ls /sys/bus/iio/devices/
# iio:device0  trigger0

# Verify it's SMI330
cat /sys/bus/iio/devices/iio:device0/name
# smi330
```

### 5.2 Functional Testing

**Basic Data Reading:**
```bash
# Enable sensors in normal mode
echo 4 > /sys/bus/iio/devices/iio:device0/in_accel_en
echo 4 > /sys/bus/iio/devices/iio:device0/in_anglvel_en

# Set reasonable ODR
echo 100 > /sys/bus/iio/devices/iio:device0/sampling_frequency

# Read raw values multiple times
for i in {1..5}; do
    echo "Reading $i:"
    cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
    cat /sys/bus/iio/devices/iio:device0/in_accel_y_raw
    cat /sys/bus/iio/devices/iio:device0/in_accel_z_raw
    sleep 1
done
```

**Interrupt Verification (if configured):**
```bash
# Check interrupt registration
cat /proc/interrupts | grep smi330
#  26:         42          0   GPIO  26 Edge      smi330
#  20:          0          0   GPIO  20 Edge      smi330

# Check interrupt counters during data acquisition
watch "cat /proc/interrupts | grep smi330"
```

**Buffer Mode Testing:**
```bash
# Use kernel IIO tools (if available)
./iio_generic_buffer -n smi330 -a -g -l 100 -c 10

# Or enable buffer manually
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en
echo 1 > /sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en
echo 1 > /sys/bus/iio/devices/iio:device0/buffer0/enable

# Read buffer data
hexdump -C /dev/iio:device0 | head -20
```

**Event Testing (if advanced features enabled):**
```bash
# Enable any-motion detection
echo 4 > /sys/bus/iio/devices/iio:device0/in_accel_en
echo 1 > /sys/bus/iio/devices/iio:device0/events/in_accel_x_roc_rising_en

# Monitor events
./iio_event_monitor smi330 &

# Generate motion to trigger events
# Should see event messages when sensor is moved
```

### 5.3 Troubleshooting Common Issues (Quick Hints)

**Issue: No IIO device appears**
```bash
# Check dmesg for probe errors
dmesg | grep -E "(smi330|iio|spi|i2c|regmap)"

# Verify device tree loading
ls /proc/device-tree/soc/*/smi330* 2>/dev/null || echo "No DT node found"

# Check bus controllers
ls /sys/class/spi_master/  # For SPI
ls /sys/class/i2c-dev/     # For I2C
```

**Issue: All sensor readings are zero**
```bash
# Check power mode (should not be 0=suspend)
cat /sys/bus/iio/devices/iio:device0/in_accel_en
cat /sys/bus/iio/devices/iio:device0/in_anglvel_en

# Verify ODR is set
cat /sys/bus/iio/devices/iio:device0/sampling_frequency

# Check for device errors
dmesg | tail -20 | grep -i error
```

**Issue: Interrupts not working**
```bash
# Verify GPIO interrupt configuration
cat /sys/kernel/debug/gpio | grep -A5 -B5 "26\|20"  # Replace with your GPIO numbers

# Check interrupt flags
cat /proc/interrupts | grep smi330

# Verify edge configuration in device tree matches hardware
```

**Issue: FIFO overruns**
```bash
# Increase software buffer size
echo 1024 > /sys/bus/iio/devices/iio:device0/buffer0/length

# Reduce ODR or increase watermark
echo 50 > /sys/bus/iio/devices/iio:device0/sampling_frequency
echo 100 > /sys/bus/iio/devices/iio:device0/buffer0/watermark
```

---
## 6. Known Limitations

**Hardware Limitations:**
* **Gyroscope Range:** Currently limited to ±125/250/500°/s (hardware supports up to ±2000°/s)
* **FIFO Timestamp:** Hardware FIFO does not store timestamps; software timestamps added during readout

**Driver Implementation Limitations:**
* **Power Mode Constraints:** Low Power mode limited to ODR ≤400 Hz; higher rates require Normal/Performance mode
* **Auto-Operation Dependency:** Requires proper advanced feature interrupt configuration
* **Event Limitations:** Motion detection events require accelerometer in Normal/Performance mode

**Platform-Specific Considerations:**
* **Interrupt Sharing:** Cannot share interrupt lines with other devices
* **GPIO Limitations:** Some platforms may not support both edge types or open-drain configuration
* **Bus Arbitration:** High-frequency operation may impact other devices on shared I2C/SPI bus
* **Real-time Constraints:** Very high data rates (>3200 Hz) may require real-time kernel and careful system tuning

---
## 7. Documentation & Support Resources

### 7.1 Technical Documentation

**Driver Documentation:**
* **Device Tree Binding:** `dts/binding/bosch,smi330.yaml` - Complete DT property specification
* **Sysfs Interface:** `Documentation/sysfs-bus-iio-smi330` - All available sysfs attributes and their usage
* **Release Guide:** This document - Integration and operation guide

**Online Resources:**
* **Linux IIO Documentation:** `Documentation/driver-api/iio/` in kernel source
* **IIO ABI Documentation:** `Documentation/ABI/testing/sysfs-bus-iio`

**Hardware Documentation:**
* **Product Information:** https://www.bosch-semiconductors.com/products/mems-sensors/adas/smi330/
* **Datasheet:** Available through Bosch technical support channels
* **Application Notes:** Check Bosch developer resources for integration guides

### 7.2 Related Driver Projects

**Linux IIO Examples:**
* **IIO Tools:** `tools/iio/` in kernel source tree
* **IIO Utilities:** Various distributions provide `iio-utils` packages
* **Example Applications:** Available in kernel documentation
