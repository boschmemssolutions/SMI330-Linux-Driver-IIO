# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Bosch SMI330 IMU
#
obj-$(CONFIG_SMI330) += smi330.o
smi330-objs := smi330_core.o

ifeq ($(CONFIG_SMI330_I2C),y)
    smi330-objs += smi330_i2c.o
else ifeq ($(CONFIG_SMI330_I2C),m)
    smi330-objs += smi330_i2c.o
else ifeq ($(CONFIG_SMI330_SPI),y)
    smi330-objs += smi330_spi.o
else ifeq ($(CONFIG_SMI330_SPI),m)
    smi330-objs += smi330_spi.o
endif
