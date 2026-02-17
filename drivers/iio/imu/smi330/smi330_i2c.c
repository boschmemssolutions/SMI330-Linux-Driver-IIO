// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright (c) 2024, 2025 Robert Bosch GmbH. All rights reserved.
 *
 * GPL LICENSE
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "smi330.h"

#define SMI330_NUM_DUMMY_BYTES 2
#define SMI330_I2C_MAX_RX_BUFFER_SIZE \
	(SMI330_NUM_DUMMY_BYTES + SMI330_FIFO_SIZE)

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0)
#define pm_sleep_ptr(_ptr) PTR_IF(IS_ENABLED(CONFIG_PM_SLEEP), (_ptr))
#endif

struct smi330_i2c_priv {
	struct i2c_client *i2c;
	u8 rx_buffer[SMI330_I2C_MAX_RX_BUFFER_SIZE];
};

static int smi330_regmap_i2c_read(void *context, const void *reg_buf,
				  size_t reg_size, void *val_buf,
				  size_t val_size)
{
	struct smi330_i2c_priv *priv = context;
	int ret;

	/*
	 * SMI330 I2C read frame:
	 * <Slave address[6:0], RnW> <x, Register address[6:0]>
	 * <Slave address[6:0], RnW> <Dummy[7:0]> <Dummy[7:0]> <Data_0[7:0]> <Data_1[15:8]>...
	 *                                                     <Data_N[7:0]> <Data_N[15:8]>
	 * Remark: Slave address is not considered part of the frame in the following definitions
	 */
	struct i2c_msg msgs[] = {
		{
			.addr = priv->i2c->addr,
			.flags = priv->i2c->flags,
			.len = reg_size,
			.buf = (u8 *)reg_buf,
		},
		{
			.addr = priv->i2c->addr,
			.flags = priv->i2c->flags | I2C_M_RD,
			.len = SMI330_NUM_DUMMY_BYTES + val_size,
			.buf = priv->rx_buffer,
		},
	};

	if (SMI330_NUM_DUMMY_BYTES + val_size > SMI330_I2C_MAX_RX_BUFFER_SIZE)
		return -EINVAL;

	ret = i2c_transfer(priv->i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	memcpy(val_buf, priv->rx_buffer + SMI330_NUM_DUMMY_BYTES, val_size);

	return 0;
}

static int smi330_regmap_i2c_write(void *context, const void *data,
				   size_t count)
{
	struct smi330_i2c_priv *priv = context;
	u8 reg;

	/*
	 * SMI330 I2C write frame:
	 * <Slave address[6:0], RnW> <x, Register address[6:0]> <Data_0[7:0]> <Data_1[15:8]>...
	 *                                                      <Data_N[7:0]> <Data_N[15:8]>
	 * Remark: Slave address is not considered part of the frame in the following definitions
	 */
	reg = *(u8 *)data;
	return i2c_smbus_write_i2c_block_data(priv->i2c, reg,
					      count - sizeof(u8),
					      data + sizeof(u8));
}

static const struct regmap_bus smi330_regmap_bus = {
	.read = smi330_regmap_i2c_read,
	.write = smi330_regmap_i2c_write,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int smi330_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
#else
static int smi330_i2c_probe(struct i2c_client *i2c)
#endif
{
	struct device *dev = &i2c->dev;
	struct smi330_i2c_priv *priv;
	struct regmap *regmap;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i2c = i2c;
	regmap = devm_regmap_init(dev, &smi330_regmap_bus, priv,
				  &smi330_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize I2C Regmap\n");

	return smi330_core_probe(dev, regmap);
}

static const struct i2c_device_id smi330_i2c_device_id[] = {
	{ .name = "smi330" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi330_i2c_device_id);

static const struct of_device_id smi330_of_match[] = {
	{ .compatible = "bosch,smi330" },
	{ }
};
MODULE_DEVICE_TABLE(of, smi330_of_match);

static struct i2c_driver smi330_i2c_driver = {
	.probe = smi330_i2c_probe,
	.id_table = smi330_i2c_device_id,
	.driver = {
		.of_match_table = smi330_of_match,
		.name = "smi330_i2c",
	        .pm = pm_sleep_ptr(&smi330_pm_ops),
	},
};
module_i2c_driver(smi330_i2c_driver);

MODULE_AUTHOR("Stefan Gutmann <stefan.gutmann@de.bosch.com>");
MODULE_AUTHOR("Roman Huber <roman.huber@de.bosch.com>");
MODULE_AUTHOR("Filip Andrei <Andrei.Filip@ro.bosch.com>");
MODULE_AUTHOR("Drimbarean Avram Andrei <Avram-Andrei.Drimbarean@ro.bosch.com>");
MODULE_DESCRIPTION("Bosch SMI330 I2C driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_IMPORT_NS(IIO_SMI330);
