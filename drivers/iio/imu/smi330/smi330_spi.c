// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "smi330.h"

static int smi330_regmap_spi_read(void *context, const void *reg_buf,
				  size_t reg_size, void *val_buf,
				  size_t val_size)
{
	struct spi_device *spi = context;

	return spi_write_then_read(spi, reg_buf, reg_size, val_buf, val_size);
}

static int smi330_regmap_spi_write(void *context, const void *data,
				   size_t count)
{
	struct spi_device *spi = context;
	u8 *data_buff = (u8 *)data;

	data_buff[1] = data_buff[0];
	return spi_write(spi, data_buff + 1, count - 1);
}

static const struct regmap_bus smi330_regmap_bus = {
	.read = smi330_regmap_spi_read,
	.write = smi330_regmap_spi_write,
	.read_flag_mask = 0x80,
};

static const struct regmap_config smi330_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.pad_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int smi330_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init(&spi->dev, &smi330_regmap_bus, &spi->dev,
				  &smi330_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "Failed to initialize SPI Regmap\n");

	return smi330_core_probe(&spi->dev, regmap);
}

static const struct spi_device_id smi330_spi_id[] = { { "smi330", 0 }, {} };
MODULE_DEVICE_TABLE(spi, smi330_spi_id);

static const struct of_device_id smi330_of_match[] = {
	{ .compatible = "bosch,smi330" },
	{},
};
MODULE_DEVICE_TABLE(of, smi330_of_match);

static struct spi_driver smi330_spi_driver = {
	.probe = smi330_spi_probe,
	.id_table = smi330_spi_id,
	.driver = {
		.of_match_table = smi330_of_match,
		.name = "smi330_spi",
	},
};
module_spi_driver(smi330_spi_driver);
