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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/units.h>
#include <linux/version.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

#include "smi330.h"

static struct iio_event_spec smi330_accel_events[] = {
	/* Any-Motion */
	{
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD) |
			BIT(IIO_EV_INFO_HYSTERESIS) | BIT(IIO_EV_INFO_TIMEOUT),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	/* No-Motion */
	{
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD) |
			BIT(IIO_EV_INFO_HYSTERESIS) | BIT(IIO_EV_INFO_TIMEOUT),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	/* Tilt-Detection */
	{
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_EITHER,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_PERIOD) |
				      BIT(IIO_EV_INFO_LOW_PASS_FILTER_3DB),
		.mask_shared_by_type = BIT(IIO_EV_INFO_ENABLE),
	},
};

// clang-format off
#define SMI330_ACCEL_CHANNEL(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type =					\
		BIT(IIO_CHAN_INFO_ENABLE) |				\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |			\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_ENABLE) |				\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |			\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.info_mask_shared_by_dir =					\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_dir_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
	.event_spec = smi330_accel_events,				\
	.num_event_specs = ARRAY_SIZE(smi330_accel_events),		\
}

#define SMI330_GYRO_CHANNEL(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type =					\
		BIT(IIO_CHAN_INFO_ENABLE) |				\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |			\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_ENABLE) |				\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |			\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.info_mask_shared_by_dir =					\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_dir_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
}

#define SMI330_TEMP_CHANNEL(_index) {			\
	.type = IIO_TEMP,				\
	.modified = 1,					\
	.channel2 = IIO_MOD_TEMP_OBJECT,		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
		BIT(IIO_CHAN_INFO_OFFSET) |		\
		BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = _index,				\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = 16,				\
		.storagebits = 16,			\
		.endianness = IIO_LE,			\
	},						\
}

// clang-format on

static const struct iio_chan_spec smi330_channels[] = {
	SMI330_ACCEL_CHANNEL(IIO_ACCEL, X, SMI330_SCAN_ACCEL_X),
	SMI330_ACCEL_CHANNEL(IIO_ACCEL, Y, SMI330_SCAN_ACCEL_Y),
	SMI330_ACCEL_CHANNEL(IIO_ACCEL, Z, SMI330_SCAN_ACCEL_Z),
	SMI330_GYRO_CHANNEL(IIO_ANGL_VEL, X, SMI330_SCAN_GYRO_X),
	SMI330_GYRO_CHANNEL(IIO_ANGL_VEL, Y, SMI330_SCAN_GYRO_Y),
	SMI330_GYRO_CHANNEL(IIO_ANGL_VEL, Z, SMI330_SCAN_GYRO_Z),
	SMI330_TEMP_CHANNEL(SMI330_TEMP_OBJECT),
	IIO_CHAN_SOFT_TIMESTAMP(SMI330_SCAN_TIMESTAMP),
};

static const struct smi330_sysfs_attr smi330_mode_attr = {
	.reg_vals = (int[]){ SMI330_MODE_SUSPEND, SMI330_MODE_LOW_POWER,
			     SMI330_MODE_NORMAL, SMI330_MODE_HIGH_PERF },
	.vals = (int[]){ 0, 3, 4, 7 },
	.len = 4,
	.type = IIO_VAL_INT
};

static const struct smi330_sysfs_attr smi330_mode_alt_gyro_attr = {
	.reg_vals = (int[]){ SMI330_MODE_SUSPEND, SMI330_MODE_GYRO_DRIVE,
			     SMI330_MODE_LOW_POWER, SMI330_MODE_NORMAL,
			     SMI330_MODE_HIGH_PERF },
	.vals = (int[]){ 0, 1, 3, 4, 7 },
	.len = 5,
	.type = IIO_VAL_INT
};

static const struct smi330_sysfs_attr smi330_accel_scale_attr = {
	.reg_vals = (int[]){ SMI330_ACCEL_RANGE_2G, SMI330_ACCEL_RANGE_4G,
			     SMI330_ACCEL_RANGE_8G, SMI330_ACCEL_RANGE_16G },
	.vals = (int[]){ 0, 61035, 0, 122070, 0, 244140, 0, 488281 },
	.len = 8,
	.type = IIO_VAL_INT_PLUS_NANO
};

static const struct smi330_sysfs_attr smi330_gyro_scale_attr = {
	.reg_vals = (int[]){ SMI330_GYRO_RANGE_125, SMI330_GYRO_RANGE_250,
			     SMI330_GYRO_RANGE_500 },
	.vals = (int[]){ 0, 3814697, 0, 7629395, 0, 15258789 },
	.len = 6,
	.type = IIO_VAL_INT_PLUS_NANO
};

static const struct smi330_sysfs_attr smi330_average_attr = {
	.reg_vals = (int[]){ SMI330_AVG_NUM_1, SMI330_AVG_NUM_2,
			     SMI330_AVG_NUM_4, SMI330_AVG_NUM_8,
			     SMI330_AVG_NUM_16, SMI330_AVG_NUM_32,
			     SMI330_AVG_NUM_64 },
	.vals = (int[]){ 1, 2, 4, 8, 16, 32, 64 },
	.len = 7,
	.type = IIO_VAL_INT
};

static const struct smi330_sysfs_attr smi330_bandwidth_attr = {
	.reg_vals = (int[]){ SMI330_BW_2, SMI330_BW_4 },
	.vals = (int[]){ 2, 4 },
	.len = 2,
	.type = IIO_VAL_INT
};

static const struct smi330_sysfs_attr smi330_odr_attr = {
	.reg_vals = (int[]){ SMI330_ODR_0_78125_HZ, SMI330_ODR_1_5625_HZ,
			     SMI330_ODR_3_125_HZ, SMI330_ODR_6_25_HZ,
			     SMI330_ODR_12_5_HZ, SMI330_ODR_25_HZ,
			     SMI330_ODR_50_HZ, SMI330_ODR_100_HZ,
			     SMI330_ODR_200_HZ, SMI330_ODR_400_HZ,
			     SMI330_ODR_800_HZ, SMI330_ODR_1600_HZ,
			     SMI330_ODR_3200_HZ, SMI330_ODR_6400_HZ },
	.vals = (int[]){ 0, 1, 3, 6, 12, 25, 50, 100, 200, 400, 800, 1600, 3200,
			 6400 },
	.len = 14,
	.type = IIO_VAL_INT
};

static int smi330_dev_init(struct smi330_data *data);
static int smi330_get_odr_ns(enum smi330_odr odr, int64_t *odr_ns);
static int smi330_get_sensor_config_reg(struct smi330_data *data,
					enum smi330_sensor sensor,
					union smi330_sensor_conf *cfg);
static int smi330_set_sensor_config(struct smi330_data *data,
				    enum smi330_sensor sensor,
				    enum smi330_sensor_conf_select config,
				    int reg_value);

static int smi330_get_sysfs_attr(enum smi330_sensor_conf_select config,
				 enum smi330_sensor sensor,
				 const struct smi330_sysfs_attr **attr)
{
	switch (config) {
	case SMI330_ODR:
		*attr = &smi330_odr_attr;
		return 0;
	case SMI330_RANGE:
		if (sensor == SMI330_ACCEL)
			*attr = &smi330_accel_scale_attr;
		else
			*attr = &smi330_gyro_scale_attr;
		return 0;
	case SMI330_BW:
		*attr = &smi330_bandwidth_attr;
		return 0;
	case SMI330_AVG_NUM:
		*attr = &smi330_average_attr;
		return 0;
	case SMI330_MODE:
		if (sensor == SMI330_ALT_GYRO)
			*attr = &smi330_mode_alt_gyro_attr;
		else
			*attr = &smi330_mode_attr;
		return 0;
	default:
		return -EINVAL;
	}
}

static int smi330_value_to_reg(int value, enum smi330_sensor_conf_select config,
			       enum smi330_sensor sensor, int *reg_value)
{
	int ret, i;
	const struct smi330_sysfs_attr *attr;

	ret = smi330_get_sysfs_attr(config, sensor, &attr);
	if (ret)
		return ret;

	for (i = 0; i < attr->len; i++) {
		if (attr->vals[i] == value) {
			if (attr->type == IIO_VAL_INT)
				*reg_value = attr->reg_vals[i];
			else
				*reg_value = attr->reg_vals[i / 2];
			return 0;
		}
	}

	return -EINVAL;
}

static int smi330_reg_to_value(int reg_value,
			       enum smi330_sensor_conf_select config,
			       enum smi330_sensor sensor, int *value)
{
	int ret, i;
	const struct smi330_sysfs_attr *attr;

	ret = smi330_get_sysfs_attr(config, sensor, &attr);
	if (ret)
		return ret;

	if (attr->type == IIO_VAL_INT) {
		for (i = 0; i < attr->len; i++) {
			if (attr->reg_vals[i] == reg_value) {
				*value = attr->vals[i];
				return 0;
			}
		}
	} else {
		for (i = 0; i < attr->len / 2; i++) {
			if (attr->reg_vals[i] == reg_value) {
				*value = attr->vals[2 * i + 1];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int smi330_get_regs_dma(u8 reg_addr, s16 *reg_data, size_t len,
			       struct smi330_data *data)
{
	int ret;
	union smi330_feature_data_status status = { 0 };

	ret = regmap_read(data->regmap, SMI330_FEATURE_DATA_STATUS_REG,
			  &status.value);
	if (ret)
		return ret;

	if (!status.fields.data_tx_ready)
		return -EBUSY;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   reg_addr);
	if (ret)
		return ret;

	return regmap_bulk_read(data->regmap, SMI330_FEATURE_DATA_TX_REG,
				reg_data, len);
}

static int smi330_set_regs_dma(u8 reg_addr, const s16 *reg_data, size_t len,
			       struct smi330_data *data)
{
	int ret;
	union smi330_feature_data_status status = { 0 };

	ret = regmap_read(data->regmap, SMI330_FEATURE_DATA_STATUS_REG,
			  &status.value);
	if (ret)
		return ret;

	if (!status.fields.data_tx_ready)
		return -EBUSY;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   reg_addr);
	if (ret)
		return ret;

	return regmap_bulk_write(data->regmap, SMI330_FEATURE_DATA_TX_REG,
				 reg_data, len);
}

static int smi330_get_data(struct smi330_data *data, int chan_type, int axis,
			   int *val)
{
	u8 reg;
	int ret, sample;

	switch (chan_type) {
	case IIO_ACCEL:
		reg = SMI330_ACCEL_X_REG + (axis - IIO_MOD_X);
		break;
	case IIO_ANGL_VEL:
		reg = SMI330_GYRO_X_REG + (axis - IIO_MOD_X);
		break;
	case IIO_TEMP:
		reg = SMI330_TEMP_REG;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(data->regmap, reg, &sample);
	if (ret)
		return ret;

	*val = sign_extend32(sample, 15);

	return 0;
}

static int smi330_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*vals = smi330_mode_attr.vals;
		*length = smi330_mode_attr.len;
		*type = smi330_mode_attr.type;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ACCEL) {
			*vals = smi330_accel_scale_attr.vals;
			*length = smi330_accel_scale_attr.len;
			*type = smi330_accel_scale_attr.type;
		} else {
			*vals = smi330_gyro_scale_attr.vals;
			*length = smi330_gyro_scale_attr.len;
			*type = smi330_gyro_scale_attr.type;
		}
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = smi330_average_attr.vals;
		*length = smi330_average_attr.len;
		*type = smi330_average_attr.type;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = smi330_bandwidth_attr.vals;
		*length = smi330_bandwidth_attr.len;
		*type = smi330_bandwidth_attr.type;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = smi330_odr_attr.vals;
		*length = smi330_odr_attr.len;
		*type = smi330_odr_attr.type;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int smi330_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	int ret;
	struct smi330_data *data = iio_priv(indio_dev);
	enum smi330_sensor sensor;
	union smi330_sensor_conf cfg = { 0 };

	if (chan->type == IIO_ACCEL)
		sensor = SMI330_ACCEL;
	else if (chan->type == IIO_ANGL_VEL)
		sensor = SMI330_GYRO;
	else
		sensor = SMI330_ALL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = smi330_get_data(data, chan->type, chan->channel2, val);
		iio_device_release_direct_mode(indio_dev);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_ENABLE:
		ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
		if (ret)
			return ret;
		ret = smi330_reg_to_value(cfg.fields.mode, SMI330_MODE, sensor,
					  val);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
		if (ret)
			return ret;
		ret = smi330_reg_to_value(cfg.fields.avg_num, SMI330_AVG_NUM,
					  sensor, val);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
		if (ret)
			return ret;
		ret = smi330_reg_to_value(cfg.fields.bw, SMI330_BW, sensor,
					  val);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
		if (ret)
			return ret;
		ret = smi330_reg_to_value(cfg.fields.odr, SMI330_ODR, sensor,
					  val);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_TEMP:
			*val = SMI330_TEMP_SCALE / GIGA;
			*val2 = SMI330_TEMP_SCALE % GIGA;
			return IIO_VAL_INT_PLUS_NANO;
		default:
			ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
			if (ret)
				return ret;
			*val = 0;
			ret = smi330_reg_to_value(cfg.fields.range,
						  SMI330_RANGE, sensor, val2);
			return ret ? ret : IIO_VAL_INT_PLUS_NANO;
		}

	case IIO_CHAN_INFO_OFFSET:
		*val = SMI330_TEMP_OFFSET;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int smi330_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret, reg_value;
	struct smi330_data *data = iio_priv(indio_dev);
	enum smi330_sensor sensor;

	if (chan->type == IIO_ACCEL)
		sensor = SMI330_ACCEL;
	else if (chan->type == IIO_ANGL_VEL)
		sensor = SMI330_GYRO;
	else
		sensor = SMI330_ALL;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = smi330_value_to_reg(val, SMI330_MODE, sensor, &reg_value);
		if (ret)
			return ret;
		return smi330_set_sensor_config(data, sensor, SMI330_MODE,
						reg_value);
	case IIO_CHAN_INFO_SCALE:
		ret = smi330_value_to_reg(val2, SMI330_RANGE, sensor,
					  &reg_value);
		if (ret)
			return ret;
		return smi330_set_sensor_config(data, sensor, SMI330_RANGE,
						reg_value);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		ret = smi330_value_to_reg(val, SMI330_AVG_NUM, sensor,
					  &reg_value);
		if (ret)
			return ret;
		return smi330_set_sensor_config(data, sensor, SMI330_AVG_NUM,
						reg_value);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = smi330_value_to_reg(val, SMI330_BW, sensor, &reg_value);
		if (ret)
			return ret;
		return smi330_set_sensor_config(data, sensor, SMI330_BW,
						reg_value);
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = smi330_value_to_reg(val, SMI330_ODR, sensor, &reg_value);
		if (ret)
			return ret;

		ret = smi330_set_sensor_config(data, SMI330_ACCEL, SMI330_ODR,
					       reg_value);
		if (ret)
			return ret;

		ret = smi330_set_sensor_config(data, SMI330_GYRO, SMI330_ODR,
					       reg_value);
		if (ret)
			return ret;

		return smi330_get_odr_ns(reg_value, &data->cfg.odr_ns);
	default:
		return -EINVAL;
	}
}

static int smi330_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
}

static int smi330_validate_sensor_config(enum smi330_mode mode,
					 enum smi330_odr odr,
					 enum smi330_avg_num avg)
{
	int ret, avg_num;
	s64 odr_ns, min_odr_ns, skipped_samples;

	if (mode == SMI330_MODE_LOW_POWER) {
		if (odr > SMI330_ODR_400_HZ)
			return -EINVAL;

		ret = smi330_get_odr_ns(SMI330_ODR_6400_HZ, &min_odr_ns);
		if (ret)
			return ret;

		ret = smi330_get_odr_ns(odr, &odr_ns);
		if (ret)
			return ret;

		ret = smi330_reg_to_value(avg, SMI330_AVG_NUM, SMI330_ALL,
					  &avg_num);
		if (ret)
			return ret;

		skipped_samples = div_s64(odr_ns, min_odr_ns) - avg_num;

		if (skipped_samples <= 0)
			return -EINVAL;
	}

	if (mode == SMI330_MODE_NORMAL || mode == SMI330_MODE_HIGH_PERF) {
		if (odr <= SMI330_ODR_6_25_HZ)
			return -EINVAL;
	}

	return 0;
}

static int smi330_get_sensor_config_reg(struct smi330_data *data,
					enum smi330_sensor sensor,
					union smi330_sensor_conf *cfg)
{
	u8 reg;

	if (sensor == SMI330_ACCEL)
		reg = SMI330_ACCEL_CFG_REG;
	else if (sensor == SMI330_GYRO)
		reg = SMI330_GYRO_CFG_REG;
	else if (sensor == SMI330_ALT_ACCEL)
		reg = SMI330_ALT_ACCEL_CFG_REG;
	else if (sensor == SMI330_ALT_GYRO)
		reg = SMI330_ALT_GYRO_CFG_REG;
	else
		return -EINVAL;

	return regmap_read(data->regmap, reg, &cfg->value);
}

static int smi330_set_sensor_config_reg(struct smi330_data *data,
					enum smi330_sensor sensor,
					union smi330_sensor_conf cfg)
{
	int ret, tmp;
	u8 reg;
	union smi330_error_reg error_reg = { 0 };

	ret = smi330_validate_sensor_config(cfg.fields.mode, cfg.fields.odr,
					    cfg.fields.avg_num);
	if (ret)
		return ret;

	switch (sensor) {
	case SMI330_ACCEL:
		reg = SMI330_ACCEL_CFG_REG;
		break;
	case SMI330_GYRO:
		reg = SMI330_GYRO_CFG_REG;
		break;
	case SMI330_ALT_ACCEL:
		reg = SMI330_ALT_ACCEL_CFG_REG;
		break;
	case SMI330_ALT_GYRO:
		reg = SMI330_ALT_GYRO_CFG_REG;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(data->regmap, reg, cfg.value);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, reg, &tmp);
	if (ret)
		return ret;

	if (memcmp(&cfg.value, &tmp, sizeof(tmp)) != 0)
		return -EIO;

	ret = regmap_read(data->regmap, SMI330_ERR_REG, &error_reg.value);
	if (ret)
		return ret;

	if (error_reg.fields.acc_conf_err || error_reg.fields.gyr_conf_err)
		return -EIO;

	return smi330_get_odr_ns(cfg.fields.odr, &data->cfg.odr_ns);
}

static int smi330_set_sensor_config(struct smi330_data *data,
				    enum smi330_sensor sensor,
				    enum smi330_sensor_conf_select config,
				    int reg_value)
{
	int ret;
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, sensor, &cfg);
	if (ret)
		return ret;

	switch (config) {
	case SMI330_ODR:
		cfg.fields.odr = reg_value;
		break;
	case SMI330_RANGE:
		cfg.fields.range = reg_value;
		break;
	case SMI330_BW:
		cfg.fields.bw = reg_value;
		break;
	case SMI330_AVG_NUM:
		cfg.fields.avg_num = reg_value;
		break;
	case SMI330_MODE:
		cfg.fields.mode = reg_value;
		break;
	default:
		return -EINVAL;
	}

	return smi330_set_sensor_config_reg(data, sensor, cfg);
}

static u16 smi330_get_fifo_length(struct smi330_data *data)
{
	int ret, fill_level;

	ret = regmap_read(data->regmap, SMI330_FIFO_FILL_LEVEL_REG,
			  &fill_level);
	if (ret == 0)
		return fill_level & SMI330_FIFO_FILL_LEVEL_MASK;
	else
		return 0;
}

static int smi330_get_fifo_frame_length(struct iio_dev *indio_dev)
{
	int i;
	int fifo_frame_length = 0;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		fifo_frame_length++;
	}

	return fifo_frame_length;
}

static int
smi330_config_auto_operation_mode(struct smi330_data *data,
				  enum smi330_auto_op_mode auto_op_mode)
{
	int ret;
	union smi330_alt_conf alt_conf = { 0 };

	ret = regmap_read(data->regmap, SMI330_ALT_CONF_REG, &alt_conf.value);
	if (ret)
		return ret;

	switch (auto_op_mode) {
	case SMI330_AUTO_OP_EN_ALL:
		alt_conf.fields.alt_acc_en = SMI330_AUTO_OP_SET;
		alt_conf.fields.alt_gyr_en = SMI330_AUTO_OP_SET;
		break;
	case SMI330_AUTO_OP_EN_ACC:
		alt_conf.fields.alt_acc_en = SMI330_AUTO_OP_SET;
		alt_conf.fields.alt_gyr_en = SMI330_AUTO_OP_RESET;
		break;
	case SMI330_AUTO_OP_EN_GYR:
		alt_conf.fields.alt_acc_en = SMI330_AUTO_OP_RESET;
		alt_conf.fields.alt_gyr_en = SMI330_AUTO_OP_SET;
		break;
	case SMI330_AUTO_OP_DISABLE:
		alt_conf.value = SMI330_AUTO_OP_RESET;
		ret = smi330_set_regs_dma(SMI330_ALT_CONF_CHG_REG,
					  (s16 *)&alt_conf.value, 1, data);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return regmap_write(data->regmap, SMI330_ALT_CONF_REG, alt_conf.value);
}

static int
smi330_auto_op_cfg_user_overwrite(struct smi330_data *data,
				  enum smi330_auto_op_setup user_overwrite)
{
	int ret;
	union smi330_alt_conf alt_conf = { 0 };

	ret = regmap_read(data->regmap, SMI330_ALT_CONF_REG, &alt_conf.value);
	if (ret)
		return ret;

	alt_conf.fields.alt_rst_conf_write_en = user_overwrite;

	return regmap_write(data->regmap, SMI330_ALT_CONF_REG, alt_conf.value);
}

static int
smi330_set_auto_op_mode_cond(struct smi330_data *data,
			     enum smi330_auto_op_config auto_op_config,
			     enum smi330_auto_op_adv_feat auto_op_adv_feat)
{
	int ret;
	union smi330_alt_config_chg alt_config_chg = { 0 };

	ret = smi330_get_regs_dma(SMI330_ALT_CONF_CHG_REG,
				  (s16 *)&alt_config_chg.value, 1, data);
	if (ret)
		return ret;

	if (auto_op_config == SMI330_AUTO_OP_CONFIG_ALT) {
		if (alt_config_chg.fields.alt_conf_user_switch_src_select !=
		    auto_op_adv_feat)
			alt_config_chg.fields.alt_conf_alt_switch_src_select =
				auto_op_adv_feat;
		else
			return -EIO;
	} else if (auto_op_config == SMI330_AUTO_OP_CONFIG_USER) {
		if (alt_config_chg.fields.alt_conf_alt_switch_src_select !=
		    auto_op_adv_feat)
			alt_config_chg.fields.alt_conf_user_switch_src_select =
				auto_op_adv_feat;
		else
			return -EIO;
	}

	return smi330_set_regs_dma(SMI330_ALT_CONF_CHG_REG,
				   (s16 *)&alt_config_chg.value, 1, data);
}

static int smi330_get_st_result(struct smi330_data *data)
{
	int ret;
	int sc_st_base_addr = SMI330_BASE_ADDR_ST_RESULT;

	union smi330_st_result st_result = { 0 };
	union smi330_io1_data io1_data = { 0 };

	ret = regmap_read_poll_timeout(data->regmap, SMI330_FEATURE_IO1_REG,
				       io1_data.value,
				       io1_data.fields.sc_st_complete == 1,
				       SMI330_FEAT_ENG_POLL,
				       SMI330_FEAT_ENG_TIMEOUT);
	if (ret)
		return ret;

	if (io1_data.fields.st_result) {
		// get the self-test result for all sensors
		ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
				   sc_st_base_addr);
		if (ret)
			return ret;

		ret = regmap_read(data->regmap, SMI330_FEATURE_DATA_TX_REG,
				  &st_result.value);
		if (ret)
			return ret;

		// Check if all tests passed
		if (st_result.value != SMI330_ST_SUCCESS_MASK)
			return -EIO;
	} else {
		return -EIO;
	}

	return 0;
}

static int smi330_disable_alt_conf_acc_gyr_mode(struct smi330_data *data)
{
	int ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_MODE,
				       SMI330_MODE_SUSPEND);
	if (ret)
		return ret;

	return smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_MODE,
					SMI330_MODE_SUSPEND);
}

static int smi330_st_precondition(struct smi330_data *data)
{
	int ret;
	union smi330_sensor_conf acc_cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ACCEL, &acc_cfg);
	if (ret)
		return ret;

	acc_cfg.fields.mode = SMI330_MODE_NORMAL;
	acc_cfg.fields.odr = SMI330_ODR_100_HZ;

	return smi330_set_sensor_config_reg(data, SMI330_ACCEL, acc_cfg);
}

static int smi330_trigger_self_test(struct smi330_data *data)
{
	int ret;
	u16 st_trigger = SMI330_CMD_SELF_TEST_TRIGGER;

	/* Pre conditions to be checked */
	ret = smi330_st_precondition(data);
	if (ret)
		return ret;

	/* Disable alternate accel and gyro mode */
	ret = smi330_disable_alt_conf_acc_gyr_mode(data);
	if (ret)
		return ret;

	/* trigger selftest */
	return regmap_write(data->regmap, SMI330_CMD_REG, st_trigger);
}

static int smi330_set_gyro_filter_coefficients(struct smi330_data *data)
{
	int ret;
	s16 data_array[9];

	int gyro_filter_coeff_base_addr =
		SMI330_BASE_ADDR_GYRO_SC_ST_COEFFICIENTS;

	data_array[0] = SMI330_SC_ST_VALUE_0;
	data_array[1] = SMI330_SC_ST_VALUE_1;
	data_array[2] = SMI330_SC_ST_VALUE_2;
	data_array[3] = SMI330_SC_ST_VALUE_3;
	data_array[4] = SMI330_SC_ST_VALUE_4;
	data_array[5] = SMI330_SC_ST_VALUE_5;
	data_array[6] = SMI330_SC_ST_VALUE_6;
	data_array[7] = SMI330_SC_ST_VALUE_7;
	data_array[8] = SMI330_SC_ST_VALUE_8;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   gyro_filter_coeff_base_addr);
	if (ret)
		return ret;

	ret = regmap_bulk_write(data->regmap, SMI330_FEATURE_DATA_TX_REG,
				data_array, 9);
	return ret;
}

static int smi330_set_self_test_mode(struct smi330_data *data)
{
	int ret, reg_data;
	int base_addr = SMI330_BASE_ADDR_ST_SELECT;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   base_addr);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			  &reg_data);
	if (ret)
		return ret;
	reg_data = SMI330_ST_BOTH_ACC_GYR;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   base_addr);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_TX_REG, reg_data);

	return ret;
}

static int smi330_perform_self_test(struct smi330_data *data)
{
	int ret, cfg_restore;
	s16 data_array[9] = { 0 };

	int gyro_filter_coeff_base_addr =
		SMI330_BASE_ADDR_GYRO_SC_ST_COEFFICIENTS;
	union smi330_sensor_conf acc_cfg = { 0 };

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   gyro_filter_coeff_base_addr);
	if (ret == 0)
		ret = regmap_bulk_write(data->regmap,
					SMI330_FEATURE_DATA_TX_REG, data_array,
					9);
	if (ret == 0)
		ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
				   gyro_filter_coeff_base_addr);
	if (ret == 0)
		ret = regmap_bulk_read(data->regmap, SMI330_FEATURE_DATA_TX_REG,
				       data_array, 9);

	if (ret == 0 && (data_array[3] != SMI330_SC_ST_VALUE_3 &&
			 (data_array[0] != SMI330_SC_ST_VALUE_0 ||
			  data_array[1] != SMI330_SC_ST_VALUE_1 ||
			  data_array[2] != SMI330_SC_ST_VALUE_2 ||
			  data_array[4] != SMI330_SC_ST_VALUE_4 ||
			  data_array[5] != SMI330_SC_ST_VALUE_5 ||
			  data_array[6] != SMI330_SC_ST_VALUE_6 ||
			  data_array[7] != SMI330_SC_ST_VALUE_7 ||
			  data_array[8] != SMI330_SC_ST_VALUE_8))) {
		ret = smi330_set_gyro_filter_coefficients(data);
	}

	if (ret == 0)
		/* Set the self-test mode in the self-test dma register. */
		ret = smi330_set_self_test_mode(data);

	/*
	 * Save accel configurations  in separate variable to
	 * restore configuration independent from other errors.
	 */
	cfg_restore =
		smi330_get_sensor_config_reg(data, SMI330_ACCEL, &acc_cfg);
	ret = cfg_restore;

	if (ret == 0)
		/*
		 * Sets the self-test preconditions and triggers
		 * the self-test in the command register.
		 */
		ret = smi330_trigger_self_test(data);

	if (ret == 0)
		ret = smi330_get_st_result(data);

	if (cfg_restore == 0)
		/* Restore accel configurations */
		ret = smi330_set_sensor_config_reg(data, SMI330_ACCEL, acc_cfg);

	return ret;
}

static int self_calib_select(struct smi330_data *data)
{
	int ret;
	const int sc_select_addr = SMI330_GYRO_SC_SELECT;
	const u8 apply_correction = 0x4;
	const u8 sc_both_acc_gyr = 0x3;
	int sc_select = sc_both_acc_gyr | apply_correction;

	ret = regmap_write(data->regmap, SMI330_FEATURE_DATA_ADDR_REG,
			   sc_select_addr);
	if (ret)
		return ret;

	return regmap_write(data->regmap, SMI330_FEATURE_DATA_TX_REG,
			    sc_select);
}

static int self_calib_preconfig(struct smi330_data *data)
{
	int ret;
	union smi330_sensor_conf acc_cfg = { 0 }, temp = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ACCEL, &acc_cfg);
	if (ret)
		return ret;
	memcpy(&temp.value, &acc_cfg.value, sizeof(temp.value));

	acc_cfg.fields.mode = SMI330_MODE_HIGH_PERF;
	acc_cfg.fields.odr = SMI330_ODR_100_HZ;

	ret = smi330_set_sensor_config_reg(data, SMI330_ACCEL, acc_cfg);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_MODE,
				       SMI330_MODE_SUSPEND);
	if (ret) {
		smi330_set_sensor_config_reg(data, SMI330_ACCEL, temp);
		return ret;
	}

	return smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_MODE,
					SMI330_MODE_SUSPEND);
}

static int self_calib_restore_config(struct smi330_data *data,
				     union smi330_sensor_conf accel_config,
				     union smi330_sensor_conf accel_alt_config,
				     union smi330_sensor_conf gyro_alt_config)
{
	int ret;

	ret = smi330_set_sensor_config_reg(data, SMI330_ACCEL, accel_config);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_MODE,
				       accel_alt_config.fields.mode);
	if (ret)
		return ret;

	return smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_MODE,
					gyro_alt_config.fields.mode);
}

static int smi330_self_calibration(struct smi330_data *data,
				   struct iio_dev *indio_dev)
{
	int ret;
	struct device *dev = regmap_get_device(data->regmap);
	int start_calib = SMI330_CMD_SELF_CALIBRATION_TRIG;
	union smi330_io1_data io1_data = { 0 };
	union smi330_sensor_conf acc_cfg = { 0 };
	union smi330_sensor_conf acc_alt_cfg = { 0 };
	union smi330_sensor_conf gyr_alt_cfg = { 0 };

	ret = regmap_read(data->regmap, SMI330_FEATURE_IO1_REG,
			  &io1_data.value);
	if (ret)
		return ret;

	if (io1_data.fields.state != 0)
		return -EBUSY;

	ret = self_calib_select(data);
	if (ret)
		return ret;

	ret = smi330_get_sensor_config_reg(data, SMI330_ACCEL, &acc_cfg);
	if (ret)
		return ret;

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_ACCEL,
					   &acc_alt_cfg);
	if (ret)
		return ret;

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_GYRO, &gyr_alt_cfg);
	if (ret)
		return ret;

	ret = self_calib_preconfig(data);
	if (ret) {
		self_calib_restore_config(data, acc_cfg, acc_alt_cfg,
					  gyr_alt_cfg);
		return ret;
	}

	ret = regmap_write(data->regmap, SMI330_CMD_REG, start_calib);
	if (ret) {
		self_calib_restore_config(data, acc_cfg, acc_alt_cfg,
					  gyr_alt_cfg);
		return ret;
	}

	ret = regmap_read_poll_timeout(data->regmap, SMI330_FEATURE_IO1_REG,
				       io1_data.value,
				       io1_data.fields.sc_st_complete == 1,
				       SMI330_FEAT_ENG_POLL,
				       SMI330_FEAT_ENG_TIMEOUT);

	if (ret) {
		dev_err(dev, "Self calibration not complete");
		self_calib_restore_config(data, acc_cfg, acc_alt_cfg,
					  gyr_alt_cfg);
		return ret;
	}

	return self_calib_restore_config(data, acc_cfg, acc_alt_cfg,
					 gyr_alt_cfg);
}

static int smi330_enable_feature_engine(struct smi330_data *data)
{
	int ret, io1_error_status;
	int feature_ctrl_data = 0x0001, io2_data = SMI330_FEATURE_IO2_VALUE,
	    io2_status_data = SMI330_FEATURE_IO2_STATUS_VALUE;

	ret = regmap_write(data->regmap, SMI330_FEATURE_IO2_REG, io2_data);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, SMI330_FEATURE_IO_STATUS_REG,
			   io2_status_data);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, SMI330_FEATURE_CTRL_REG,
			   feature_ctrl_data);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(data->regmap, SMI330_FEATURE_IO1_REG,
					io1_error_status,
					io1_error_status ==
						SMI330_FEATURE_ENGINE_ENABLE_MASK,
					SMI330_FEAT_ENG_POLL,
					SMI330_FEAT_ENG_TIMEOUT);
}

static int smi330_enable_adv_feat(struct smi330_data *data)
{
	int ret;
	int reset = 0;
	struct smi330_feature_io_status io_status = { 0 };

	// reset the register before updating new values
	ret = regmap_write(data->regmap, SMI330_FEATURE_IO0_REG, reset);
	if (ret)
		return ret;

	// update advanced feature config
	ret = regmap_write(data->regmap, SMI330_FEATURE_IO0_REG,
			   data->cfg.adv_cfg.value);
	if (ret)
		return ret;

	// sync settings
	io_status.feature_io_status = 1;
	ret = regmap_write(data->regmap, SMI330_FEATURE_IO_STATUS_REG,
			   io_status.feature_io_status);
	return ret;
}

static int smi330_set_anymo_cfg(struct smi330_data *data)
{
	s16 anymo[3];

	anymo[0] = data->cfg.anymo_cfg.motion_1.value;
	anymo[1] = data->cfg.anymo_cfg.motion_2.value;
	anymo[2] = data->cfg.anymo_cfg.motion_3.value;

	return smi330_set_regs_dma(SMI330_BASE_ADDR_ANYMO, anymo, 3, data);
}

static int smi330_get_anymo_cfg(struct smi330_data *data)
{
	int ret;
	s16 anymo[3];

	ret = smi330_get_regs_dma(SMI330_BASE_ADDR_ANYMO, anymo, 3, data);
	if (ret)
		return ret;

	data->cfg.anymo_cfg.motion_1.value = anymo[0];
	data->cfg.anymo_cfg.motion_2.value = anymo[1];
	data->cfg.anymo_cfg.motion_3.value = anymo[2];

	return 0;
}

static int smi330_set_nomo_cfg(struct smi330_data *data)
{
	s16 nomo[3];

	nomo[0] = data->cfg.nomo_cfg.motion_1.value;
	nomo[1] = data->cfg.nomo_cfg.motion_2.value;
	nomo[2] = data->cfg.nomo_cfg.motion_3.value;

	return smi330_set_regs_dma(SMI330_BASE_ADDR_NOMO, nomo, 3, data);
}

static int smi330_get_nomo_cfg(struct smi330_data *data)
{
	int ret;
	s16 nomo[3];

	ret = smi330_get_regs_dma(SMI330_BASE_ADDR_NOMO, nomo, 3, data);
	if (ret)
		return ret;

	data->cfg.nomo_cfg.motion_1.value = nomo[0];
	data->cfg.nomo_cfg.motion_2.value = nomo[1];
	data->cfg.nomo_cfg.motion_3.value = nomo[2];

	return 0;
}

static int smi330_set_tilt_cfg(struct smi330_data *data)
{
	s16 tilt[2];

	tilt[0] = data->cfg.tilt_cfg.tilt_1.value;
	tilt[1] = data->cfg.tilt_cfg.tilt_2.value;

	return smi330_set_regs_dma(SMI330_BASE_ADDR_TILT, tilt, 2, data);
}

static int smi330_get_tilt_cfg(struct smi330_data *data)
{
	int ret;
	s16 tilt[2];

	ret = smi330_get_regs_dma(SMI330_BASE_ADDR_TILT, tilt, 2, data);
	if (ret)
		return ret;

	data->cfg.tilt_cfg.tilt_1.value = tilt[0];
	data->cfg.tilt_cfg.tilt_2.value = tilt[1];

	return 0;
}

static int smi330_soft_reset(struct smi330_data *data)
{
	int ret, dummy_byte;
	int soft_reset_cmd = 0xDEAF;

	/* Reset smi330 device */
	ret = regmap_write(data->regmap, SMI330_CMD_REG, soft_reset_cmd);
	if (ret)
		return ret;
	fsleep(SMI330_SOFT_RESET_DELAY);

	/* Performing a dummy read after a soft-reset */
	regmap_read(data->regmap, SMI330_CHIP_ID_REG, &dummy_byte);
	if (ret)
		return ret;

	if (data->cfg.feat_irq != SMI330_INT_DISABLED) {
		ret = smi330_enable_feature_engine(data);
		if (ret)
			return ret;
	}

	return 0;
}

static int smi330_data_ready_handler(struct iio_dev *indio_dev)
{
	int ret, sample, chan;
	int i = 0;
	struct smi330_data *data = iio_priv(indio_dev);

	// Ignore first interrupt due to timestamp issue
	if (data->last_timestamp == 0)
		return 0;

	mutex_lock(&data->lock);

	// Ignore if buffer disabled
	if (!iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&data->lock);
		return 0;
	}

	if (*indio_dev->active_scan_mask == SMI330_ALL_CHAN_MSK) {
		ret = regmap_bulk_read(data->regmap, SMI330_ACCEL_X_REG,
				       data->buf, ARRAY_SIZE(smi330_channels));
	} else {
		for_each_set_bit(chan, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			ret = regmap_read(data->regmap,
					  SMI330_ACCEL_X_REG + chan, &sample);
			if (ret)
				break;
			data->buf[i++] = sample;
		}
	}

	if (ret == 0)
		ret = iio_push_to_buffers_with_timestamp(indio_dev, data->buf,
							 data->current_timestamp);
	mutex_unlock(&data->lock);

	return ret;
}

static int smi330_fifo_handler(struct iio_dev *indio_dev)
{
	int ret, index, chan, i;
	int failure_count = 0;
	struct smi330_data *data = iio_priv(indio_dev);
	s16 *iio_buffer_iter = data->buf;
	s16 *fifo_iter = data->fifo;
	int fifo_frame_length = smi330_get_fifo_frame_length(indio_dev);
	int fifo_length = smi330_get_fifo_length(data);
	int frame_count = fifo_length / fifo_frame_length;
	s64 tsamp = data->cfg.odr_ns;
	s64 timestamp;

	if (data->last_timestamp != 0 && frame_count != 0)
		tsamp = div_s64(data->current_timestamp - data->last_timestamp,
				frame_count);

	mutex_lock(&data->lock);

	ret = regmap_noinc_read(data->regmap, SMI330_FIFO_DATA_REG, data->fifo,
				fifo_length * sizeof(s16));
	if (ret == 0) {
		for (i = 0; i < frame_count; i++) {
			index = 0;
			fifo_iter = &data->fifo[i * fifo_frame_length];

			// Ignore if buffer disabled
			if (!iio_buffer_enabled(indio_dev))
				break;

			for_each_set_bit(chan, indio_dev->active_scan_mask,
					 indio_dev->masklength) {
				iio_buffer_iter[index++] = fifo_iter[chan];
			}
			timestamp = data->current_timestamp -
				    tsamp * (frame_count - i - 1);
			ret = iio_push_to_buffers_with_timestamp(indio_dev,
								 data->buf,
								 timestamp);
			if (ret)
				failure_count++;
		}
	}

	mutex_unlock(&data->lock);

	if (failure_count > 0)
		return -EBUSY;

	return ret;
}

static irqreturn_t smi330_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct smi330_data *data = iio_priv(indio_dev);

	atomic64_set(&data->irq_timestamp, iio_get_time_ns(indio_dev));

	return IRQ_WAKE_THREAD;
}

static int smi330_eval_int_register(struct iio_dev *indio_dev,
				    union smi330_int_status *int_stat)
{
	struct smi330_data *data = iio_priv(indio_dev);

	if (int_stat->fields.int_any_motion) {
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_ROC,
						  IIO_EV_DIR_RISING),
			       data->current_timestamp);
	}

	if (int_stat->fields.int_no_motion) {
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_ROC,
						  IIO_EV_DIR_FALLING),
			       data->current_timestamp);
	}

	if (int_stat->fields.int_tilt) {
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  IIO_MOD_X_OR_Y_OR_Z,
						  IIO_EV_TYPE_CHANGE,
						  IIO_EV_DIR_EITHER),
			       data->current_timestamp);
	}

	if (int_stat->fields.int_acc_drdy || int_stat->fields.int_gyr_drdy ||
	    int_stat->fields.int_temp_drdy)
		smi330_data_ready_handler(indio_dev);

	if (int_stat->fields.int_fwm || int_stat->fields.int_ffull)
		smi330_fifo_handler(indio_dev);

	if (int_stat->fields.int_acc_drdy || int_stat->fields.int_gyr_drdy ||
	    int_stat->fields.int_temp_drdy || int_stat->fields.int_fwm ||
	    int_stat->fields.int_ffull)
		data->last_timestamp = data->current_timestamp;

	return 0;
}

static irqreturn_t smi330_irq_thread_handler(int irq, void *indio_dev_)
{
	int ret;
	s16 int_status[2] = { 0 };
	struct iio_dev *indio_dev = indio_dev_;
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_int_status int_stat_reg[2] = { 0 };

	data->current_timestamp = atomic64_read(&data->irq_timestamp);

	ret = regmap_bulk_read(data->regmap, SMI330_INT1_STATUS_REG, int_status,
			       2);
	if (ret)
		return ret;

	int_stat_reg[0].value = int_status[0];
	int_stat_reg[1].value = int_status[1];

	if (data->cfg.data_irq == SMI330_INT_1 ||
	    data->cfg.feat_irq == SMI330_INT_1)
		smi330_eval_int_register(indio_dev, &int_stat_reg[0]);
	if (data->cfg.data_irq == SMI330_INT_2 ||
	    data->cfg.feat_irq == SMI330_INT_2)
		smi330_eval_int_register(indio_dev, &int_stat_reg[1]);

	return IRQ_HANDLED;
}

static int smi330_set_int_pin_config(struct smi330_data *data,
				     enum smi330_int_out irq_num,
				     bool active_high, bool open_drain,
				     bool latch)
{
	int ret;
	union smi330_io_int_ctrl io_int_ctrl = { 0 };
	union smi330_int_conf int_conf = { 0 };

	ret = regmap_read(data->regmap, SMI330_IO_INT_CTRL_REG,
			  &io_int_ctrl.value);
	if (ret)
		return ret;

	if (irq_num == SMI330_INT_1) {
		io_int_ctrl.fields.int1_level = active_high;
		io_int_ctrl.fields.int1_open_drain = open_drain;
		io_int_ctrl.fields.int1_output_en = true;
	} else if (irq_num == SMI330_INT_2) {
		io_int_ctrl.fields.int2_level = active_high;
		io_int_ctrl.fields.int2_open_drain = open_drain;
		io_int_ctrl.fields.int2_output_en = true;
	} else {
		return -EINVAL;
	}

	ret = regmap_write(data->regmap, SMI330_IO_INT_CTRL_REG,
			   io_int_ctrl.value);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, SMI330_INT_CONF_REG, &int_conf.value);
	if (ret)
		return ret;

	int_conf.fields.int_latch = latch;

	return regmap_write(data->regmap, SMI330_INT_CONF_REG, int_conf.value);
}

static int smi330_setup_irq(struct device *dev, struct iio_dev *indio_dev,
			    int irq, enum smi330_int_out irq_num)
{
	int ret, irq_type;
	bool open_drain, active_high, latch;
	struct smi330_data *data = iio_priv(indio_dev);
	struct irq_data *desc;
	struct fwnode_handle *fwnode;

	desc = irq_get_irq_data(irq);
	if (!desc)
		return -EINVAL;

	irq_type = irqd_get_trigger_type(desc);
	switch (irq_type) {
	case IRQF_TRIGGER_RISING:
		latch = false;
		active_high = true;
		break;
	case IRQF_TRIGGER_HIGH:
		latch = true;
		active_high = true;
		break;
	case IRQF_TRIGGER_FALLING:
		latch = false;
		active_high = false;
		break;
	case IRQF_TRIGGER_LOW:
		latch = true;
		active_high = false;
		break;
	default:
		return -EINVAL;
	}

	fwnode = dev_fwnode(dev);
	if (!fwnode)
		return -ENODEV;

	open_drain = fwnode_property_read_bool(fwnode, "drive-open-drain");

	ret = smi330_set_int_pin_config(data, irq_num, active_high, open_drain,
					latch);
	if (ret)
		return ret;

	return devm_request_threaded_irq(dev, irq, smi330_irq_handler,
					 smi330_irq_thread_handler, irq_type,
					 indio_dev->name, indio_dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
static int fwnode_irq_get_byname(const struct fwnode_handle *fwnode,
				 const char *name)
{
	int index;

	if (!name)
		return -EINVAL;

	index = fwnode_property_match_string(fwnode, "interrupt-names", name);
	if (index < 0)
		return index;

	return fwnode_irq_get(fwnode, index);
}
#endif

static int smi330_register_irq(struct device *dev, struct iio_dev *indio_dev)
{
	int ret, irq;
	struct smi330_data *data = iio_priv(indio_dev);
	struct smi330_cfg *cfg = &data->cfg;
	struct fwnode_handle *fwnode;

	fwnode = dev_fwnode(dev);
	if (!fwnode)
		return -ENODEV;

	irq = fwnode_irq_get_byname(fwnode, "INT1");
	if (irq > 0) {
		ret = smi330_setup_irq(dev, indio_dev, irq, SMI330_INT_1);
		if (ret)
			return ret;
	} else if (cfg->data_irq == SMI330_INT_1 ||
		   cfg->feat_irq == SMI330_INT_1) {
		return -ENODEV;
	}

	irq = fwnode_irq_get_byname(fwnode, "INT2");
	if (irq > 0) {
		ret = smi330_setup_irq(dev, indio_dev, irq, SMI330_INT_2);
		if (ret)
			return ret;
	} else if (cfg->data_irq == SMI330_INT_2 ||
		   cfg->feat_irq == SMI330_INT_2) {
		return -ENODEV;
	}

	return 0;
}

static int smi330_map_interrupt(struct iio_dev *indio_dev, bool enable)
{
	int ret, i;
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_int_map_reg2 int_map2 = { 0 };

	ret = regmap_read(data->regmap, SMI330_INT_MAP2_REG, &int_map2.value);
	if (ret)
		return ret;

	if (data->cfg.op_mode == SMI330_DATA_READY) {
		for_each_set_bit(i, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			switch (i) {
			case SMI330_SCAN_ACCEL_X:
			case SMI330_SCAN_ACCEL_Y:
			case SMI330_SCAN_ACCEL_Z:
				int_map2.fields.acc_drdy_int =
					enable ? data->cfg.data_irq : 0;
				break;
			case SMI330_SCAN_GYRO_X:
			case SMI330_SCAN_GYRO_Y:
			case SMI330_SCAN_GYRO_Z:
				int_map2.fields.gyr_drdy_int =
					enable ? data->cfg.data_irq : 0;
				break;
			case SMI330_TEMP_OBJECT:
				// We don't use temperature data ready irq
				break;
			default:
				break;
			}
		}
	} else if (data->cfg.op_mode == SMI330_FIFO) {
		int_map2.fields.fifo_watermark_int =
			enable ? data->cfg.data_irq : 0;
	}

	return regmap_write(data->regmap, SMI330_INT_MAP2_REG, int_map2.value);
}

static int smi330_hw_fifo_setup(struct iio_dev *indio_dev, bool enable)
{
	int ret, i;
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_fifo_conf config = { 0 };

	ret = regmap_read(data->regmap, SMI330_FIFO_CONF_REG, &config.value);
	if (ret)
		return ret;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch (i) {
		case SMI330_SCAN_ACCEL_X:
		case SMI330_SCAN_ACCEL_Y:
		case SMI330_SCAN_ACCEL_Z:
			config.fields.fifo_acc_en = enable;
			break;
		case SMI330_SCAN_GYRO_X:
		case SMI330_SCAN_GYRO_Y:
		case SMI330_SCAN_GYRO_Z:
			config.fields.fifo_gyr_en = enable;
			break;
		case SMI330_TEMP_OBJECT:
			config.fields.fifo_temp_en = enable;
			break;
		default:
			break;
		}
	}

	return regmap_write(data->regmap, SMI330_FIFO_CONF_REG, config.value);
}

static int smi330_hwfifo_set_watermark(struct iio_dev *indio_dev,
				       unsigned int val)
{
	struct smi330_data *data = iio_priv(indio_dev);

	unsigned int fifo_frame_length =
		smi330_get_fifo_frame_length(indio_dev);
	unsigned int abs_max_wm = (SMI330_FIFO_MAX_LENGTH - 1);
	unsigned int max_wm = abs_max_wm - 3 * max(fifo_frame_length, 3u);
	unsigned int watermark = min(fifo_frame_length * val, max_wm);

	if (val > 1)
		data->cfg.op_mode = SMI330_FIFO;
	else
		data->cfg.op_mode = SMI330_DATA_READY;

	return regmap_write(data->regmap, SMI330_FIFO_WATERMARK_REG, watermark);
}

static int smi330_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;
	enum smi330_mode acc_mode = SMI330_MODE_SUSPEND;
	enum smi330_mode gyr_mode = SMI330_MODE_SUSPEND;
	struct smi330_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	union smi330_sensor_conf cfg = { 0 };

	data->last_timestamp = 0;

	ret = smi330_map_interrupt(indio_dev, true);
	if (ret)
		return ret;

	if (data->cfg.op_mode == SMI330_FIFO) {
		ret = smi330_get_sensor_config_reg(data, SMI330_ACCEL, &cfg);
		if (ret)
			return ret;
		acc_mode = cfg.fields.mode;
		ret = smi330_get_sensor_config_reg(data, SMI330_GYRO, &cfg);
		if (ret)
			return ret;
		gyr_mode = cfg.fields.mode;

		if (acc_mode == SMI330_MODE_LOW_POWER ||
		    gyr_mode == SMI330_MODE_LOW_POWER) {
			dev_err(dev, "Fifo can't be enabled in low power mode");
			smi330_map_interrupt(indio_dev, false);
			return -EINVAL;
		}

		ret = smi330_hw_fifo_setup(indio_dev, true);
	}

	return ret;
}

static int smi330_buffer_predisable(struct iio_dev *indio_dev)
{
	int ret;
	struct smi330_data *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);

	ret = smi330_map_interrupt(indio_dev, false);

	if (data->cfg.op_mode == SMI330_FIFO) {
		ret = smi330_hw_fifo_setup(indio_dev, false);
		if (ret)
			return ret;
	}

	return ret;
}

static int smi330_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct smi330_data *data = iio_priv(indio_dev);

	mutex_unlock(&data->lock);

	return 0;
}

static int smi330_read_odr_reg_value(struct smi330_data *data,
				     enum smi330_odr *odr)
{
	int ret;
	u16 acc_odr, gyr_odr;
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ACCEL, &cfg);
	if (ret)
		return ret;
	acc_odr = cfg.fields.odr;

	ret = smi330_get_sensor_config_reg(data, SMI330_GYRO, &cfg);
	if (ret)
		return ret;
	gyr_odr = cfg.fields.odr;

	if (acc_odr == gyr_odr)
		*odr = cfg.fields.odr;
	else
		return -EINVAL;

	return 0;
}

static int smi330_get_odr_ns(enum smi330_odr odr, int64_t *odr_ns)
{
	switch (odr) {
	case SMI330_ODR_0_78125_HZ:
		*odr_ns = 1280000000;
		break;
	case SMI330_ODR_1_5625_HZ:
		*odr_ns = 640000000;
		break;
	case SMI330_ODR_3_125_HZ:
		*odr_ns = 320000000;
		break;
	case SMI330_ODR_6_25_HZ:
		*odr_ns = 160000000;
		break;
	case SMI330_ODR_12_5_HZ:
		*odr_ns = 80000000;
		break;
	case SMI330_ODR_25_HZ:
		*odr_ns = 40000000;
		break;
	case SMI330_ODR_50_HZ:
		*odr_ns = 20000000;
		break;
	case SMI330_ODR_100_HZ:
		*odr_ns = 10000000;
		break;
	case SMI330_ODR_200_HZ:
		*odr_ns = 5000000;
		break;
	case SMI330_ODR_400_HZ:
		*odr_ns = 2500000;
		break;
	case SMI330_ODR_800_HZ:
		*odr_ns = 1250000;
		break;
	case SMI330_ODR_1600_HZ:
		*odr_ns = 625000;
		break;
	case SMI330_ODR_3200_HZ:
		*odr_ns = 312500;
		break;
	case SMI330_ODR_6400_HZ:
		*odr_ns = 156250;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t alt_odr_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret, acc_odr, gyr_odr, odr_hz;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_ACCEL, &cfg);
	if (ret)
		return ret;
	acc_odr = cfg.fields.odr;

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_GYRO, &cfg);
	if (ret)
		return ret;
	gyr_odr = cfg.fields.odr;

	if (acc_odr == gyr_odr) {
		ret = smi330_reg_to_value(cfg.fields.odr, SMI330_ODR,
					  SMI330_ALL, &odr_hz);
		if (ret)
			return ret;
		return snprintf(buf, PAGE_SIZE, "%d\n", odr_hz);
	}
	return snprintf(buf, PAGE_SIZE,
			"ODR read failed, potential ACC and GYRO mismatch\n");
}

static ssize_t alt_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret, status;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI330_ALT_STATUS_REG, &status);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%x\n", status);
}

static ssize_t alt_acc_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret, mode;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_ACCEL, &cfg);
	if (ret)
		return ret;

	ret = smi330_reg_to_value(cfg.fields.mode, SMI330_MODE,
				  SMI330_ALT_ACCEL, &mode);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", mode);
}

static ssize_t alt_acc_avg_num_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret, avg_num;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_ACCEL, &cfg);
	if (ret)
		return ret;

	ret = smi330_reg_to_value(cfg.fields.avg_num, SMI330_AVG_NUM,
				  SMI330_ALT_ACCEL, &avg_num);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", avg_num);
}

static ssize_t alt_gyr_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret, mode;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_GYRO, &cfg);
	if (ret)
		return ret;

	ret = smi330_reg_to_value(cfg.fields.mode, SMI330_MODE, SMI330_ALT_GYRO,
				  &mode);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", mode);
}

static ssize_t alt_gyr_avg_num_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret, avg_num;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_sensor_conf cfg = { 0 };

	ret = smi330_get_sensor_config_reg(data, SMI330_ALT_GYRO, &cfg);
	if (ret)
		return ret;

	ret = smi330_reg_to_value(cfg.fields.avg_num, SMI330_AVG_NUM,
				  SMI330_ALT_GYRO, &avg_num);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", avg_num);
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret, watermark;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI330_FIFO_WATERMARK_REG, &watermark);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%ld\n",
			watermark & SMI330_FIFO_WATERMARK_MASK);
}

static ssize_t hwfifo_watermark_min_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t hwfifo_watermark_max_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", SMI330_FIFO_MAX_LENGTH - 1);
}

static ssize_t hwfifo_fill_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	int fill_level = smi330_get_fifo_length(data);

	return snprintf(buf, PAGE_SIZE, "%d\n", fill_level);
}

static ssize_t hwfifo_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	if (data->cfg.op_mode == SMI330_FIFO)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t control_auto_op_mode_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 op_mode;

	ret = kstrtou16(buf, 10, &op_mode);
	if (ret)
		return ret;

	if (op_mode != SMI330_AUTO_OP_EN_ACC &&
	    op_mode != SMI330_AUTO_OP_EN_GYR &&
	    op_mode != SMI330_AUTO_OP_EN_ALL &&
	    op_mode != SMI330_AUTO_OP_DISABLE) {
		return -EINVAL;
	}

	ret = smi330_config_auto_operation_mode(data, op_mode);
	if (ret)
		return ret;

	return count;
}

static ssize_t config_user_overwrite_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 user_overwrite;

	ret = kstrtou16(buf, 10, &user_overwrite);
	if (ret)
		return ret;

	if (user_overwrite != SMI330_AUTO_OP_RESET &&
	    user_overwrite != SMI330_AUTO_OP_SET) {
		return -EINVAL;
	}

	ret = smi330_auto_op_cfg_user_overwrite(data, user_overwrite);
	if (ret)
		return ret;

	return count;
}

static ssize_t set_auto_op_mode_cond_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 option;

	ret = kstrtou16(buf, 10, &option);
	if (ret)
		return ret;

	switch (option) {
	case SMI330_AUTO_OP_USER_A:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_USER,
						   A_NO_MOTION);
		break;

	case SMI330_AUTO_OP_USER_B:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_USER,
						   B_ANY_MOTION);
		break;

	case SMI330_AUTO_OP_USER_H:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_USER,
						   H_TILT_DETECTION);
		break;

	case SMI330_AUTO_OP_ALT_A:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_ALT,
						   A_NO_MOTION);
		break;

	case SMI330_AUTO_OP_ALT_B:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_ALT,
						   B_ANY_MOTION);
		break;

	case SMI330_AUTO_OP_ALT_H:
		ret = smi330_set_auto_op_mode_cond(data,
						   SMI330_AUTO_OP_CONFIG_ALT,
						   H_TILT_DETECTION);
		break;

	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	return count;
}

static ssize_t alt_odr_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 odr;

	ret = kstrtou16(buf, 10, &odr);
	if (ret)
		return ret;

	ret = smi330_value_to_reg(odr, SMI330_ODR, SMI330_ALL, (int *)&odr);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_ODR, odr);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_ODR, odr);
	if (ret)
		return ret;

	ret = smi330_get_odr_ns(odr, &data->cfg.odr_ns);
	if (ret)
		return ret;

	return count;
}

static ssize_t alt_acc_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 mode;

	ret = kstrtou16(buf, 10, &mode);
	if (ret)
		return ret;

	ret = smi330_value_to_reg(mode, SMI330_MODE, SMI330_ALT_ACCEL,
				  (int *)&mode);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_MODE,
				       mode);
	if (ret)
		return ret;

	return count;
}

static ssize_t alt_acc_avg_num_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 avg_num;

	ret = kstrtou16(buf, 10, &avg_num);
	if (ret)
		return ret;

	ret = smi330_value_to_reg(avg_num, SMI330_AVG_NUM, SMI330_ALT_ACCEL,
				  (int *)&avg_num);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_ACCEL, SMI330_AVG_NUM,
				       avg_num);
	if (ret)
		return ret;

	return count;
}

static ssize_t alt_gyr_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 mode;

	ret = kstrtou16(buf, 10, &mode);
	if (ret)
		return ret;

	ret = smi330_value_to_reg(mode, SMI330_MODE, SMI330_ALT_GYRO,
				  (int *)&mode);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_MODE,
				       mode);
	if (ret)
		return ret;

	return count;
}

static ssize_t alt_gyr_avg_num_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);
	u16 avg_num;

	ret = kstrtou16(buf, 10, &avg_num);
	if (ret)
		return ret;

	ret = smi330_value_to_reg(avg_num, SMI330_AVG_NUM, SMI330_ALT_GYRO,
				  (int *)&avg_num);
	if (ret)
		return ret;

	ret = smi330_set_sensor_config(data, SMI330_ALT_GYRO, SMI330_AVG_NUM,
				       avg_num);
	if (ret)
		return ret;

	return count;
}

static ssize_t self_test_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	ret = smi330_perform_self_test(data);
	if (ret)
		return ret;

	return count;
}

static ssize_t self_cal_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	ret = smi330_self_calibration(data, indio_dev);
	if (ret)
		return ret;

	return count;
}

static ssize_t soft_reset_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi330_data *data = iio_priv(indio_dev);

	ret = smi330_soft_reset(data);
	if (ret)
		return ret;

	ret = smi330_dev_init(data);
	if (ret)
		return ret;

	return count;
}

static int smi330_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct smi330_data *data = iio_priv(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				return data->cfg.adv_cfg.fields.any_motion_x_en;
			case IIO_MOD_Y:
				return data->cfg.adv_cfg.fields.any_motion_y_en;
			case IIO_MOD_Z:
				return data->cfg.adv_cfg.fields.any_motion_z_en;
			default:
				return 0;
			}
		}
		if (dir == IIO_EV_DIR_FALLING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				return data->cfg.adv_cfg.fields.no_motion_x_en;
			case IIO_MOD_Y:
				return data->cfg.adv_cfg.fields.no_motion_y_en;
			case IIO_MOD_Z:
				return data->cfg.adv_cfg.fields.no_motion_z_en;
			default:
				return 0;
			}
		}
		return 0;
	case IIO_EV_TYPE_CHANGE:
		if (dir == IIO_EV_DIR_EITHER)
			return data->cfg.adv_cfg.fields.tilt_en;
		return 0;
	default:
		return 0;
	}
}

static int smi330_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir, int state)
{
	int ret;
	struct smi330_data *data = iio_priv(indio_dev);
	union smi330_int_map_reg1 int_map_reg1 = { 0 };

	ret = regmap_read(data->regmap, SMI330_INT_MAP1_REG,
			  &int_map_reg1.value);
	if (ret)
		return ret;

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				data->cfg.adv_cfg.fields.any_motion_x_en =
					state;
				break;
			case IIO_MOD_Y:
				data->cfg.adv_cfg.fields.any_motion_y_en =
					state;
				break;
			case IIO_MOD_Z:
				data->cfg.adv_cfg.fields.any_motion_z_en =
					state;
				break;
			default:
				break;
			}
			int_map_reg1.fields.any_motion_out = data->cfg.feat_irq;
		} else if (dir == IIO_EV_DIR_FALLING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				data->cfg.adv_cfg.fields.no_motion_x_en = state;
				break;
			case IIO_MOD_Y:
				data->cfg.adv_cfg.fields.no_motion_y_en = state;
				break;
			case IIO_MOD_Z:
				data->cfg.adv_cfg.fields.no_motion_z_en = state;
				break;
			default:
				break;
			}
			int_map_reg1.fields.no_motion_out = data->cfg.feat_irq;
		}
		break;
	case IIO_EV_TYPE_CHANGE:
		if (dir == IIO_EV_DIR_EITHER) {
			data->cfg.adv_cfg.fields.tilt_en = state;
			int_map_reg1.fields.tilt_out = data->cfg.feat_irq;
		}
		break;
	default:
		break;
	}

	ret = smi330_enable_adv_feat(data);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, SMI330_INT_MAP1_REG,
			   int_map_reg1.value);
	return ret;
}

static int smi330_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	int ret;
	struct smi330_data *data = iio_priv(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			ret = smi330_get_anymo_cfg(data);
			if (ret)
				return ret;
			switch (info) {
			case IIO_EV_INFO_PERIOD:
				*val = data->cfg.anymo_cfg.motion_3.fields
					       .duration;
				break;
			case IIO_EV_INFO_VALUE:
				*val = data->cfg.anymo_cfg.motion_1.fields
					       .slope_thres;
				break;
			case IIO_EV_INFO_HYSTERESIS:
				*val = data->cfg.anymo_cfg.motion_2.fields
					       .hysteresis;
				break;
			case IIO_EV_INFO_TIMEOUT:
				*val = data->cfg.anymo_cfg.motion_3.fields
					       .wait_time;
				break;
			default:
				break;
			}
		}

		else if (dir == IIO_EV_DIR_FALLING) {
			ret = smi330_get_nomo_cfg(data);
			if (ret)
				return ret;
			switch (info) {
			case IIO_EV_INFO_PERIOD:
				*val = data->cfg.nomo_cfg.motion_3.fields
					       .duration;
				break;
			case IIO_EV_INFO_VALUE:
				*val = data->cfg.nomo_cfg.motion_1.fields
					       .slope_thres;
				break;
			case IIO_EV_INFO_HYSTERESIS:
				*val = data->cfg.nomo_cfg.motion_2.fields
					       .hysteresis;
				break;
			case IIO_EV_INFO_TIMEOUT:
				*val = data->cfg.nomo_cfg.motion_3.fields
					       .wait_time;
				break;
			default:
				break;
			}
		}
		break;
	case IIO_EV_TYPE_CHANGE:
		if (dir == IIO_EV_DIR_EITHER) {
			ret = smi330_get_tilt_cfg(data);
			if (ret)
				return ret;

			if (info == IIO_EV_INFO_VALUE)
				*val = data->cfg.tilt_cfg.tilt_1.fields
					       .min_tilt_angle;
			if (info == IIO_EV_INFO_PERIOD)
				*val = data->cfg.tilt_cfg.tilt_1.fields
					       .segment_size;
			if (info == IIO_EV_INFO_LOW_PASS_FILTER_3DB)
				*val = data->cfg.tilt_cfg.tilt_2.fields
					       .beta_acc_mean;
		}
		break;
	default:
		break;
	}

	return IIO_VAL_INT;
}

static int smi330_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	int ret = 0;
	struct smi330_data *data = iio_priv(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			ret = smi330_get_anymo_cfg(data);
			if (ret)
				return ret;
			switch (info) {
			case IIO_EV_INFO_PERIOD:
				data->cfg.anymo_cfg.motion_3.fields.duration =
					val;
				break;
			case IIO_EV_INFO_VALUE:
				data->cfg.anymo_cfg.motion_1.fields.slope_thres =
					val;
				break;
			case IIO_EV_INFO_HYSTERESIS:
				data->cfg.anymo_cfg.motion_2.fields.hysteresis =
					val;
				break;
			case IIO_EV_INFO_TIMEOUT:
				data->cfg.anymo_cfg.motion_3.fields.wait_time =
					val;
				break;
			default:
				break;
			}

			ret = smi330_set_anymo_cfg(data);
		} else if (dir == IIO_EV_DIR_FALLING) {
			ret = smi330_get_nomo_cfg(data);
			if (ret)
				return ret;
			switch (info) {
			case IIO_EV_INFO_PERIOD:
				data->cfg.nomo_cfg.motion_3.fields.duration =
					val;
				break;
			case IIO_EV_INFO_VALUE:
				data->cfg.nomo_cfg.motion_1.fields.slope_thres =
					val;
				break;
			case IIO_EV_INFO_HYSTERESIS:
				data->cfg.nomo_cfg.motion_2.fields.hysteresis =
					val;
				break;
			case IIO_EV_INFO_TIMEOUT:
				data->cfg.nomo_cfg.motion_3.fields.wait_time =
					val;
				break;
			default:
				break;
			}

			ret = smi330_set_nomo_cfg(data);
		}
		return ret;
	case IIO_EV_TYPE_CHANGE:
		if (dir == IIO_EV_DIR_EITHER) {
			ret = smi330_get_tilt_cfg(data);
			if (ret)
				return ret;
			if (info == IIO_EV_INFO_VALUE)
				data->cfg.tilt_cfg.tilt_1.fields.min_tilt_angle =
					val;
			if (info == IIO_EV_INFO_PERIOD)
				data->cfg.tilt_cfg.tilt_1.fields.segment_size =
					val;
			if (info == IIO_EV_INFO_LOW_PASS_FILTER_3DB)
				data->cfg.tilt_cfg.tilt_2.fields.beta_acc_mean =
					val;

			ret = smi330_set_tilt_cfg(data);
		}
		return ret;
	default:
		return 0;
	}
}

static IIO_DEVICE_ATTR_WO(control_auto_op_mode, 0);
static IIO_DEVICE_ATTR_WO(set_auto_op_mode_cond, 0);
static IIO_DEVICE_ATTR_WO(config_user_overwrite, 0);

static IIO_DEVICE_ATTR_WO(self_test, 0);
static IIO_DEVICE_ATTR_WO(self_cal, 0);
static IIO_DEVICE_ATTR_WO(soft_reset, 0);

static IIO_DEVICE_ATTR_RW(alt_acc_mode, 0);
static IIO_DEVICE_ATTR_RW(alt_acc_avg_num, 0);
static IIO_DEVICE_ATTR_RW(alt_gyr_mode, 0);
static IIO_DEVICE_ATTR_RW(alt_gyr_avg_num, 0);
static IIO_DEVICE_ATTR_RW(alt_odr, 0);
static IIO_DEVICE_ATTR_RO(alt_status, 0);

static struct attribute *smi330_event_attributes[] = {
	&iio_dev_attr_control_auto_op_mode.dev_attr.attr,
	&iio_dev_attr_set_auto_op_mode_cond.dev_attr.attr,
	&iio_dev_attr_config_user_overwrite.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_self_cal.dev_attr.attr,
	&iio_dev_attr_soft_reset.dev_attr.attr,
	&iio_dev_attr_alt_acc_mode.dev_attr.attr,
	&iio_dev_attr_alt_acc_avg_num.dev_attr.attr,
	&iio_dev_attr_alt_gyr_mode.dev_attr.attr,
	&iio_dev_attr_alt_gyr_avg_num.dev_attr.attr,
	&iio_dev_attr_alt_odr.dev_attr.attr,
	&iio_dev_attr_alt_status.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi330_event_attribute_group = {
	.attrs = smi330_event_attributes,
};

static IIO_DEVICE_ATTR_RO(hwfifo_watermark_min, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark_max, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_enabled, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_fill_level, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
static const struct attribute *smi330_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_fill_level.dev_attr.attr,
	NULL,
};
#else
static const struct iio_dev_attr *smi330_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_min, &iio_dev_attr_hwfifo_watermark_max,
	&iio_dev_attr_hwfifo_watermark,	    &iio_dev_attr_hwfifo_enabled,
	&iio_dev_attr_hwfifo_fill_level,    NULL,
};
#endif

const struct iio_buffer_setup_ops smi330_buffer_ops = {
	.postenable = smi330_buffer_postenable,
	.predisable = smi330_buffer_predisable,
	.postdisable = smi330_buffer_postdisable,
};

static struct iio_info smi330_info = {
	.read_event_config = smi330_read_event_config,
	.read_event_value = smi330_read_event_value,
	.write_event_config = smi330_write_event_config,
	.write_event_value = smi330_write_event_value,
	.read_avail = smi330_read_avail,
	.read_raw = smi330_read_raw,
	.write_raw = smi330_write_raw,
	.write_raw_get_fmt = smi330_write_raw_get_fmt,
	.hwfifo_set_watermark = smi330_hwfifo_set_watermark,
};

static int smi330_dev_init(struct smi330_data *data)
{
	int ret, chip_id;
	union smi330_error_reg error_reg = { 0 };
	union smi330_status_reg status_reg = { 0 };
	enum smi330_odr odr;
	struct device *dev = regmap_get_device(data->regmap);
	struct smi330_cfg *cfg = &data->cfg;

	ret = regmap_read(data->regmap, SMI330_CHIP_ID_REG, &chip_id);
	if (ret)
		return ret;

	chip_id &= 0x00FF;

	if (chip_id != SMI330_CHIP_ID)
		dev_info(dev, "Unknown chip id: 0x%04x\n", chip_id);

	ret = regmap_read(data->regmap, SMI330_ERR_REG, &error_reg.value);
	if (ret || error_reg.fields.fatal_err)
		return -ENODEV;

	ret = regmap_read(data->regmap, SMI330_STATUS_REG, &status_reg.value);
	if (ret || status_reg.fields.por_detected == 0)
		return -ENODEV;

	ret = smi330_read_odr_reg_value(data, &odr);
	if (ret)
		return ret;

	ret = smi330_get_odr_ns(odr, &cfg->odr_ns);
	return ret;
}

int smi330_core_probe(struct device *dev, struct regmap *regmap)
{
	int ret;
	struct iio_dev *indio_dev;
	struct smi330_data *data;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;
	mutex_init(&data->lock);

	if (IS_ENABLED(CONFIG_SMI330_IRQ_DATA_INT1))
		data->cfg.data_irq = SMI330_INT_1;
	else if (IS_ENABLED(CONFIG_SMI330_IRQ_DATA_INT2))
		data->cfg.data_irq = SMI330_INT_2;
	else
		data->cfg.data_irq = SMI330_INT_DISABLED;

	if (IS_ENABLED(CONFIG_SMI330_IRQ_ADV_FEAT_INT1))
		data->cfg.feat_irq = SMI330_INT_1;
	else if (IS_ENABLED(CONFIG_SMI330_IRQ_ADV_FEAT_INT2))
		data->cfg.feat_irq = SMI330_INT_2;
	else
		data->cfg.feat_irq = SMI330_INT_DISABLED;

	ret = smi330_soft_reset(data);
	if (ret)
		return dev_err_probe(dev, ret, "Soft reset failed\n");

	indio_dev->channels = smi330_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi330_channels);
	indio_dev->name = "smi330";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &smi330_info;

	ret = smi330_dev_init(data);
	if (ret)
		return dev_err_probe(dev, ret, "Init failed\n");

	ret = smi330_register_irq(dev, indio_dev);
	if (ret)
		return dev_err_probe(
			dev, ret,
			"Register IRQ failed - check Kconfig and devicetree\n");

	if (data->cfg.data_irq == SMI330_INT_DISABLED) {
		data->cfg.op_mode = SMI330_POLLING;
	} else {
		data->cfg.op_mode = SMI330_DATA_READY;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
		ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev,
						      INDIO_BUFFER_SOFTWARE,
						      &smi330_buffer_ops,
						      smi330_fifo_attributes);
#else
		ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev,
						      &smi330_buffer_ops,
						      smi330_fifo_attributes);
#endif
		if (ret)
			return dev_err_probe(dev, ret,
					     "IIO buffer setup failed\n");
	}

	if (data->cfg.feat_irq != SMI330_INT_DISABLED)
		smi330_info.event_attrs = &smi330_event_attribute_group;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Register IIO device failed\n");

	return 0;
}
EXPORT_SYMBOL_NS_GPL(smi330_core_probe, IIO_SMI330);

MODULE_AUTHOR("Roman Huber <roman.huber@de.bosch.com>");
MODULE_AUTHOR("Stefan Gutmann <stefan.gutmann@de.bosch.com>");
MODULE_DESCRIPTION("Bosch SMI330 driver");
MODULE_LICENSE("Dual BSD/GPL");
