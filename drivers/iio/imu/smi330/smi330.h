/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
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
#ifndef _SMI330_H
#define _SMI330_H

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define SMI330_NO_ERROR_MASK (BIT(2) | BIT(0))
#define SMI330_ST_SUCCESS_MASK GENMASK(6, 0)

#define SMI330_ALL_CHAN_MSK GENMASK(6, 0)

#define SMI330_FIFO_SIZE U16_C(2048)
#define SMI330_FIFO_MAX_LENGTH U16_C(1024)
#define SMI330_FIFO_MAX_FRAME_LENGTH U16_C(8)

#define SMI330_CHIP_ID U16_C(0x0042)
#define SMI330_SPI_WR_MASK U16_C(0x7F)
#define SMI330_SPI_RD_MASK U16_C(0x80)

#define SMI330_GYRO_SC_SELECT U8_C(0x26)
#define SMI330_CMD_SELF_CALIBRATION_TRIG U16_C(0x0101)
#define SMI330_SOFT_RESET_DELAY U16_C(2000)
#define SMI330_FEAT_ENG_POLL U16_C(10000)
#define SMI330_FEAT_ENG_TIMEOUT U16_C(1000000)

#define SMI330_CHIP_ID_REG 0x00
#define SMI330_ERR_REG 0x01
#define SMI330_STATUS_REG 0x02
#define SMI330_ACCEL_X_REG 0x03
#define SMI330_GYRO_X_REG 0x06
#define SMI330_TEMP_REG 0x09
#define SMI330_INT1_STATUS_REG 0x0D
#define SMI330_FEATURE_IO0_REG 0x10
#define SMI330_FEATURE_IO1_REG 0x11
#define SMI330_FEATURE_IO2_REG 0x12
#define SMI330_FEATURE_IO_STATUS_REG 0x14
#define SMI330_FIFO_FILL_LEVEL_REG 0x15
#define SMI330_FIFO_DATA_REG 0x16
#define SMI330_ACCEL_CFG_REG 0x20
#define SMI330_GYRO_CFG_REG 0x21
#define SMI330_ALT_CONF_CHG_REG 0x23
#define SMI330_ALT_ACCEL_CFG_REG 0x28
#define SMI330_ALT_GYRO_CFG_REG 0x29
#define SMI330_ALT_CONF_REG 0x2A
#define SMI330_ALT_STATUS_REG 0x2B
#define SMI330_FIFO_WATERMARK_REG 0x35
#define SMI330_FIFO_CONF_REG 0x36
#define SMI330_FIFO_CTRL_REG 0x37
#define SMI330_IO_INT_CTRL_REG 0x38
#define SMI330_INT_CONF_REG 0x39
#define SMI330_INT_MAP1_REG 0x3A
#define SMI330_INT_MAP2_REG 0x3B
#define SMI330_FEATURE_CTRL_REG 0x40
#define SMI330_FEATURE_DATA_ADDR_REG 0x41
#define SMI330_FEATURE_DATA_TX_REG 0x42
#define SMI330_FEATURE_DATA_STATUS_REG 0x43
#define SMI330_CMD_REG 0x7E
#define SMI330_RES_CFG_REG 0x7F

#define SMI330_FIFO_FILL_LEVEL_MASK GENMASK(10, 0)
#define SMI330_FIFO_WATERMARK_MASK GENMASK(9, 0)
#define SMI330_FEATURE_ENGINE_ENABLE_MASK BIT(0)

#define SMI330_FEATURE_IO2_VALUE U16_C(0x012C)
#define SMI330_FEATURE_IO2_STATUS_VALUE U16_C(0x0001)

#define SMI330_BASE_ADDR_GYRO_SC_ST_COEFFICIENTS U8_C(0x28)
#define SMI330_SC_ST_VALUE_0 U16_C(0x5A2E)
#define SMI330_SC_ST_VALUE_1 U16_C(0x9219)
#define SMI330_SC_ST_VALUE_2 U16_C(0x5637)
#define SMI330_SC_ST_VALUE_3 U16_C(0xFFE8)
#define SMI330_SC_ST_VALUE_4 U16_C(0xFFEF)
#define SMI330_SC_ST_VALUE_5 U16_C(0x000D)
#define SMI330_SC_ST_VALUE_6 U16_C(0x07CA)
#define SMI330_SC_ST_VALUE_7 U16_C(0xFFCD)
#define SMI330_SC_ST_VALUE_8 U16_C(0xEF6C)

#define SMI330_CMD_SELF_TEST_TRIGGER U16_C(0x0100)
#define SMI330_ST_BOTH_ACC_GYR U8_C(3)

/* Extended map registers */
#define SMI330_BASE_ADDR_ST_SELECT U8_C(0x25)
#define SMI330_BASE_ADDR_ST_RESULT U8_C(0x24)
#define SMI330_BASE_ADDR_ANYMO U8_C(0x05)
#define SMI330_BASE_ADDR_NOMO U8_C(0x08)
#define SMI330_BASE_ADDR_TILT U8_C(0x21)

/* T°C = (temp / 512) + 23 */
#define SMI330_TEMP_OFFSET 11776 // 23 * 512
#define SMI330_TEMP_SCALE 1953125 // (1 / 512) * 1e9

enum smi330_accel_range {
	SMI330_ACCEL_RANGE_2G = U8_C(0x00),
	SMI330_ACCEL_RANGE_4G = U8_C(0x01),
	SMI330_ACCEL_RANGE_8G = U8_C(0x02),
	SMI330_ACCEL_RANGE_16G = U8_C(0x03)
};

enum smi330_gyro_range {
	SMI330_GYRO_RANGE_125 = U8_C(0x00),
	SMI330_GYRO_RANGE_250 = U8_C(0x01),
	SMI330_GYRO_RANGE_500 = U8_C(0x02)
};

enum smi330_odr {
	SMI330_ODR_0_78125_HZ = U8_C(0x01),
	SMI330_ODR_1_5625_HZ = U8_C(0x02),
	SMI330_ODR_3_125_HZ = U8_C(0x03),
	SMI330_ODR_6_25_HZ = U8_C(0x04),
	SMI330_ODR_12_5_HZ = U8_C(0x05),
	SMI330_ODR_25_HZ = U8_C(0x06),
	SMI330_ODR_50_HZ = U8_C(0x07),
	SMI330_ODR_100_HZ = U8_C(0x08),
	SMI330_ODR_200_HZ = U8_C(0x09),
	SMI330_ODR_400_HZ = U8_C(0x0A),
	SMI330_ODR_800_HZ = U8_C(0x0B),
	SMI330_ODR_1600_HZ = U8_C(0x0C),
	SMI330_ODR_3200_HZ = U8_C(0x0D),
	SMI330_ODR_6400_HZ = U8_C(0x0E)
};

enum smi330_avg_num {
	SMI330_AVG_NUM_1 = U8_C(0x00),
	SMI330_AVG_NUM_2 = U8_C(0x01),
	SMI330_AVG_NUM_4 = U8_C(0x02),
	SMI330_AVG_NUM_8 = U8_C(0x03),
	SMI330_AVG_NUM_16 = U8_C(0x04),
	SMI330_AVG_NUM_32 = U8_C(0x05),
	SMI330_AVG_NUM_64 = U8_C(0x06)
};

enum smi330_mode {
	SMI330_MODE_SUSPEND = U8_C(0x00),
	SMI330_MODE_GYRO_DRIVE = U8_C(0x01),
	SMI330_MODE_LOW_POWER = U8_C(0x03),
	SMI330_MODE_NORMAL = U8_C(0x04),
	SMI330_MODE_HIGH_PERF = U8_C(0x07)
};

enum smi330_bw {
	SMI330_BW_2 = U8_C(0x00), /*! ODR/2 */
	SMI330_BW_4 = U8_C(0x01) /*! ODR/4 */
};

enum smi330_auto_op_adv_feat {
	A_NO_MOTION = U8_C(0x01),
	B_ANY_MOTION = U8_C(0x02),
	H_TILT_DETECTION = U8_C(0x08)
};

enum smi330_auto_op_use {
	SMI330_AUTO_OP_USER_A,
	SMI330_AUTO_OP_USER_B,
	SMI330_AUTO_OP_USER_H,
	SMI330_AUTO_OP_ALT_A,
	SMI330_AUTO_OP_ALT_B,
	SMI330_AUTO_OP_ALT_H,
};

enum {
	SMI330_SCAN_ACCEL_X,
	SMI330_SCAN_ACCEL_Y,
	SMI330_SCAN_ACCEL_Z,
	SMI330_SCAN_GYRO_X,
	SMI330_SCAN_GYRO_Y,
	SMI330_SCAN_GYRO_Z,
	SMI330_TEMP_OBJECT,
	SMI330_SCAN_TIMESTAMP,
};

enum smi330_operation_mode {
	SMI330_POLLING,
	SMI330_DATA_READY,
	SMI330_FIFO,
};

enum smi330_auto_op_setup {
	SMI330_AUTO_OP_RESET,
	SMI330_AUTO_OP_SET,
};

enum smi330_auto_op_mode {
	SMI330_AUTO_OP_EN_ACC,
	SMI330_AUTO_OP_EN_GYR,
	SMI330_AUTO_OP_EN_ALL,
	SMI330_AUTO_OP_DISABLE,
};

enum smi330_auto_op_config {
	SMI330_AUTO_OP_CONFIG_USER,
	SMI330_AUTO_OP_CONFIG_ALT,
};

enum smi330_sensor {
	SMI330_ALL,
	SMI330_ACCEL,
	SMI330_GYRO,
	SMI330_ALT_ACCEL,
	SMI330_ALT_GYRO,
};

enum smi330_sensor_conf_select {
	SMI330_ODR,
	SMI330_RANGE,
	SMI330_BW,
	SMI330_AVG_NUM,
	SMI330_MODE,
};

union smi330_sensor_conf {
	struct {
		uint16_t odr : 4; /*! Output data rate */
		uint16_t range : 3; /*! Full scale range */
		uint16_t bw : 1; /*! Bandwidth */
		uint16_t avg_num : 3; /*! Averaging number */
		uint16_t reserved_0 : 1; /*! Reserved */
		uint16_t mode : 3; /*! Mode */
		uint16_t reserved_1 : 1; /*! Reserved */
	} fields;
	int value;
};

union smi330_error_reg {
	struct {
		uint16_t fatal_err : 1;
		uint16_t reserved_0 : 1;
		uint16_t feat_eng_ovrld : 1;
		uint16_t reserved_1 : 1;
		uint16_t feat_eng_wd : 1;
		uint16_t acc_conf_err : 1;
		uint16_t gyr_conf_err : 1;
		uint16_t reserved_2 : 1;
		uint16_t i3c_error0 : 1;
		uint16_t reserved_3 : 2;
		uint16_t i3c_error3 : 1;
		uint16_t reserved_4 : 4;
	} fields;
	int value;
};

union smi330_status_reg {
	struct {
		uint16_t por_detected : 1;
		uint16_t reserved_0 : 4;
		uint16_t drdy_temp : 1;
		uint16_t drdy_gyr : 1;
		uint16_t drdy_acc : 1;
		uint16_t reserved_1 : 8;
	} fields;
	int value;
};

union smi330_int_map_reg2 {
	struct {
		uint16_t tap_out : 2;
		uint16_t i3c_out : 2;
		uint16_t err_status : 2;
		uint16_t temp_drdy_int : 2;
		uint16_t gyr_drdy_int : 2;
		uint16_t acc_drdy_int : 2;
		uint16_t fifo_watermark_int : 2;
		uint16_t fifo_full_int : 2;
	} fields;
	int value;
};

union smi330_io_int_ctrl {
	struct {
		uint16_t int1_level : 1;
		uint16_t int1_open_drain : 1;
		uint16_t int1_output_en : 1;
		uint16_t reserved_0 : 5;
		uint16_t int2_level : 1;
		uint16_t int2_open_drain : 1;
		uint16_t int2_output_en : 1;
		uint16_t reserved_1 : 5;
	} fields;
	int value;
};

union smi330_int_conf {
	struct {
		uint16_t int_latch : 1;
		uint16_t reserved_0 : 15;
	} fields;
	int value;
};

union smi330_int_status {
	struct {
		uint16_t int_no_motion : 1;
		uint16_t int_any_motion : 1;
		uint16_t int_flat : 1;
		uint16_t int_orientation : 1;
		uint16_t int_step_detector : 1;
		uint16_t int_step_counter : 1;
		uint16_t int_sig_motion : 1;
		uint16_t int_tilt : 1;
		uint16_t int_tap : 1;
		uint16_t int_i3c : 1;
		uint16_t int_err_status : 1;
		uint16_t int_temp_drdy : 1;
		uint16_t int_gyr_drdy : 1;
		uint16_t int_acc_drdy : 1;
		uint16_t int_fwm : 1;
		uint16_t int_ffull : 1;
	} fields;
	int value;
};

union smi330_fifo_conf {
	struct {
		uint16_t fifo_stop_on_full : 1;
		uint16_t reserved_0 : 7;
		uint16_t fifo_time_en : 1;
		uint16_t fifo_acc_en : 1;
		uint16_t fifo_gyr_en : 1;
		uint16_t fifo_temp_en : 1;
		uint16_t reserved_1 : 4;
	} fields;
	int value;
};

enum smi330_int_out {
	SMI330_INT_DISABLED = 0x00,
	SMI330_INT_1 = 0x01,
	SMI330_INT_2 = 0x02,
	SMI330_INT_I3C_IBI = 0x03,
};

union smi330_int_map_reg1 {
	struct {
		uint16_t no_motion_out : 2;
		uint16_t any_motion_out : 2;
		uint16_t flat_out : 2;
		uint16_t orientation_out : 2;
		uint16_t step_detector_out : 2;
		uint16_t step_counter_out : 2;
		uint16_t sig_motion_out : 2;
		uint16_t tilt_out : 2;
	} fields;
	int value;
};

struct extended_gen_set {
	uint16_t reserved : 10;
	uint16_t sw_lock : 1;
	uint16_t int_hold_dur : 4;
	uint16_t event_report_mode : 1;
};

struct smi330_feature_io_status {
	uint16_t feature_io_status : 1;
	uint16_t reserved : 15;
};

union smi330_tilt_1 {
	struct {
		uint16_t segment_size : 8;
		uint16_t min_tilt_angle : 8;
	} fields;
	int value;
};

union smi330_tilt_2 {
	struct {
		uint16_t beta_acc_mean : 16;
	} fields;
	int value;
};

union smi330_motion_1 {
	struct {
		uint16_t slope_thres : 12;
		uint16_t acc_ref_up : 1;
		uint16_t reserved : 3;
	} fields;
	int value;
};

union smi330_motion_2 {
	struct {
		uint16_t hysteresis : 10;
		uint16_t reserved : 6;
	} fields;
	int value;
};

union smi330_motion_3 {
	struct {
		uint16_t duration : 13;
		uint16_t wait_time : 3;
	} fields;
	int value;
};

struct smi330_motion_cfg {
	union smi330_motion_1 motion_1;
	union smi330_motion_2 motion_2;
	union smi330_motion_3 motion_3;
};

struct smi330_tilt_cfg {
	union smi330_tilt_1 tilt_1;
	union smi330_tilt_2 tilt_2;
};

union smi330_feature_data_status {
	struct {
		uint16_t data_outofbound_err : 1;
		uint16_t data_tx_ready : 1;
		uint16_t reserved : 14;
	} fields;
	int value;
};

union smi330_feature_io0_cfg {
	struct {
		uint16_t no_motion_x_en : 1;
		uint16_t no_motion_y_en : 1;
		uint16_t no_motion_z_en : 1;
		uint16_t any_motion_x_en : 1;
		uint16_t any_motion_y_en : 1;
		uint16_t any_motion_z_en : 1;
		uint16_t flat_en : 1;
		uint16_t orientation_en : 1;
		uint16_t step_detector_en : 1;
		uint16_t step_counter_en : 1;
		uint16_t sig_motion_en : 1;
		uint16_t tilt_en : 1;
		uint16_t tap_detector_s_tap_en : 1;
		uint16_t tap_detector_d_tap_en : 1;
		uint16_t tap_detector_t_tap_en : 1;
		uint16_t i3c_sync_en : 1;
	} fields;
	int value;
};

union smi330_st_result {
	struct {
		/*! Bit is set to 1 when accelerometer X-axis test passed */
		uint16_t acc_sens_x_ok : 1;
		/*! Bit is set to 1 when accelerometer y-axis test passed */
		uint16_t acc_sens_y_ok : 1;
		/*! Bit is set to 1 when accelerometer z-axis test passed */
		uint16_t acc_sens_z_ok : 1;
		/*! Bit is set to 1 when gyroscope X-axis test passed */
		uint16_t gyr_sens_x_ok : 1;
		/*! Bit is set to 1 when gyroscope y-axis test passed */
		uint16_t gyr_sens_y_ok : 1;
		/*! Bit is set to 1 when gyroscope z-axis test passed */
		uint16_t gyr_sens_z_ok : 1;
		/*! Bit is set to 1 when gyroscope drive test passed */
		uint16_t gyr_drive_ok : 1;
		uint16_t reserved : 9;
	} fields;
	int value;
};

union smi330_io1_data {
	struct {
		uint16_t error_status : 4;
		uint16_t sc_st_complete : 1;
		uint16_t gyro_st_result : 1;
		uint16_t st_result : 1;
		uint16_t sample_rate_err : 1;
		uint16_t reserved0 : 2;
		uint16_t axis_map_complete : 1;
		uint16_t state : 2;
		uint16_t reserved1 : 3;
	} fields;
	int value;
};

union smi330_alt_conf {
	struct {
		uint16_t alt_acc_en : 1;
		uint16_t reserved0 : 3;
		uint16_t alt_gyr_en : 1;
		uint16_t reserved1 : 3;
		uint16_t alt_rst_conf_write_en : 1;
		uint16_t reserved2 : 7;
	} fields;
	int value;
};

union smi330_alt_config_chg {
	struct {
		uint16_t alt_conf_alt_switch_src_select : 4;
		uint16_t alt_conf_user_switch_src_select : 4;
		uint16_t reserved : 8;
	} fields;
	int value;
};

struct smi330_sysfs_attr {
	int *reg_vals;
	int *vals;
	int len;
	int type;
};

struct smi330_cfg {
	s64 odr_ns;
	enum smi330_operation_mode op_mode;
	enum smi330_int_out data_irq;
	enum smi330_int_out feat_irq;
	union smi330_feature_io0_cfg adv_cfg;
	struct smi330_motion_cfg anymo_cfg;
	struct smi330_motion_cfg nomo_cfg;
	struct smi330_tilt_cfg tilt_cfg;
};

struct smi330_data {
	struct regmap *regmap;
	struct smi330_cfg cfg;
	s64 current_timestamp;
	s64 last_timestamp;
	atomic64_t irq_timestamp;
	struct mutex lock;
	s16 fifo[SMI330_FIFO_MAX_LENGTH];
	/*
	 * Ensure natural alignment for timestamp if present.
	 * Channel size: 2 bytes.
	 * Max length needed: 2 * 3 channels + temp channel + 2 bytes padding + 8 byte ts.
	 * If fewer channels are enabled, less space may be needed, as
	 * long as the timestamp is still aligned to 8 bytes.
	 */
	s16 buf[12] __aligned(8);
};

int smi330_core_probe(struct device *dev, struct regmap *regmap);

#endif /* _SMI330_H */
