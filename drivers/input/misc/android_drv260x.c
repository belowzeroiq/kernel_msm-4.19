/*
 * drv260x.c - DRV260X haptics driver family
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright:   (C) 2014 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <dt-bindings/input/ti-drv260x.h>
#include <linux/platform_data/drv260x-pdata.h>

#include "../../staging/android/timed_output.h"

#define DRV260X_STATUS		0x0
#define DRV260X_MODE		0x1
#define DRV260X_RT_PB_IN	0x2
#define DRV260X_LIB_SEL		0x3
#define DRV260X_WV_SEQ_1	0x4
#define DRV260X_WV_SEQ_2	0x5
#define DRV260X_WV_SEQ_3	0x6
#define DRV260X_WV_SEQ_4	0x7
#define DRV260X_WV_SEQ_5	0x8
#define DRV260X_WV_SEQ_6	0x9
#define DRV260X_WV_SEQ_7	0xa
#define DRV260X_WV_SEQ_8	0xb
#define DRV260X_GO				0xc
#define DRV260X_OVERDRIVE_OFF	0xd
#define DRV260X_SUSTAIN_P_OFF	0xe
#define DRV260X_SUSTAIN_N_OFF	0xf
#define DRV260X_BRAKE_OFF		0x10
#define DRV260X_A_TO_V_CTRL		0x11
#define DRV260X_A_TO_V_MIN_INPUT	0x12
#define DRV260X_A_TO_V_MAX_INPUT	0x13
#define DRV260X_A_TO_V_MIN_OUT	0x14
#define DRV260X_A_TO_V_MAX_OUT	0x15
#define DRV260X_RATED_VOLT		0x16
#define DRV260X_OD_CLAMP_VOLT	0x17
#define DRV260X_CAL_COMP		0x18
#define DRV260X_CAL_BACK_EMF	0x19
#define DRV260X_FEEDBACK_CTRL	0x1a
#define DRV260X_CTRL1			0x1b
#define DRV260X_CTRL2			0x1c
#define DRV260X_CTRL3			0x1d
#define DRV260X_CTRL4			0x1e
#define DRV260X_CTRL5			0x1f
#define DRV260X_LRA_LOOP_PERIOD	0x20
#define DRV260X_VBAT_MON		0x21
#define DRV260X_LRA_RES_PERIOD	0x22
#define DRV260X_MAX_REG			0x23

#define DRV260X_ALLOWED_R_BYTES	25
#define DRV260X_ALLOWED_W_BYTES	2
#define DRV260X_MAX_RW_RETRIES	5
#define DRV260X_I2C_RETRY_DELAY 10

#define DRV260X_GO_BIT				0x01

/* Library Selection */
#define DRV260X_LIB_SEL_MASK		0x07
#define DRV260X_LIB_SEL_RAM			0x0
#define DRV260X_LIB_SEL_OD			0x1
#define DRV260X_LIB_SEL_40_60		0x2
#define DRV260X_LIB_SEL_60_80		0x3
#define DRV260X_LIB_SEL_100_140		0x4
#define DRV260X_LIB_SEL_140_PLUS	0x5

#define DRV260X_LIB_SEL_HIZ_MASK	0x10
#define DRV260X_LIB_SEL_HIZ_EN		0x01
#define DRV260X_LIB_SEL_HIZ_DIS		0

/* Mode register */
#define DRV260X_STANDBY				(1 << 6)
#define DRV260X_STANDBY_MASK		0x40
#define DRV260X_INTERNAL_TRIGGER	0x00
#define DRV260X_EXT_TRIGGER_EDGE	0x01
#define DRV260X_EXT_TRIGGER_LEVEL	0x02
#define DRV260X_PWM_ANALOG_IN		0x03
#define DRV260X_AUDIOHAPTIC			0x04
#define DRV260X_RT_PLAYBACK			0x05
#define DRV260X_DIAGNOSTICS			0x06
#define DRV260X_AUTO_CAL			0x07

/* Audio to Haptics Control */
#define DRV260X_AUDIO_HAPTICS_PEAK_10MS		(0 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_20MS		(1 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_30MS		(2 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_40MS		(3 << 2)

#define DRV260X_AUDIO_HAPTICS_FILTER_100HZ	0x00
#define DRV260X_AUDIO_HAPTICS_FILTER_125HZ	0x01
#define DRV260X_AUDIO_HAPTICS_FILTER_150HZ	0x02
#define DRV260X_AUDIO_HAPTICS_FILTER_200HZ	0x03

/* Min/Max Input/Output Voltages */
#define DRV260X_AUDIO_HAPTICS_MIN_IN_VOLT	0x19
#define DRV260X_AUDIO_HAPTICS_MAX_IN_VOLT	0x64
#define DRV260X_AUDIO_HAPTICS_MIN_OUT_VOLT	0x19
#define DRV260X_AUDIO_HAPTICS_MAX_OUT_VOLT	0xFF

/* Feedback register */
#define DRV260X_FB_REG_ERM_MODE			0x7f
#define DRV260X_FB_REG_LRA_MODE			(1 << 7)

#define DRV260X_BRAKE_FACTOR_MASK	0x1f
#define DRV260X_BRAKE_FACTOR_2X		(1 << 0)
#define DRV260X_BRAKE_FACTOR_3X		(2 << 4)
#define DRV260X_BRAKE_FACTOR_4X		(3 << 4)
#define DRV260X_BRAKE_FACTOR_6X		(4 << 4)
#define DRV260X_BRAKE_FACTOR_8X		(5 << 4)
#define DRV260X_BRAKE_FACTOR_16		(6 << 4)
#define DRV260X_BRAKE_FACTOR_DIS	(7 << 4)

#define DRV260X_LOOP_GAIN_LOW		0xf3
#define DRV260X_LOOP_GAIN_MED		(1 << 2)
#define DRV260X_LOOP_GAIN_HIGH		(2 << 2)
#define DRV260X_LOOP_GAIN_VERY_HIGH	(3 << 2)

#define DRV260X_BEMF_GAIN_0			0xfc
#define DRV260X_BEMF_GAIN_1		(1 << 0)
#define DRV260X_BEMF_GAIN_2		(2 << 0)
#define DRV260X_BEMF_GAIN_3		(3 << 0)

/* Control 1 register */
#define DRV260X_AC_CPLE_EN			(1 << 5)
#define DRV260X_STARTUP_BOOST		(1 << 7)

/* Control 2 register */

#define DRV260X_IDISS_TIME_45		0
#define DRV260X_IDISS_TIME_75		(1 << 0)
#define DRV260X_IDISS_TIME_150		(1 << 1)
#define DRV260X_IDISS_TIME_225		0x03

#define DRV260X_BLANK_TIME_45	(0 << 2)
#define DRV260X_BLANK_TIME_75	(1 << 2)
#define DRV260X_BLANK_TIME_150	(2 << 2)
#define DRV260X_BLANK_TIME_225	(3 << 2)

#define DRV260X_SAMP_TIME_150	(0 << 4)
#define DRV260X_SAMP_TIME_200	(1 << 4)
#define DRV260X_SAMP_TIME_250	(2 << 4)
#define DRV260X_SAMP_TIME_300	(3 << 4)

#define DRV260X_BRAKE_STABILIZER	(1 << 6)
#define DRV260X_UNIDIR_IN			(0 << 7)
#define DRV260X_BIDIR_IN			(1 << 7)

/* Control 3 Register */
#define DRV260X_LRA_OPEN_LOOP		(1 << 0)
#define DRV260X_ANANLOG_IN			(1 << 1)
#define DRV260X_LRA_DRV_MODE		(1 << 2)
#define DRV260X_RTP_UNSIGNED_DATA	(1 << 3)
#define DRV260X_SUPPLY_COMP_DIS		(1 << 4)
#define DRV260X_ERM_OPEN_LOOP		(1 << 5)
#define DRV260X_NG_THRESH_0			(0 << 6)
#define DRV260X_NG_THRESH_2			(1 << 6)
#define DRV260X_NG_THRESH_4			(2 << 6)
#define DRV260X_NG_THRESH_8			(3 << 6)

/* Control 4 Register */
#define DRV260X_AUTOCAL_TIME_150MS		(0 << 4)
#define DRV260X_AUTOCAL_TIME_250MS		(1 << 4)
#define DRV260X_AUTOCAL_TIME_500MS		(2 << 4)
#define DRV260X_AUTOCAL_TIME_1000MS		(3 << 4)

/**
 * struct drv260x_data -
 * @input_dev - Pointer to the input device
 * @client - Pointer to the I2C client
 * @timer - High res timer used to turn on/off vibration
 * @regmap - Register map of the device
 * @lock - Lock for device access
 * @work - Work item used to off load the enable/disable of the vibration
 * @enable_gpio - Pointer to the gpio used for enable/disabling
 * @regulator - Pointer to the regulator for the IC
 * @upload_effects - Vibration effect data array
 * @runtime_left - Indicates how much runtime is remaining in the timer
 * @effect_id - The ID of the vibration effect requested to be played
 * @mode - The operating mode of the IC (RTP, ERM or LRA)
 * @library - The vibration library to be used
 * @rated_voltage - The rated_voltage of the actuator
 * @overdriver_voltage - The over drive voltage of the actuator
**/
struct drv260x_data {
	struct timed_output_dev dev;
	struct i2c_client *client;
	struct hrtimer timer;
	struct regmap *regmap;
	struct mutex lock;
	struct work_struct work;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator;
	u32 runtime_left;
	u32 time_chunk_ms;
	int mode;
	int library;
	int rated_voltage;
	int overdrive_voltage;
};

static struct reg_default drv260x_reg_defs[] = {
	{ DRV260X_STATUS, 0xe0 },
	{ DRV260X_MODE, 0x40 },
	{ DRV260X_RT_PB_IN, 0x00},
	{ DRV260X_LIB_SEL, 0x00},
	{ DRV260X_WV_SEQ_1, 0x01},
	{ DRV260X_WV_SEQ_2, 0x00},
	{ DRV260X_WV_SEQ_3, 0x00},
	{ DRV260X_WV_SEQ_4, 0x00},
	{ DRV260X_WV_SEQ_5, 0x00},
	{ DRV260X_WV_SEQ_6, 0x00},
	{ DRV260X_WV_SEQ_7, 0x00},
	{ DRV260X_WV_SEQ_8, 0x00},
	{ DRV260X_GO, 0x00},
	{ DRV260X_OVERDRIVE_OFF, 0x00},
	{ DRV260X_SUSTAIN_P_OFF, 0x00},
	{ DRV260X_SUSTAIN_N_OFF, 0x00},
	{ DRV260X_BRAKE_OFF	, 0x00},
	{ DRV260X_A_TO_V_CTRL	, 0x05},
	{ DRV260X_A_TO_V_MIN_INPUT, 0x19},
	{ DRV260X_A_TO_V_MAX_INPUT, 0xff},
	{ DRV260X_A_TO_V_MIN_OUT, 0x19},
	{ DRV260X_A_TO_V_MAX_OUT, 0xff},
	{ DRV260X_RATED_VOLT, 0x3e},
	{ DRV260X_OD_CLAMP_VOLT, 0x8c},
	{ DRV260X_CAL_COMP, 0x0c},
	{ DRV260X_CAL_BACK_EMF, 0x6c},
	{ DRV260X_FEEDBACK_CTRL, 0x36},
	{ DRV260X_CTRL1, 0x93},
	{ DRV260X_CTRL2, 0xfa},
	{ DRV260X_CTRL3, 0xa0},
	{ DRV260X_CTRL4, 0x20},
	{ DRV260X_CTRL5, 0x80},
	{ DRV260X_LRA_LOOP_PERIOD, 0x33},
	{ DRV260X_VBAT_MON, 0x00},
	{ DRV260X_LRA_RES_PERIOD, 0x00},
};

/* Remove this line to disable debug */
#define DRV260X_DEBUG

#ifdef DRV260X_DEBUG
/* The registers can be accessed via
 * cat /sys/class/i2c-adapter/i2c-2/2-005a/registers
 * And written through echo for example
 * echo "CFG1 0x00" > /sys/class/i2c-adapter/i2c-2/2-005a/registers
 */
struct drv260x_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} drv260x_regs[] = {
	{ "STATUS", DRV260X_STATUS, 0 },
	{ "MODE", DRV260X_MODE, 1 },
	{ "REAL_PB", DRV260X_RT_PB_IN, 1},
	{ "LIB_SELECT", DRV260X_LIB_SEL, 1},
	{ "WAVE_SEQ_1", DRV260X_WV_SEQ_1, 1},
	{ "WAVE_SEQ_2", DRV260X_WV_SEQ_2, 1},
	{ "WAVE_SEQ_3", DRV260X_WV_SEQ_3, 1},
	{ "WAVE_SEQ_4", DRV260X_WV_SEQ_4, 1},
	{ "WAVE_SEQ_5", DRV260X_WV_SEQ_5, 1},
	{ "WAVE_SEQ_6", DRV260X_WV_SEQ_6, 1},
	{ "WAVE_SEQ_7", DRV260X_WV_SEQ_7, 1},
	{ "WAVE_SEQ_8", DRV260X_WV_SEQ_8, 1},
	{ "GO",	DRV260X_GO, 1},
	{ "OD_OFF", DRV260X_OVERDRIVE_OFF, 1},
	{ "SUSTAIN_P_OFF", DRV260X_SUSTAIN_P_OFF, 1},
	{ "SUSTAIN_N_OFF", DRV260X_SUSTAIN_N_OFF, 1},
	{ "BRAKE_OFF", DRV260X_BRAKE_OFF, 1},
	{ "AV_CTRL", DRV260X_A_TO_V_CTRL, 1},
	{ "AV_MIN_IN", DRV260X_A_TO_V_MIN_INPUT, 1},
	{ "AV_MAX_IN", DRV260X_A_TO_V_MAX_INPUT, 1},
	{ "AV_MIN_OUT",	DRV260X_A_TO_V_MIN_OUT, 1},
	{ "AV_MAX_OUT",	DRV260X_A_TO_V_MAX_OUT, 1},
	{ "RATED_VOLT",	DRV260X_RATED_VOLT, 1},
	{ "OD_CLAMP", DRV260X_OD_CLAMP_VOLT, 1},
	{ "CAL_COMP", DRV260X_CAL_COMP	, 1},
	{ "CAL_EMF", DRV260X_CAL_BACK_EMF, 1},
	{ "FB_CTRL", DRV260X_FEEDBACK_CTRL, 1},
	{ "CTRL1", DRV260X_CTRL1, 1},
	{ "CTRL2", DRV260X_CTRL2, 1},
	{ "CTRL3", DRV260X_CTRL3, 1},
	{ "CTRL4", DRV260X_CTRL4, 1},
	{ "CTRL5", DRV260X_CTRL5, 1},
	{ "LRA_LOOP", DRV260X_LRA_LOOP_PERIOD, 1},
	{ "VBAT_MON", DRV260X_VBAT_MON, 1},
	{ "LRA_RES", DRV260X_LRA_RES_PERIOD, 1},
};

static ssize_t drv260x_registers_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
	struct drv260x_data *data = dev_get_drvdata(dev);

	reg_count = sizeof(drv260x_regs) / sizeof(drv260x_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, drv260x_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       drv260x_regs[i].name,
			       read_buf);
	}
	return n;
}

static ssize_t drv260x_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct drv260x_data *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(drv260x_regs) / sizeof(drv260x_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, drv260x_regs[i].name)) {
			if (drv260x_regs[i].writeable) {
				error = regmap_write(data->regmap, drv260x_regs[i].reg, value);
				if (error) {
					pr_err("%s:Failed to write %s\n",
						__func__, name);
					return -1;
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
						__func__, name);
					return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		drv260x_registers_show, drv260x_registers_store);

static struct attribute *drv260x_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group drv260x_attr_group = {
	.attrs = drv260x_attrs,
};
#endif

#define DRV260X_DEF_RATED_VOLT		0x90
#define DRV260X_DEF_OD_CLAMP_VOLT	0x90
/*
 * Rated and Overdriver Voltages:
 * Calculated using the formula r = v * 255 / 5.6
 * where r is what will be written to the register
 * and v is the rated or overdriver voltage of the actuator
 */
static int drv260x_calculate_voltage(int voltage)
{
	return (voltage * 255 / 5600);
}

static void drv260x_worker(struct work_struct *work)
{
	struct drv260x_data *haptics = container_of(work, struct drv260x_data, work);

	gpiod_set_value(haptics->enable_gpio, 1);
	/* Data sheet says to wait 250us before trying to communicate */
	udelay(250);

	regmap_write(haptics->regmap, DRV260X_MODE,
				DRV260X_RT_PLAYBACK);

	if (haptics->runtime_left > 0)
		regmap_write(haptics->regmap,
					DRV260X_RT_PB_IN,
					0x3f);
	else
		regmap_write(haptics->regmap,
					DRV260X_RT_PB_IN,
					0x0);
}

static enum hrtimer_restart drv260x_timer(struct hrtimer *timer)
{
	struct drv260x_data *data = container_of(timer, struct drv260x_data, timer);

	data->runtime_left = 0;
	schedule_work(&data->work);

	return HRTIMER_NORESTART;
}

static void drv260x_enable(struct timed_output_dev *dev, int value)
{
	struct drv260x_data *data = container_of(dev, struct drv260x_data, dev);
	unsigned long time_ms;

	mutex_lock(&data->lock);
	hrtimer_cancel(&data->timer);

	data->runtime_left = value;
	time_ms = value * NSEC_PER_MSEC;

	schedule_work(&data->work);
	hrtimer_start(&data->timer, ktime_set(0, time_ms), HRTIMER_MODE_REL);

	mutex_unlock(&data->lock);
}

static int drv260x_get_time(struct timed_output_dev *dev)
{
	struct drv260x_data *data = container_of(dev, struct drv260x_data, dev);

	if (hrtimer_active(&data->timer))
		return	data->runtime_left +
			ktime_to_ms(hrtimer_get_remaining(&data->timer));

	return 0;
}

static const struct reg_default drv260x_lra_cal_regs[] = {
	{ DRV260X_MODE, DRV260X_AUTO_CAL },
	{ DRV260X_CTRL3, DRV260X_NG_THRESH_2},
	{ DRV260X_FEEDBACK_CTRL, DRV260X_FB_REG_LRA_MODE | DRV260X_BRAKE_FACTOR_4X | DRV260X_LOOP_GAIN_HIGH },
};

static const struct reg_default drv260x_lra_init_regs[] = {
	{ DRV260X_MODE, DRV260X_RT_PLAYBACK},
	{ DRV260X_LIB_SEL, DRV260X_LIB_F },
	{ DRV260X_A_TO_V_CTRL, DRV260X_AUDIO_HAPTICS_PEAK_20MS | DRV260X_AUDIO_HAPTICS_FILTER_125HZ},
	{ DRV260X_A_TO_V_MIN_INPUT, DRV260X_AUDIO_HAPTICS_MIN_IN_VOLT },
	{ DRV260X_A_TO_V_MAX_INPUT, DRV260X_AUDIO_HAPTICS_MAX_IN_VOLT },
	{ DRV260X_A_TO_V_MIN_OUT, DRV260X_AUDIO_HAPTICS_MIN_OUT_VOLT },
	{ DRV260X_A_TO_V_MAX_OUT, DRV260X_AUDIO_HAPTICS_MAX_OUT_VOLT },
	{ DRV260X_FEEDBACK_CTRL, DRV260X_FB_REG_LRA_MODE | DRV260X_BRAKE_FACTOR_2X | DRV260X_LOOP_GAIN_MED | DRV260X_BEMF_GAIN_3 },
	{ DRV260X_CTRL1, DRV260X_STARTUP_BOOST },
	{ DRV260X_CTRL2, DRV260X_SAMP_TIME_250 },
	{ DRV260X_CTRL3, DRV260X_NG_THRESH_2 | DRV260X_ANANLOG_IN },
	{ DRV260X_CTRL4, DRV260X_AUTOCAL_TIME_500MS },
};

static const struct reg_default drv260x_erm_cal_regs[] = {
	{ DRV260X_MODE, DRV260X_AUTO_CAL },
	{ DRV260X_A_TO_V_MIN_INPUT, DRV260X_AUDIO_HAPTICS_MIN_IN_VOLT },
	{ DRV260X_A_TO_V_MAX_INPUT, DRV260X_AUDIO_HAPTICS_MAX_IN_VOLT },
	{ DRV260X_A_TO_V_MIN_OUT, DRV260X_AUDIO_HAPTICS_MIN_OUT_VOLT },
	{ DRV260X_A_TO_V_MAX_OUT, DRV260X_AUDIO_HAPTICS_MAX_OUT_VOLT },
	{ DRV260X_FEEDBACK_CTRL, DRV260X_BRAKE_FACTOR_3X | DRV260X_LOOP_GAIN_MED | DRV260X_BEMF_GAIN_2 },
	{ DRV260X_CTRL1, DRV260X_STARTUP_BOOST },
	{ DRV260X_CTRL2, DRV260X_SAMP_TIME_250 | DRV260X_BLANK_TIME_75 | DRV260X_SAMP_TIME_250 | DRV260X_IDISS_TIME_75 },
	{ DRV260X_CTRL3, DRV260X_NG_THRESH_2 | DRV260X_ERM_OPEN_LOOP },
	{ DRV260X_CTRL4, DRV260X_AUTOCAL_TIME_500MS },
};

static int drv260x_init(struct drv260x_data *haptics)
{
	int ret;
	unsigned int cal_buf;

	ret = regmap_write(haptics->regmap,
			   DRV260X_RATED_VOLT, haptics->rated_voltage);
	if (ret != 0)
		goto write_failure;

	ret = regmap_write(haptics->regmap,
			   DRV260X_OD_CLAMP_VOLT, haptics->overdrive_voltage);
	if (ret != 0)
		goto write_failure;

	switch(haptics->mode) {
	case DRV260X_LRA_MODE:
		ret = regmap_register_patch(haptics->regmap,
					drv260x_lra_cal_regs,
					ARRAY_SIZE(drv260x_lra_cal_regs));
		if (ret != 0)
			goto write_failure;
		break;
	case DRV260X_ERM_MODE:
		ret = regmap_register_patch(haptics->regmap,
					drv260x_erm_cal_regs,
					ARRAY_SIZE(drv260x_erm_cal_regs));
		if (ret != 0)
			goto write_failure;

		ret = regmap_update_bits(haptics->regmap, DRV260X_LIB_SEL,
					DRV260X_LIB_SEL_MASK,
					haptics->library);
		if (ret != 0)
			goto write_failure;
		break;
	default:
		ret = regmap_register_patch(haptics->regmap,
					drv260x_lra_init_regs,
					ARRAY_SIZE(drv260x_lra_init_regs));
		if (ret != 0)
			goto write_failure;

		ret = regmap_update_bits(haptics->regmap, DRV260X_LIB_SEL,
					DRV260X_LIB_SEL_MASK,
					haptics->library);
		if (ret != 0)
			goto write_failure;

		goto skip_go_bit;
		break;
	}

	if (ret != 0) {
		dev_err(&haptics->client->dev,
			"Failed to write init registers: %d\n",
			ret);
		goto write_failure;
	}

	ret = regmap_write(haptics->regmap, DRV260X_GO, DRV260X_GO_BIT);
	if (ret != 0)
		goto write_failure;

	do {
		ret = regmap_read(haptics->regmap, DRV260X_GO, &cal_buf);
		if (ret != 0)
			goto write_failure;
	} while (cal_buf == DRV260X_GO_BIT || ret != 0);

	return ret;

write_failure:
	dev_err(&haptics->client->dev,
		"Failed to write init registers: %d\n",
		ret);
skip_go_bit:
	return ret;
}

static const struct regmap_config drv260x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = DRV260X_MAX_REG,
	.reg_defaults = drv260x_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(drv260x_reg_defs),
	.cache_type = REGCACHE_NONE,
};

#ifdef CONFIG_OF
static int drv260x_parse_dt(struct device_node *dev_node,
				struct drv260x_data *haptics,
				struct device *dev)
{
	int ret;
	int voltage;

	ret = of_property_read_u32(dev_node, "mode", &haptics->mode);
	if (ret < 0) {
		dev_err(dev, "%s: No entry for mode\n", __func__);

		return ret;
	}
	ret = of_property_read_u32(dev_node, "library-sel",
				&haptics->library);
	if (ret < 0) {
		dev_err(dev, "%s: No entry for library selection\n",
			__func__);

		return ret;
	}
	ret = of_property_read_u32(dev_node, "vib-rated-voltage",
				&voltage);
	if (!ret)
		haptics->rated_voltage = drv260x_calculate_voltage(voltage);


	ret = of_property_read_u32(dev_node, "vib-overdrive-voltage",
				&voltage);
	if (!ret)
		haptics->overdrive_voltage = drv260x_calculate_voltage(voltage);

	return ret;

}
#else
static inline int drv260x_parse_dt(struct device *dev)
{
	dev_err(dev, "no platform data defined\n");

	return -EINVAL;
}
#endif

static int drv260x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct drv260x_data *haptics;
	struct device_node *np = client->dev.of_node;
	struct drv260x_platform_data *pdata = client->dev.platform_data;
	int ret;

	haptics = devm_kzalloc(&client->dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->rated_voltage = DRV260X_DEF_OD_CLAMP_VOLT;
	haptics->rated_voltage = DRV260X_DEF_RATED_VOLT;

	 if (pdata) {
		haptics->mode = pdata->mode;
		haptics->library = pdata->library_selection;
		if (pdata->vib_overdrive_voltage)
			haptics->overdrive_voltage = drv260x_calculate_voltage(pdata->vib_overdrive_voltage);
		if (pdata->vib_rated_voltage)
			haptics->rated_voltage = drv260x_calculate_voltage(pdata->vib_rated_voltage);
	} else if (np) {
		ret = drv260x_parse_dt(np, haptics, &client->dev);
		if (ret)
			return ret;
	} else {
		dev_err(&client->dev, "Platform data not set\n");
		return -ENODEV;
	}

	if (haptics->mode < DRV260X_LRA_MODE ||
	    haptics->mode > DRV260X_ERM_MODE) {
		dev_err(&client->dev,
			"Vibrator mode is invalid: %i\n",
			haptics->mode);
		return -EINVAL;
	}
	
	if (haptics->library < DRV260X_LIB_SEL_DEFAULT ||
	    haptics->library > DRV260X_LIB_F) {
		dev_err(&client->dev,
			"Library value is invalid: %i\n", haptics->library);
		return -EINVAL;
	}	

	haptics->enable_gpio = devm_gpiod_get(&client->dev, "enable");
	if (IS_ERR(haptics->enable_gpio)) {
		ret = PTR_ERR(haptics->enable_gpio);
		if (ret != -ENOENT && ret != -ENOSYS)
			return ret;

		haptics->enable_gpio = NULL;
		goto err_gpio;
	} else {
		gpiod_direction_output(haptics->enable_gpio, 1);
	}

	haptics->regulator = devm_regulator_get(&client->dev, "vbat");
	if (IS_ERR(haptics->regulator)) {
		ret = PTR_ERR(haptics->regulator);
		dev_err(&client->dev,
			"unable to get regulator, error: %d\n", ret);
		return ret;
	}

	mutex_init(&haptics->lock);
	INIT_WORK(&haptics->work, drv260x_worker);
	hrtimer_init(&haptics->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptics->timer.function = drv260x_timer;

	haptics->dev.name = "drv260x-haptics";
	haptics->dev.get_time = drv260x_get_time;
	haptics->dev.enable = drv260x_enable;

#ifdef DRV260X_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &drv260x_attr_group);
	if (ret < 0)
		dev_err(&client->dev, "Failed to create sysfs: %d\n", ret);
#endif

	haptics->client = client;
	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client, &drv260x_regmap_config);
	if (IS_ERR(haptics->regmap)) {
		ret = PTR_ERR(haptics->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto err_regmap;
	}

	drv260x_init(haptics);

	ret = timed_output_dev_register(&haptics->dev);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s:Cannot register time output dev error %i\n",
			__func__, ret);
		goto err_timed_out;
	}

	return 0;

err_regmap:
err_gpio:
	timed_output_dev_unregister(&haptics->dev);
err_timed_out:
	return ret;
}

static int drv260x_remove(struct i2c_client *client)
{
	struct drv260x_data *data = i2c_get_clientdata(client);

	timed_output_dev_unregister(&data->dev);

#ifdef DRV260X_DEBUG
	sysfs_remove_group(&client->dev.kobj, &drv260x_attr_group);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int drv260x_suspend(struct device *dev)
{
	struct drv260x_data *haptics = dev_get_drvdata(dev);

	regmap_update_bits(haptics->regmap,
			   DRV260X_MODE,
			   DRV260X_STANDBY_MASK,
			   DRV260X_STANDBY);
	gpiod_set_value(haptics->enable_gpio, 0);

	regulator_disable(haptics->regulator);

	return 0;
}

static int drv260x_resume(struct device *dev)
{
	struct drv260x_data *haptics = dev_get_drvdata(dev);
	int ret;

	ret = regulator_enable(haptics->regulator);
	if (ret) {
		dev_err(dev, "Failed to enable regulator\n");
		return ret;
	}
	regmap_update_bits(haptics->regmap,
			   DRV260X_MODE,
			   DRV260X_STANDBY_MASK, 0);

	gpiod_set_value(haptics->enable_gpio, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(drv260x_pm_ops, drv260x_suspend, drv260x_resume);

static const struct i2c_device_id drv260x_id[] = {
	{ "drv260x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, drv260x_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id drv260x_of_match[] = {
	{ .compatible = "ti,drv2604", },
	{ .compatible = "ti,drv2604l", },
	{ .compatible = "ti,drv2605", },
	{ .compatible = "ti,drv2605l", },
	{}
};
MODULE_DEVICE_TABLE(of, drv260x_of_match);
#endif

static struct i2c_driver drv260x_driver = {
	.probe		= drv260x_probe,
	.remove		= drv260x_remove,
	.driver		= {
		.name	= "drv260x-haptics",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(drv260x_of_match),
		.pm	= &drv260x_pm_ops,
	},
	.id_table = drv260x_id,
};
module_i2c_driver(drv260x_driver);

MODULE_ALIAS("platform:drv260x-haptics");
MODULE_DESCRIPTION("TI DRV260x haptics driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
