/* SPDX-License-Identifier: GPL-2.0-only */
// BQ25790 Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#ifndef _BQ25790_CHARGER_H
#define _BQ25790_CHARGER_H

#include <linux/i2c.h>

#define BQ25790_MANUFACTURER	"Texas Instruments"
#define BQ25790_NAME		"bq25790"

#define BQ25790_MIN_SYS_V	0x00
#define BQ25790_CHRG_V_LIM_MSB	0x01
#define BQ25790_CHRG_V_LIM_LSB	0x02
#define BQ25790_CHRG_I_LIM_MSB	0x03
#define BQ25790_CHRG_I_LIM_LSB	0x04
#define BQ25790_INPUT_V_LIM	0x05
#define BQ25790_INPUT_I_LIM_MSB	0x06
#define BQ25790_INPUT_I_LIM_LSB	0x07
#define BQ25790_PRECHRG_CTRL	0x08
#define BQ25790_TERM_CTRL	0x09
#define BQ25790_RECHRG_CTRL	0x0a
#define BQ25790_VOTG_REG	0x0b
#define BQ25790_IOTG_REG	0x0d
#define BQ25790_TIMER_CTRL	0x0e
#define BQ25790_CHRG_CTRL_0	0x0f
#define BQ25790_CHRG_CTRL_1	0x10
#define BQ25790_CHRG_CTRL_2	0x11
#define BQ25790_CHRG_CTRL_3	0x12
#define BQ25790_CHRG_CTRL_4	0x13
#define BQ25790_CHRG_CTRL_5	0x14
#define BQ25790_MPPT_CTRL	0x15
#define BQ25790_TEMP_CTRL	0x16
#define BQ25790_NTC_CTRL_0	0x17
#define BQ25790_NTC_CTRL_1	0x18
#define BQ25790_ICO_I_LIM	0x19
#define BQ25790_CHRG_STAT_0	0x1b
#define BQ25790_CHRG_STAT_1	0x1c
#define BQ25790_CHRG_STAT_2	0x1d
#define BQ25790_CHRG_STAT_3	0x1e
#define BQ25790_CHRG_STAT_4	0x1f
#define BQ25790_FAULT_STAT_0	0x20
#define BQ25790_FAULT_STAT_1	0x21
#define BQ25790_CHRG_FLAG_0	0x22
#define BQ25790_CHRG_FLAG_1	0x23
#define BQ25790_CHRG_FLAG_2	0x24
#define BQ25790_CHRG_FLAG_3	0x25
#define BQ25790_FAULT_FLAG_0	0x26
#define BQ25790_FAULT_FLAG_1	0x27
#define BQ25790_CHRG_MSK_0	0x28
#define BQ25790_CHRG_MSK_1	0x29
#define BQ25790_CHRG_MSK_2	0x2a
#define BQ25790_CHRG_MSK_3	0x2b
#define BQ25790_FAULT_MSK_0	0x2c
#define BQ25790_FAULT_MSK_1	0x2d
#define BQ25790_ADC_CTRL	0x2e
#define BQ25790_FN_DISABE_0	0x2f
#define BQ25790_FN_DISABE_1	0x30
#define BQ25790_ADC_IBUS	0x31
#define BQ25790_ADC_IBAT_MSB	0x33
#define BQ25790_ADC_IBAT_LSB	0x34
#define BQ25790_ADC_VBUS	0x35
#define BQ25790_ADC_VAC1	0x37
#define BQ25790_ADC_VAC2	0x39
#define BQ25790_ADC_VBAT_MSB	0x3b
#define BQ25790_ADC_VBAT_LSB	0x3c
#define BQ25790_ADC_VSYS_MSB	0x3d
#define BQ25790_ADC_VSYS_LSB	0x3e
#define BQ25790_ADC_TS		0x3f
#define BQ25790_ADC_TDIE	0x41
#define BQ25790_ADC_DP		0x43
#define BQ25790_ADC_DM		0x45
#define BQ25790_DPDM_DRV	0x47
#define BQ25790_PART_INFO	0x48

#define BQ25790_CHRG_EN		BIT(5)
#define BQ25790_ADC_EN		BIT(7)

/* Charger Status 1 */
#define BQ25790_CHG_STAT_MSK	GENMASK(7, 5)
#define BQ25790_NOT_CHRGING	0
#define BQ25790_TRICKLE_CHRG	BIT(5)
#define BQ25790_PRECHRG		BIT(6)
#define BQ25790_FAST_CHRG	(BIT(5) | BIT(6))
#define BQ25790_TAPER_CHRG	BIT(7)
#define BQ25790_TOP_OFF_CHRG	(BIT(6) | BIT(7))
#define BQ25790_TERM_CHRG	(BIT(5) | BIT(6) | BIT(7))
#define BQ25790_VBUS_PRESENT	BIT(0)

#define BQ25790_VBUS_STAT_MSK	GENMASK(4, 1)
#define BQ25790_USB_SDP		BIT(1)
#define BQ25790_USB_CDP		BIT(2)
#define BQ25790_USB_DCP		(BIT(1) | BIT(2))
#define BQ25790_HVDCP		BIT(3)
#define BQ25790_UNKNOWN_3A	(BIT(3) | BIT(1))
#define BQ25790_NON_STANDARD	(BIT(3) | BIT(2))
#define BQ25790_OTG_MODE	(BIT(3) | BIT(2) | BIT(1))
#define BQ25790_UNQUAL_ADAPT	BIT(4)
#define BQ25790_DIRECT_PWR	(BIT(4) | BIT(2) | BIT(1))

/* Charger Status 4 */
#define BQ25790_TEMP_HOT	BIT(0)
#define BQ25790_TEMP_WARM	BIT(1)
#define BQ25790_TEMP_COOL	BIT(2)
#define BQ25790_TEMP_COLD	BIT(3)
#define BQ25790_TEMP_MASK	GENMASK(3, 0)

#define BQ25790_OTG_OVP		BIT(5)
#define BQ25790_VSYS_OVP	BIT(6)
#define BQ25790_VSYS_STAT	BIT(4)

#define BQ25790_PRECHRG_CUR_MASK		GENMASK(5, 0)
#define BQ25790_PRECHRG_CURRENT_STEP_uA		40000
#define BQ25790_PRECHRG_I_MIN_uA		40000
#define BQ25790_PRECHRG_I_MAX_uA		2000000
#define BQ25790_PRECHRG_I_DEF_uA		120000
#define BQ25790_TERMCHRG_CUR_MASK		GENMASK(4, 0)
#define BQ25790_TERMCHRG_CURRENT_STEP_uA	40000
#define BQ25790_TERMCHRG_I_MIN_uA		40000
#define BQ25790_TERMCHRG_I_MAX_uA		1000000
#define BQ25790_TERMCHRG_I_DEF_uA		200000
#define BQ25790_ICHRG_CURRENT_STEP_uA		10000
#define BQ25790_ICHRG_I_MIN_uA			50000
#define BQ25790_ICHRG_I_MAX_uA			5000000
#define BQ25790_ICHRG_I_DEF_uA			1000000

#define BQ25790_VREG_V_MAX_uV	18800000
#define BQ25790_VREG_V_MIN_uV	3000000
#define BQ25790_VREG_V_DEF_uV	3600000
#define BQ25790_VREG_V_STEP_uV	10000

#define BQ25790_IINDPM_I_MIN_uA	100000
#define BQ25790_IINDPM_I_MAX_uA	3300000
#define BQ25790_IINDPM_STEP_uA	10000
#define BQ25790_IINDPM_DEF_uA	1000000

#define BQ25790_VINDPM_V_MIN_uV 3600000
#define BQ25790_VINDPM_V_MAX_uV 22000000
#define BQ25790_VINDPM_STEP_uV	100000
#define BQ25790_VINDPM_DEF_uV	3600000

#define BQ25790_ADC_VOLT_STEP_uV	1000
#define BQ25790_ADC_CURR_STEP_uA	1000

#define BQ25790_WATCHDOG_MASK	GENMASK(2, 0)
#define BQ25790_WATCHDOG_DIS	0
#define BQ25790_WATCHDOG_MAX	160000

struct bq25790_init_data {
	u32 ichg;	/* charge current		*/
	u32 ilim;	/* input current		*/
	u32 vreg;	/* regulation voltage		*/
	u32 iterm;	/* termination current		*/
	u32 iprechg;	/* precharge current		*/
	u32 vlim;	/* minimum system voltage limit */
};

struct bq25790_state {
	bool online;
	u8 chrg_status;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 vsys_status;
	u8 vbus_status;
	u8 fault_0;
	u8 fault_1;
	u32 vbat_adc;
	u32 vsys_adc;
	u32 ibat_adc;
};

struct bq25790_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;

	struct usb_phy *usb2_phy;
	struct usb_phy *usb3_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	struct bq25790_init_data init_data;
	struct bq25790_state state;
	u32 watchdog_timer;
};

int bq25790_init_debug(struct bq25790_device *bq);

#endif /* _BQ25790_CHARGER_H */
