/*
 * TPS6598X usb phy driver for type-c and PD
 *
 * Author:      Dan Murphy <dmurphy@ti.com>
 * Copyright (C) 2017 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/class-dual-role.h>
#include <linux/usb/usb_typec.h>
#include <linux/usb/typec.h>

#include "tps6598x.h"

#define TYPEC_STD_MA			900
#define TYPEC_MED_MA			1500
#define TYPEC_HIGH_MA			3000

static const struct tps6598x_reg_data tps6598x_reg[] = {
	{ TPS6598X_VID, 4, 0 },
	{ TPS6598X_DID, 4, 0 },
	{ TPS6598X_PROTO_VER, 4, 0 },
	{ TPS6598X_MODE, 4, 0 },
	{ TPS6598X_TYPE, 4, 0 },
	{ TPS6598X_UID, 16, 0 },
	{ TPS6598X_VERSION, 4, 0 },
	{ TPS6598X_INT_EVENT_1, 8, 0 },
	{ TPS6598X_INT_EVENT_2, 8, 0 },
	{ TPS6598X_INT_MASK_1, 8, 1 },
	{ TPS6598X_INT_MASK_2, 8, 1 },
	{ TPS6598X_INT_CLEAR_1, 8, 1 },
	{ TPS6598X_INT_CLEAR_2, 8, 1 },
	{ TPS6598X_STATUS, 4, 0 },
	{ TPS6598X_PWR_STATUS, 2, 0 },
	{ TPS6598X_SYS_CFG, 17, 1 },
	{ TPS6598X_CTRL_CFG, 5, 1 },
	{ TPS6598X_TX_SNK_CAP, 57, 1 },
};

struct tps6598x_priv {
	struct device *dev;
	struct i2c_client *client;
	struct dual_role_phy_instance *tps6598x_instance;
	int gpio_int;
	int gpio_reset;
	struct work_struct tps6598x_work;
	struct power_supply type_c_psy;
	struct power_supply *batt_psy;
	enum typec_attached_state attached_state;
	enum typec_port_mode port_mode;
	int irqz_int;
	int typec_state;
	int current_ma;
	int bc_charger_type;
};

static struct tps6598x_priv *tps6598x_data;

enum dual_role_property tps6598x_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_PR,
};

static enum power_supply_property tps6598x_typec_properties[] = {
	POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_TYPEC_MODE,
};

static int tps6598x_i2c_write(struct tps6598x_priv *tps6598x_data, int reg,
				const void *data)
{
	unsigned i, reg_count, writeable, bytes_to_write = 0;
	u8 *buf;
	int ret;

	reg_count = sizeof(tps6598x_reg) / sizeof(tps6598x_reg[0]);
	for (i = 0; i < reg_count; i++) {
		if (tps6598x_reg[i].reg_num == reg) {
			writeable = tps6598x_reg[i].writeable;
			bytes_to_write = tps6598x_reg[i].num_of_bytes;
			break;
		}

		if (i == reg_count) {
			dev_warn(&tps6598x_data->client->dev,
				"Reg 0x%X not found\n", reg);
			return -ENODEV;
		}
	}

	buf = kzalloc(bytes_to_write, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], data, bytes_to_write);

	ret = i2c_master_send(tps6598x_data->client, buf, bytes_to_write);
	if (ret == bytes_to_write) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&tps6598x_data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	kfree(buf);
	return ret;

}

static int tps6598x_i2c_read(struct tps6598x_priv *tps6598x_data, int reg,
				void *data)
{
	unsigned i, reg_count, bytes_to_read = 0;
	u8 buf[1];
	int ret;

	reg_count = sizeof(tps6598x_reg) / sizeof(tps6598x_reg[0]);
	for (i = 0; i < reg_count; i++) {
		if (tps6598x_reg[i].reg_num == reg) {
			/* Add one to the number of bytes as the first byte
			 * indicates the number of bytes acutally returned
			 */
			bytes_to_read = tps6598x_reg[i].num_of_bytes + 1;
			break;
		}

		if (i == reg_count) {
			dev_warn(&tps6598x_data->client->dev,
				"Reg 0x%X not found\n", reg);
			return -ENODEV;
		}
	}

	buf[0] = reg;

	ret = i2c_master_send(tps6598x_data->client, buf, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;

		dev_err(&tps6598x_data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
		return ret;
	}

	ret = i2c_master_recv(tps6598x_data->client, data, bytes_to_read);
	if (ret < 0)
		dev_warn(&tps6598x_data->client->dev, "i2c read data cmd failed\n");

	return ret;
}

/* Power supply functions */
static int set_property_on_battery(enum power_supply_property prop)
{
	int rc = 0;
	union power_supply_propval ret = {0, };

	if (!tps6598x_data->batt_psy) {
		tps6598x_data->batt_psy = power_supply_get_by_name("battery");
		if (!tps6598x_data->batt_psy) {
			pr_err("no batt psy found\n");
			return -ENODEV;
		}
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		ret.intval = tps6598x_data->current_ma;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_CAPABILITY, &ret);
		if (rc)
			pr_err("failed to set current max rc=%d\n", rc);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		/*
		 * Notify the typec mode to charger. This is useful in the DFP
		 * case where there is no notification of OTG insertion to the
		 * charger driver.
		 */
		ret.intval = tps6598x_data->bc_charger_type;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
				POWER_SUPPLY_PROP_TYPE, &ret);
		if (rc)
			pr_err("failed to set typec mode rc=%d\n", rc);
		break;

	case POWER_SUPPLY_PROP_TYPEC_MODE:
		/*
		 * Notify the typec mode to charger. This is useful in the DFP
		 * case where there is no notification of OTG insertion to the
		 * charger driver.
		 */
		ret.intval = tps6598x_data->typec_state;
		rc = tps6598x_data->batt_psy->set_property(tps6598x_data->batt_psy,
				POWER_SUPPLY_PROP_TYPEC_MODE, &ret);
		if (rc)
			pr_err("failed to set typec mode rc=%d\n", rc);
		break;
	default:
		pr_err("invalid request\n");
		rc = -EINVAL;
	}

	return rc;
}

static int tps6598x_typec_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = tps6598x_data->bc_charger_type;
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		val->intval = tps6598x_data->typec_state;
		break;
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		val->intval = tps6598x_data->current_ma;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tps6598x_set_port_mode(enum typec_port_mode port_mode)
{
	int ret = 0;
	u8 obuf[18];
	u8 mask_val = 0;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -EIO;
	}

	switch (port_mode) {
	case TYPEC_UFP_MODE:
		pr_info("%s: UFP Mode\n", __func__);
		mask_val = TPS6598X_UFP_ONLY_MODE;
		break;
	case TYPEC_DFP_MODE:
		pr_info("%s: DFP Mode\n", __func__);
		mask_val = TPS6598X_DFP_ONLY_MODE;
		break;
	case TYPEC_DRP_MODE:
		mask_val = TPS6598X_DUAL_ROLE_MODE;
		pr_info("%s: DRP Mode\n", __func__);
		break;
	default:
		pr_info("%s: Default\n", __func__);
		break;
	}

	tps6598x_data->port_mode = port_mode;

	obuf[1] &= TPS6598X_PORTINFO_MASK;
	obuf[1] |= mask_val;

	return tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));
}

/* Type C framework functions */
static enum typec_port_mode tps6598x_port_mode_get(void)
{
	enum typec_port_mode port_mode = TYPEC_MODE_ACCORDING_TO_PROT;
	int ret;
	u8 obuf[5];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return port_mode;
	}

	if ((obuf[1] & TPS6598X_ATTACHED_STATUS) == 0x00)
		return TYPEC_NOT_ATTACHED;

	mask_val = obuf[1] & TPS6598X_DATA_ROLE;
	switch (mask_val) {
	case TPS6598X_REG_STATUS_AS_DFP:
		port_mode = TYPEC_ATTACHED_AS_DFP;
		break;
	case TPS6598X_REG_STATUS_AS_UFP:
		port_mode = TYPEC_ATTACHED_AS_UFP;
		break;
	default:
		port_mode = TYPEC_NOT_ATTACHED;
	}
	pr_info("%s: port mode is %d\n", __func__, port_mode);

	return port_mode;
}

static enum typec_attached_state tps6598x_attached_state_detect(void)
{
	u8 obuf[5];
	int ret;
	u8 mask_val;

	tps6598x_data->attached_state = TYPEC_NOT_ATTACHED;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return tps6598x_data->attached_state;
	}

	printk("%s: 0x%X 0x%X 0x%X 0x%X\n", __func__,
		obuf[1], obuf[2], obuf[3], obuf[4]);

	if ((obuf[1] & TPS6598X_ATTACHED_STATUS) == 0x00)
		return TYPEC_NOT_ATTACHED;

	mask_val = obuf[1] & TPS6598X_DATA_ROLE;
	switch (mask_val) {
	case TPS6598X_REG_STATUS_AS_DFP:
		tps6598x_data->attached_state = TYPEC_ATTACHED_AS_DFP;
		break;
	case TPS6598X_REG_STATUS_AS_UFP:
		tps6598x_data->attached_state = TYPEC_ATTACHED_AS_UFP;
		break;
	default:
		tps6598x_data->attached_state = TYPEC_NOT_ATTACHED;
	}

	pr_debug("%s: attached state is %d\n", __func__,
			tps6598x_data->attached_state);

	return tps6598x_data->attached_state;
}

static enum typec_current_mode tps6598x_current_mode_detect(void)
{
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	int ret;
	int current_ma;
	int charger_type;
	u8 obuf[3];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_PWR_STATUS, &obuf);
	if (ret < 0) {
		pr_err("%s: read POWER_STATUS error\n", __func__);
		return current_mode;
	}

	if (tps6598x_data->attached_state == TYPEC_NOT_ATTACHED) {
		tps6598x_data->current_ma = TYPEC_CURRENT_MODE_UNSPPORTED;
		tps6598x_data->bc_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		set_property_on_battery(POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
		set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);

		return 0;
	}

	mask_val = obuf[1] & TPS6598X_DETECT_CURR_MASK;
	switch (mask_val) {
	case TPS6598X_REG_CUR_MODE_DETECT_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		current_ma = TYPEC_STD_MA;
		break;
	case TPS6598X_REG_CUR_MODE_DETECT_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		current_ma = TYPEC_MED_MA;
		break;
	case TPS6598X_REG_CUR_MODE_DETECT_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		current_ma = TYPEC_HIGH_MA;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
		current_ma = 0;
	}

	if (tps6598x_data->current_ma != current_ma) {
		tps6598x_data->current_ma = current_ma;
		set_property_on_battery(POWER_SUPPLY_PROP_CURRENT_CAPABILITY);
	}

	mask_val = obuf[1] & TPS6598X_BCSTATUS_MASK;
	switch (mask_val) {
	case TPS6598X_BCSTATUS_SDP:
		charger_type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case TPS6598X_BCSTATUS_DCP:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case TPS6598X_BCSTATUS_CDP:
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	default:
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	if (tps6598x_data->bc_charger_type != charger_type) {
		tps6598x_data->bc_charger_type = charger_type;
		set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
	}

	pr_info("%s: current mode is %d\n", __func__, current_mode);
	pr_info("%s: bcstatus is is %d\n", __func__, charger_type);

	return current_mode;
}

static enum typec_current_mode tps6598x_current_advertise_get(void)
{
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	int ret;
	u8 obuf[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return current_mode;
	}

	mask_val = obuf[1] & TPS6598X_REG_CUR_MODE_ADVERTISE_MASK;
	switch (mask_val) {
	case TPS6598X_REG_CUR_MODE_ADVERTISE_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		break;
	case TPS6598X_REG_CUR_MODE_ADVERTISE_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		break;
	case TPS6598X_REG_CUR_MODE_ADVERTISE_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
	}

	pr_info("%s: current advertise is %d\n", __func__, current_mode);

	return current_mode;
}

static int tps6598x_set_bcenabled(int enable)
{
	int ret;
	u8 obuf[18];
	u8 bcenable;
	u8 bcmask;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_MASK_1, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -ENODEV;
	}

	if (enable)
		bcmask = obuf[4] | BIT(0);
	else
		bcmask = obuf[4] & ~BIT(0);

	obuf[4] = bcmask;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_MASK_1, &(obuf[0]));
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_MASK_2, &(obuf[0]));

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_MASK_1, &obuf);
	if (ret < 0) {
		pr_err("%s: read TPS6598X_INT_MASK_1 error\n", __func__);
		return -ENODEV;
	}

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return -ENODEV;
	}

	if (enable)
		bcenable = obuf[5] | BIT(0);
	else
		bcenable = obuf[5] & ~BIT(0);

	obuf[5] = bcenable;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));

	return ret;
}

static int tps6598x_current_advertise_set(enum typec_current_mode current_mode)
{
	int ret;
	u8 obuf[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read SYS_CFG error\n", __func__);
		return current_mode;
	}

	switch (current_mode) {
	case TYPEC_CURRENT_MODE_MID:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_MID;
		break;
	case TYPEC_CURRENT_MODE_HIGH:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_HIGH;
		break;
	default:
		mask_val = TPS6598X_REG_CUR_MODE_ADVERTISE_DEFAULT;
	}

	if (mask_val == (obuf[1] & TPS6598X_REG_CUR_MODE_ADVERTISE_MASK)) {
		pr_info("%s: current advertise is %d already\n", __func__,
			current_mode);
		return 0;
	}

	obuf[1] &= ~TPS6598X_REG_CUR_MODE_ADVERTISE_MASK;
	obuf[1] |= mask_val;

	tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(obuf[0]));

	pr_info("%s: current advertise set to %d\n", __func__, current_mode);
	return 0;
}

static int tps6598x_port_mode_set(enum typec_port_mode port_mode)
{
	int ret = 0;

	ret = tps6598x_set_port_mode(port_mode);
	if (ret)
		pr_info("%s: port mode failed to set to %d\n",
			__func__, port_mode);
	else
		pr_info("%s: port mode set to %d\n", __func__, port_mode);

	return ret;
}

static ssize_t tps6598x_dump_regs(char *buf)
{
	char reg[TPS6598X_RX_VDM] = { 0 };

	return scnprintf(buf, PAGE_SIZE,
			 "0x%02X,0x%02X,0x%02X\n", reg[8], reg[9], reg[10]);
}

/* Dual Role Class Functions */
static int tps6598x_get_property(struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				unsigned int *val)
{
	enum typec_attached_state attached_state;

	attached_state = tps6598x_attached_state_detect();
	tps6598x_current_mode_detect();

	if (attached_state == TYPEC_ATTACHED_AS_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

static int tps6598x_set_power_role(const unsigned int *val)
{
	int ret = 0;
	u8 obuf[6];
	u8 sink_cap[58];
	u8 sys_cfg[18];
	u8 mask_val;

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_CTRL_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read CTRL_CFG error\n", __func__);
		return -EIO;
	}

	if (*val == DUAL_ROLE_PROP_PR_SRC)
		mask_val = (TPS6598X_INIT_SRC_MODE | TPS6598X_PROCESS_SRC_MODE);
	else if (*val == DUAL_ROLE_PROP_PR_SNK)
		mask_val = (TPS6598X_INIT_SNK_MODE | TPS6598X_PROCESS_SNK_MODE);
	else
		return -EINVAL;

	obuf[1] &= TPS6598X_PR_SWAP_MASK;
	obuf[1] |= mask_val;

	if (tps6598x_data->port_mode == TYPEC_DFP_MODE) {
		ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_SYS_CFG, &sys_cfg);
		if (ret < 0) {
			pr_err("%s: read SYS_CFG error\n", __func__);
			return -EIO;
		}
		sys_cfg[1] &= TPS6598X_PORTINFO_MASK;
		sys_cfg[1] |= TPS6598X_UFP_DR_PR_SWAP;
		sys_cfg[4] &= TPS6598X_PP_HV_MASK;
		sys_cfg[4] |= TPS6598X_PP_HV_INPUT;

		ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_TX_SNK_CAP, &sink_cap);
		if (ret < 0) {
			pr_err("%s: read SYS_CFG error\n", __func__);
			return -EIO;
		}
		/* The TRM indicates what these numbers are but not how they
		 * are derived so they are magical numbers
		 */
		sink_cap[1] = 0x01;
		sink_cap[2] = 0x2c;
		sink_cap[3] = 0x91;
		sink_cap[4] = 0x1;

		tps6598x_i2c_write(tps6598x_data, TPS6598X_SYS_CFG, &(sys_cfg[0]));
		tps6598x_i2c_write(tps6598x_data, TPS6598X_TX_SNK_CAP, &(sink_cap[0]));
	}

	tps6598x_i2c_write(tps6598x_data, TPS6598X_CTRL_CFG, &(obuf[0]));

	ret = tps6598x_i2c_read(tps6598x_data, TPS6598X_CTRL_CFG, &obuf);
	if (ret < 0) {
		pr_err("%s: read CTRL_CFG error\n", __func__);
		return -EIO;
	}

	return ret;
}

static int tps6598x_set_property(struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				const unsigned int *val)
{
	enum typec_port_mode mode_switch;
	int ret = 0;

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		if (*val == DUAL_ROLE_PROP_MODE_DFP)
			mode_switch = TYPEC_DFP_MODE;
		else if (*val == DUAL_ROLE_PROP_MODE_UFP)
			mode_switch = TYPEC_UFP_MODE;
		else if (*val == DUAL_ROLE_PROP_MODE_DRP)
			mode_switch = TYPEC_DRP_MODE;
		else
			return -EINVAL;

		ret = tps6598x_set_port_mode(mode_switch);
		break;
	case DUAL_ROLE_PROP_PR:
		ret = tps6598x_set_power_role(val);
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
	case DUAL_ROLE_PROP_SUPPORTED_MODES:
	case DUAL_ROLE_PROP_DR:
		pr_info("%s: prop: %d, not supported case so far\n",
			__func__, prop);
		break;
	default:
		pr_info("%s: the input(prop: %d) is not supported\n",
			__func__, prop);
		break;
	}

	return ret;
}

static int tps6598x_property_is_writeable(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop)
{
	switch (prop) {
	case DUAL_ROLE_PROP_PR:
	case DUAL_ROLE_PROP_MODE:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static void tps6598x_int_work(struct work_struct *work)
{
	enum typec_attached_state attached_state;
	u8 obuf[TPS6598X_MAX_READ_BYTES];
	int ret = 0;

	attached_state = tps6598x_attached_state_detect();
	tps6598x_current_mode_detect();

	if (TYPEC_ATTACHED_AS_DFP == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_DETECTED);
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_DFP;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_DFP;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);
	} else if (TYPEC_ATTACHED_AS_UFP == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_REMOVED);
		/* device in UFP state */
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_UFP;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_UFP;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);

	} else if (TYPEC_NOT_ATTACHED == attached_state) {
		typec_sink_detected_handler(TYPEC_SINK_REMOVED);
		tps6598x_data->typec_state = POWER_SUPPLY_TYPE_UNKNOWN;
		tps6598x_data->type_c_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
		ret = set_property_on_battery(POWER_SUPPLY_PROP_TYPEC_MODE);
		if (ret)
			pr_err("failed to set TYPEC MODE on battery psy rc=%d\n", ret);
	}

	tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_EVENT_1, &obuf);
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_CLEAR_1, &obuf);

	tps6598x_i2c_read(tps6598x_data, TPS6598X_INT_EVENT_2, &obuf);
	tps6598x_i2c_write(tps6598x_data, TPS6598X_INT_CLEAR_2, &obuf);

	tps6598x_i2c_read(tps6598x_data, TPS6598X_STATUS, &obuf);

	if (tps6598x_data->tps6598x_instance)
		dual_role_instance_changed(tps6598x_data->tps6598x_instance);
}

static irqreturn_t tps6598x_irq(int irq, void *dev)
{
	struct tps6598x_priv *tps6598x_data = dev;

	schedule_work(&tps6598x_data->tps6598x_work);

	return IRQ_HANDLED;
}

static const struct dual_role_phy_desc tps6598x_desc = {
	.name = "otg_default",
	.properties = tps6598x_properties,
	.num_properties = ARRAY_SIZE(tps6598x_properties),
	.get_property = tps6598x_get_property,
	.set_property = tps6598x_set_property,
	.property_is_writeable = tps6598x_property_is_writeable,
	.supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP
};

struct typec_device_ops tps6598x_ops = {
	.current_detect = tps6598x_current_mode_detect,
	.attached_state_detect = tps6598x_attached_state_detect,
	.current_advertise_get = tps6598x_current_advertise_get,
	.current_advertise_set = tps6598x_current_advertise_set,
	.port_mode_get = tps6598x_port_mode_get,
	.port_mode_set = tps6598x_port_mode_set,
	.dump_regs = tps6598x_dump_regs
};


static int tps6598x_usb_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret = 0;
	int irq = 0;

	tps6598x_data = devm_kzalloc(dev, sizeof(*tps6598x_data), GFP_KERNEL);
	if (!tps6598x_data)
		return -ENOMEM;

	tps6598x_data->client = client;
	i2c_set_clientdata(client, tps6598x_data);

	tps6598x_data->gpio_reset = of_get_named_gpio(dev->of_node, "reset", 0);
	if (!gpio_is_valid(tps6598x_data->gpio_reset))
		dev_warn(&client->dev, "failed to get Reset GPIO\n");
	else {
		ret = devm_gpio_request(&client->dev, tps6598x_data->gpio_reset, "tps6598x_reset");
		if (ret < 0)
			dev_warn(&client->dev, "failed to request Reset GPIO, ret = %d\n", ret);
	}

	tps6598x_data->gpio_int = of_get_named_gpio(dev->of_node, "i2c-irqz", 0);
	if (!gpio_is_valid(tps6598x_data->gpio_int)) {
		dev_err(&client->dev, "failed to get IRQz GPIO\n");
		ret = -EINVAL;
		goto err_interrupt;
	}

	ret = devm_gpio_request(&client->dev, tps6598x_data->gpio_int, "tps6598x_irqz");
	if (ret < 0) {
		dev_warn(&client->dev, "failed to request IRQz GPIO, ret = %d\n", ret);
		goto err_interrupt;
	}

	tps6598x_data->irqz_int  = gpio_to_irq(tps6598x_data->gpio_int);
	if (tps6598x_data->irqz_int < 0) {
		dev_err(&client->dev, "failed to translate ID_OUT GPIO to IRQ\n");
		goto err_irq;
	}

	tps6598x_data->type_c_psy.name = "typec";
	tps6598x_data->type_c_psy.get_property = tps6598x_typec_get_property;
	tps6598x_data->type_c_psy.properties = tps6598x_typec_properties;
	tps6598x_data->type_c_psy.num_properties = ARRAY_SIZE(tps6598x_typec_properties);

	ret = power_supply_register(tps6598x_data->dev,
				&tps6598x_data->type_c_psy);
	if (ret < 0) {
		pr_err("Unable to register type_c_psy ret=%d\n", ret);
		goto err_irq;
	}

	tps6598x_data->port_mode = TYPEC_MODE_ACCORDING_TO_PROT;

	INIT_WORK(&tps6598x_data->tps6598x_work, tps6598x_int_work);
	ret = request_irq(tps6598x_data->irqz_int, tps6598x_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "tps6598x_usb",
			tps6598x_data);
	if (ret) {
		pr_err("%s: request_irq error, ret=%d\n", __func__, ret);
		irq = -1;
		goto err_req_irq;
	}

	enable_irq_wake(tps6598x_data->irqz_int);

	schedule_work(&tps6598x_data->tps6598x_work);

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		tps6598x_data->tps6598x_instance = devm_dual_role_instance_register(dev,
			&tps6598x_desc);
	}

	ret = add_typec_device(&tps6598x_data->client->dev, &tps6598x_ops);

	tps6598x_set_bcenabled(1);

	return ret;

err_req_irq:
	free_irq(tps6598x_data->irqz_int, tps6598x_data);
err_irq:
err_interrupt:
	return ret;

}

static int tps6598x_usb_remove(struct i2c_client *client)
{
	free_irq(tps6598x_data->irqz_int, tps6598x_data);

	if (tps6598x_data->tps6598x_instance)
		devm_dual_role_instance_unregister(tps6598x_data->dev,
			tps6598x_data->tps6598x_instance);

	return 0;
}

static const struct i2c_device_id tps6598x_i2c_id[] = {
	{ "tps65986", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6598x_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id tps6598x_id_table[] = {
	{ .compatible = "ti,tps6598x" },
	{}
};
MODULE_DEVICE_TABLE(of, tps6598x_id_table);
#endif

static struct i2c_driver tps6598x_usb_driver = {
	.probe		= tps6598x_usb_probe,
	.remove		= tps6598x_usb_remove,
	.driver		= {
		.name	= "tps6598x_usb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tps6598x_id_table),
	},
	.id_table = tps6598x_i2c_id,
};

module_i2c_driver(tps6598x_usb_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_DESCRIPTION("tps6598x USB Type C and PD controller driver");
MODULE_LICENSE("GPL");
