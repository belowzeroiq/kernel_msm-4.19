// SPDX-License-Identifier: GPL-2.0
// TLV320ADCX140 Sound driver
// Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "tlv320adcx140.h"

/* The registers can be accessed via
 * cat /sys/class/i2c-adapter/i2c-2/2-00xx/registers
 * And written through echo for example
 * echo "SW_RESET 0x01" > /sys/class/i2c-adapter/i2c-2/2-00xx/registers
 */
struct tlv320adc5140_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} tlv320adc5140_regs[] = {
	{ "PAGE_SELECT", ADCX140_PAGE_SELECT, 1 },
	{ "SW_RESET", ADCX140_SW_RESET, 1 },
	{ "SHDN_CFG", ADCX140_SHDN_CFG, 1 },
	{ "ASI_CFG0", ADCX140_ASI_CFG0, 1 },
	{ "ASI_CFG1", ADCX140_ASI_CFG1, 1 },
	{ "ASI_CFG2", ADCX140_ASI_CFG2, 1 },
	{ "ASI_CH1", ADCX140_ASI_CH1, 1 },
	{ "ASI_CH2", ADCX140_ASI_CH2, 1 },
	{ "ASI_CH3", ADCX140_ASI_CH3, 1 },
	{ "ASI_CH4", ADCX140_ASI_CH4, 1 },
	{ "ASI_CH5", ADCX140_ASI_CH5, 1 },
	{ "ASI_CH6", ADCX140_ASI_CH6, 1 },
	{ "ASI_CH7", ADCX140_ASI_CH7, 1 },
	{ "ASI_CH8", ADCX140_ASI_CH8, 1 },
	{ "MST_CFG0", ADCX140_MST_CFG0, 1 },
	{ "MST_CFG1", ADCX140_MST_CFG1, 1 },
	{ "ASI_STS", ADCX140_ASI_STS, 0 },
	{ "CLK_SRC", ADCX140_CLK_SRC, 1 },
	{ "PDMCLK_CFG", ADCX140_PDMCLK_CFG, 1 },
	{ "PDM_CFG", ADCX140_PDM_CFG, 1 },
	{ "GPIO_CFG0", ADCX140_GPIO_CFG0, 1 },
	{ "GPO_CFG1", ADCX140_GPO_CFG1, 1 },
	{ "GPO_CFG2", ADCX140_GPO_CFG2, 1 },
	{ "GPO_CFG3", ADCX140_GPO_CFG3, 1 },
	{ "GPO_CFG4", ADCX140_GPO_CFG4, 1 },
	{ "GPO_VAL", ADCX140_GPO_VAL, 1 },
	{ "GPIO_MON", ADCX140_GPIO_MON, 1 },
	{ "GPI_CFG0", ADCX140_GPI_CFG0, 1 },
	{ "GPI_CFG1", ADCX140_GPI_CFG1, 1 },
	{ "GPI_MON", ADCX140_GPI_MON, 1 },
	{ "INT_CFG", ADCX140_INT_CFG, 1 },
	{ "INT_MSK0", ADCX140_INT_MASK0, 1 },
	{ "INT_LTCH0", ADCX140_INT_LTCH0, 1 },
	{ "BIAS_CFG", ADCX140_BIAS_CFG, 1 },
	{ "CH1_CFG0", ADCX140_CH1_CFG0, 1 },
	{ "CH1_CFG1", ADCX140_CH1_CFG1, 1 },
	{ "CH1_CFG2", ADCX140_CH1_CFG2, 1 },
	{ "CH1_CFG3", ADCX140_CH1_CFG3, 1 },
	{ "CH1_CFG4", ADCX140_CH1_CFG4, 1 },
	{ "CH2_CFG0", ADCX140_CH2_CFG0, 1 },
	{ "CH2_CFG1", ADCX140_CH2_CFG1, 1 },
	{ "CH2_CFG2", ADCX140_CH2_CFG2, 1 },
	{ "CH2_CFG3", ADCX140_CH2_CFG3, 1 },
	{ "CH2_CFG4", ADCX140_CH2_CFG4, 1 },
	{ "CH3_CFG0", ADCX140_CH3_CFG0, 1 },
	{ "CH3_CFG1", ADCX140_CH3_CFG1, 1 },
	{ "CH3_CFG2", ADCX140_CH3_CFG2, 1 },
	{ "CH3_CFG3", ADCX140_CH3_CFG3, 1 },
	{ "CH3_CFG4", ADCX140_CH3_CFG4 , 1 },
	{ "CH4_CFG0", ADCX140_CH4_CFG0, 1 },
	{ "CH4_CFG1", ADCX140_CH4_CFG1, 1 },
	{ "CH4_CFG2", ADCX140_CH4_CFG2, 1 },
	{ "CH4_CFG3", ADCX140_CH4_CFG3, 1 },
	{ "CH4_CFG4", ADCX140_CH4_CFG4, 1 },
	{ "CH5_CFG2", ADCX140_CH5_CFG2, 1 },
	{ "CH5_CFG3", ADCX140_CH5_CFG3, 1 },
	{ "CH5_CFG4", ADCX140_CH5_CFG4, 1 },
	{ "CH6_CFG2", ADCX140_CH6_CFG2, 1 },
	{ "CH6_CFG3", ADCX140_CH6_CFG3, 1 },
	{ "CH6_CFG4", ADCX140_CH6_CFG4, 1 },
	{ "CH7_CFG2", ADCX140_CH7_CFG2, 1 },
	{ "CH7_CFG3", ADCX140_CH7_CFG3, 1 },
	{ "CH7_CFG4", ADCX140_CH7_CFG4, 1 },
	{ "CH8_CFG2", ADCX140_CH8_CFG2, 1 },
	{ "CH8_CFG3", ADCX140_CH8_CFG3, 1 },
	{ "CH8_CFG4", ADCX140_CH8_CFG4, 1 },
	{ "DSP_CFG0", ADCX140_DSP_CFG0, 1 },
	{ "DSP_CFG1", ADCX140_DSP_CFG1, 1 },
	{ "DRE_CFG0", ADCX140_DRE_CFG0, 1 },
	{ "IN_CH_EN", ADCX140_IN_CH_EN, 1 },
	{ "ASI_OUT_CH_EN", ADCX140_ASI_OUT_CH_EN, 1 },
	{ "PWR_CFG", ADCX140_PWR_CFG, 1 },
	{ "DEV_STS0", ADCX140_DEV_STS0, 0 },
	{ "DEV_STS1", ADCX140_DEV_STS1, 0 },
};

static ssize_t tlv320adc5140_registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
	struct adcx140_priv *data = dev_get_drvdata(dev);

	reg_count = sizeof(tlv320adc5140_regs) / sizeof(tlv320adc5140_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, tlv320adc5140_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       tlv320adc5140_regs[i].name,
			       read_buf);
	}
	return n;
}

static ssize_t tlv320adc5140_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct adcx140_priv *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(tlv320adc5140_regs) / sizeof(tlv320adc5140_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tlv320adc5140_regs[i].name)) {
			if (tlv320adc5140_regs[i].writeable == 1) {
				error = regmap_write(data->regmap, tlv320adc5140_regs[i].reg, value);
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
		tlv320adc5140_registers_show, tlv320adc5140_registers_store);

static struct attribute *tlv320adc5140_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group tlv320adc5140_attr_group = {
	.attrs = tlv320adc5140_attrs,
};

int tlv320adc5140_init_debug(struct adcx140_priv* adc5140)
{
	int ret;
	struct adcx140_priv *dbg_adc5140;

	dbg_adc5140 = adc5140;

	ret = sysfs_create_group(&dbg_adc5140->dev->kobj, &tlv320adc5140_attr_group);
	if (ret < 0)
		dev_err(dbg_adc5140->dev, "Failed to create sysfs: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tlv320adc5140_init_debug);

MODULE_DESCRIPTION("ASoC TLV320ADCX140 debug");
MODULE_AUTHOR("Dan Murphy");
MODULE_LICENSE("GPL");
