// SPDX-License-Identifier: GPL-2.0
//
// Firmware loader for the Texas Instruments TAS25XX DSP
// Copyright (C) 2020 Texas Instruments Inc.

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "tas25xx_dsp_loader.h"

static void tas25xx_process_fw_delay(struct tas2562_data *tas2562, u8 *fw_out)
{
	u16 fw_delay = fw_out[2] << 8 | fw_out[3];

	mdelay(fw_delay);
}

static int tas25xx_process_fw_single(struct tas2562_data *tas2562,
				     struct tas25xx_cmd_data *cmd_data,
				     u8 *fw_out)
{
	int num_writes = cpu_to_be16(cmd_data->length);
	int i;
	int ret;
	int offset = 4;
	int reg_data, write_reg;

	for (i = 0; i < num_writes; i++) {
		/* Reset Page to 0 */
		ret = regmap_write(tas2562->regmap, TAS2562_PAGE_CTRL, 0);
		if (ret)
			return ret;

		cmd_data->book = fw_out[offset];
		cmd_data->page = fw_out[offset + 1];
		cmd_data->offset = fw_out[offset + 2];
		reg_data = fw_out[offset + 3];
		offset += 4;

		ret = regmap_write(tas2562->regmap, TAS2562_BOOK_CTRL,
				   cmd_data->book);
		if (ret)
			return ret;

		write_reg = TAS2562_REG(cmd_data->page, cmd_data->offset);
		ret = regmap_write(tas2562->regmap, write_reg, reg_data);
		if (ret)
			return ret;
	}

	return 0;
}

static int tas25xx_process_fw_burst(struct tas2562_data *tas2562,
				    struct tas25xx_cmd_data *cmd_data,
				    u8 *fw_out, int size)
{
	int hdr_size = sizeof(struct tas25xx_cmd_data);
	u8 *data = &fw_out[hdr_size];
	int ret;
	int reg_data = TAS2562_REG(cmd_data->page, cmd_data->offset);

	/* Reset Page to 0 */
	ret = regmap_write(tas2562->regmap, TAS2562_PAGE_CTRL, 0);
	if (ret)
		return ret;

	ret = regmap_write(tas2562->regmap, TAS2562_BOOK_CTRL, cmd_data->book);
	if (ret)
		return ret;

	ret = regmap_bulk_write(tas2562->regmap, reg_data, data,
				cpu_to_be16(cmd_data->length));
	if (ret)
		return ret;

	return 0;
}

static int tas25xx_process_block(struct tas2562_data *tas2562, u8 *data)
{
	struct tas25xx_cmd_data *cmd_data = (struct tas25xx_cmd_data *)data;
	int ret;
	int total_written;
	int hdr_size = sizeof(struct tas25xx_cmd_data);

	switch (cpu_to_be16(cmd_data->cmd_type)) {
	case TAS25XX_CMD_SING_W:
		ret = tas25xx_process_fw_single(tas2562, cmd_data, data);
		if (ret) {
			dev_err(tas2562->dev,
				"Failed to process single write %d\n", ret);
			return ret;
		}

		hdr_size -= 4;
		total_written = cpu_to_be16(cmd_data->length) * 4 + hdr_size;
		break;
	case TAS25XX_CMD_BURST:
		ret = tas25xx_process_fw_burst(tas2562, cmd_data, data,
					       cpu_to_be16(cmd_data->length));
		if (ret) {
			dev_err(tas2562->dev,
				"Failed to process burst write %d\n", ret);
			return ret;
		}
		total_written = cpu_to_be16(cmd_data->length) + hdr_size;
		break;
	case TAS25XX_CMD_DELAY:
		tas25xx_process_fw_delay(tas2562, data);
		total_written = 4;
		break;
	default:
		total_written = 0;
		break;
	};

	return total_written;
}


static int tas25xx_write_program(struct tas2562_data *tas2562, int prog_num)
{
	struct tas25xx_fw_hdr *fw_hdr = tas2562->fw_data->fw_hdr;
	struct tas25xx_program_info *prog_info;
	int offset = sizeof(struct tas25xx_program_info);
	int prog_offset = 0;
	int i;
	int length = 0;

	if (prog_num > fw_hdr->num_programs)
		return -EINVAL;

	if (prog_num) {
		for (i = 0; i < prog_num; i++)
			prog_offset += cpu_to_be32(fw_hdr->prog_size[i]);
	}

	offset += prog_offset;
	prog_info = (struct tas25xx_program_info *)&tas2562->fw_data->prog_info[prog_offset];

	for (i = 0; i < cpu_to_be32(prog_info->blk_data.num_subblocks); i++) {
		length = tas25xx_process_block(tas2562,
					  &tas2562->fw_data->prog_info[offset]);
		if (length < 0)
			return length;

		offset += length;
	}

	/* Reset Book to 0 */
	regmap_write(tas2562->regmap, TAS2562_BOOK_CTRL, 0);

	return 0;
}

static int tas25xx_write_config(struct tas2562_data *tas2562, int config_num)
{
	struct tas25xx_fw_hdr *fw_hdr = tas2562->fw_data->fw_hdr;
	struct tas25xx_config_info *cfg_info;
	int config_offset = 0;
	int i;
	int offset = sizeof(struct tas25xx_config_info);
	int length = 0;

	if (config_num > fw_hdr->num_configs)
		return -EINVAL;

	if (config_num)
		for (i = 0; i < config_num; i++)
			config_offset += cpu_to_be32(fw_hdr->config_size[i]);

	cfg_info = (struct tas25xx_config_info *)&tas2562->fw_data->config_info[config_offset];
	offset += config_offset;

	for (i = 0; i < cpu_to_be32(cfg_info->blk_data.num_subblocks); i++) {
		length = tas25xx_process_block(tas2562,
					&tas2562->fw_data->config_info[offset]);
		if (length < 0)
			return length;

		offset += length;
	}

	/* Reset Book to 0 */
	regmap_write(tas2562->regmap, TAS2562_BOOK_CTRL, 0);

	return 0;
}

static ssize_t write_config_store(struct device *dev,
				struct device_attribute *tas25xx_attr,
				const char *buf, size_t size)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	struct tas25xx_fw_hdr *fw_hdr = tas25xx->fw_data->fw_hdr;
	long configs;
	int ret;

	ret = kstrtol(buf, 10, &configs);
	if (ret != 0)
		return ret;

	if (configs < 1 || configs > cpu_to_be32(fw_hdr->num_configs))
		return -EINVAL;

	configs--;
	ret =  tas25xx_write_config(tas25xx, configs);
	if (!ret)
		ret = size;

	return ret;
}
static DEVICE_ATTR_WO(write_config);

static ssize_t write_program_store(struct device *dev,
				struct device_attribute *tas25xx_attr,
				const char *buf, size_t size)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	struct tas25xx_fw_hdr *fw_hdr = tas25xx->fw_data->fw_hdr;
	long program;
	int ret;

	ret = kstrtol(buf, 10, &program);
	if (ret != 0)
		return ret;

	if (program < 1 || program > cpu_to_be32(fw_hdr->num_programs))
		return -EINVAL;

	program--;
	ret =  tas25xx_write_program(tas25xx, program);
	if (!ret)
		ret = size;

	return ret;
}
static DEVICE_ATTR_WO(write_program);

static ssize_t enable_program_show(struct device *dev,
			      struct device_attribute *tas25xx_attr,
			      char *buf)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	int mode;

	regmap_read(tas25xx->regmap, TAS25XX_DSP_MODE, &mode);

	return sprintf(buf, "0x%X\n", mode);
}

static ssize_t enable_program_store(struct device *dev,
				struct device_attribute *tas25xx_attr,
				const char *buf, size_t size)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	long mode;
	int ret;

	ret = kstrtol(buf, 10, &mode);
	if (ret != 0)
		return ret;

	if (mode)
		ret = regmap_write(tas25xx->regmap, TAS25XX_DSP_MODE, mode);
	else
		ret = regmap_write(tas25xx->regmap, TAS25XX_DSP_MODE, 0);

	if (!ret)
		ret = size;

	return ret;

}
static DEVICE_ATTR_RW(enable_program);

static ssize_t num_configs_show(struct device *dev,
			      struct device_attribute *tas25xx_attr,
			      char *buf)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	struct tas25xx_fw_hdr *fw_hdr = tas25xx->fw_data->fw_hdr;

	return sprintf(buf, "%d\n", cpu_to_be32(fw_hdr->num_configs));
}
static DEVICE_ATTR_RO(num_configs);

static ssize_t num_programs_show(struct device *dev,
			      struct device_attribute *tas25xx_attr,
			      char *buf)
{
	struct tas2562_data *tas25xx = dev_get_drvdata(dev);
	struct tas25xx_fw_hdr *fw_hdr = tas25xx->fw_data->fw_hdr;

	return sprintf(buf, "%d\n", cpu_to_be32(fw_hdr->num_programs));
}
static DEVICE_ATTR_RO(num_programs);

static struct attribute *tas25xx_fw_attrs[] = {
	&dev_attr_num_programs.attr,
	&dev_attr_num_configs.attr,
	&dev_attr_write_config.attr,
	&dev_attr_write_program.attr,
	&dev_attr_enable_program.attr,
	NULL,
};

static const struct attribute_group tas25xx_fw_attr_group = {
	.attrs	= tas25xx_fw_attrs,
};

int tas25xx_init_fw(struct tas2562_data *tas2562, const struct firmware *fw)
{
	u32 total_prog_sz = 0;
	u32 total_config_sz = 0;
	int hdr_size = sizeof(struct tas25xx_fw_hdr);
	int i;
	u8 *out;
	int ret;

	if (!fw->size)
		return -ENODATA;

	out = devm_kzalloc(tas2562->dev, fw->size, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	memcpy(out, &fw->data[0], fw->size);
	tas2562->fw_data = (struct tas25xx_fw_data *)out;

	tas2562->fw_data->fw_hdr = devm_kzalloc(tas2562->dev, hdr_size,
						GFP_KERNEL);
	if (!tas2562->fw_data->fw_hdr)
		return -ENOMEM;

	memcpy(tas2562->fw_data->fw_hdr, &fw->data[0], hdr_size);

	for (i = 0; i < cpu_to_be32(tas2562->fw_data->fw_hdr->num_programs); i++)
		total_prog_sz += cpu_to_be32(tas2562->fw_data->fw_hdr->prog_size[i]);

	for (i = 0; i < cpu_to_be32(tas2562->fw_data->fw_hdr->num_configs); i++)
		total_config_sz += cpu_to_be32(tas2562->fw_data->fw_hdr->config_size[i]);

	tas2562->fw_data->prog_info = devm_kzalloc(tas2562->dev, total_prog_sz,
						   GFP_KERNEL);
	if (!tas2562->fw_data->prog_info)
		return -ENOMEM;

	memcpy(tas2562->fw_data->prog_info, &fw->data[hdr_size], total_prog_sz);

	tas2562->fw_data->config_info = devm_kzalloc(tas2562->dev,
						     total_config_sz,
						     GFP_KERNEL);
	if (!tas2562->fw_data->config_info)
		return -ENOMEM;

	hdr_size += total_prog_sz;
	memcpy(tas2562->fw_data->config_info, &fw->data[hdr_size],
	       total_config_sz);

	/* Create SysFS files */
	ret = device_add_group(tas2562->dev, &tas25xx_fw_attr_group);
	if (ret)
		dev_info(tas2562->dev, "error creating sysfs files\n");

	release_firmware(fw);

	return 0;
}

