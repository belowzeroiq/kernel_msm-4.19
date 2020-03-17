// SPDX-License-Identifier: GPL-2.0
/* TI ADS129X chip family driver
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>

/* Commands */
#define ADS129X_CMD_NOP		0x00
#define ADS129X_CMD_WAKEUP	0x02
#define ADS129X_CMD_STANDBY	0x04
#define ADS129X_CMD_RESET	0x06
#define ADS129X_CMD_START	0x08
#define ADS129X_CMD_STOP	0x0a
#define ADS129X_CMD_RDATAC	0x10
#define ADS129X_CMD_SDATAC	0x11
#define ADS129X_CMD_RDATA	0x12
#define ADS129X_CMD_RREG	0x20
#define ADS129X_CMD_WREG	0x40

/* Registers */
#define ADS129X_DEV_ID		0x00
#define ADS129X_CFG_1		0x01
#define ADS129X_CFG_2		0x02
#define ADS129X_CFG_3		0x03
#define ADS129X_LEAD_OFF	0x04
#define ADS129X_CH_1_SET	0x05
#define ADS129X_CH_2_SET	0x06
#define ADS129X_CH_3_SET	0x07
#define ADS129X_CH_4_SET	0x08
#define ADS1296_8_CH_5_SET	0x09
#define ADS1296_8_CH_6_SET	0x0a
#define ADS1298_CH_7_SET	0x0b
#define ADS1298_CH_8_SET	0x0c
#define ADS1298_RLD_SNS_P	0x0d
#define ADS1298_RLD_SNS_N	0x0e
#define ADS1298_LOFF_SNS_P	0x0f
#define ADS1298_LOFF_SNS_N	0x10
#define ADS1298_LOFF_FLIP	0x11
#define ADS1298_LOFF_STAT_P	0x12
#define ADS1298_LOFF_STAT_N	0x13
#define ADS129X_GPIO_CFG	0x14
#define ADS129X_PACE		0x15
#define ADS129X_RESP		0x16
#define ADS129X_CFG_4		0x17
#define ADS129X_WCT1		0x18
#define ADS129X_WCT2		0x19

#define ADS129X_MAX_CHANNELS	8

#define ADS129X_CHAN_PWR_DN	BIT(7)
#define ADS129X_PWR_DN_SHIFT	7
#define ADS129X_GAIN_MASK	GENMASK(4, 0)
#define ADS129X_NUM_GAIN_VAL	7
#define ADS129X_GAIN_SHIFT	4

enum ads129x_device_id {
	ADS1294_DEV_ID = 0x80,
	ADS1296_DEV_ID,
	ADS1298_DEV_ID,
	ADS1294R_DEV_ID = 0xC0,
	ADS1296R_DEV_ID,
	ADS1298R_DEV_ID,
};

enum ads129x_id {
	ADS1294_ID,
	ADS1294R_ID,
	ADS1296_ID,
	ADS1296R_ID,
	ADS1298_ID,
	ADS1298R_ID,
};

struct ads129x_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	int device_id;
};

struct ads129x_private {
	const struct ads129x_chip_info	*chip_info;
	struct gpio_desc *reset_gpio;
	struct spi_device *spi;
	struct iio_trigger *trig;
	struct mutex lock;
	int lead_off_stat_p;
	int lead_off_stat_n;
	int irq;
	u8 data[3] ____cacheline_aligned;
	u8 stream_data[27] ____cacheline_aligned;
};

static int ads129x_gain[ADS129X_NUM_GAIN_VAL] = {6000000, 1000000, 2000000,
						 3000000, 4000000, 8000000,
						 12000000};

#define ADS129X_CHAN(index)					\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)		\
			      | BIT(IIO_CHAN_INFO_ENABLE)	\
			      | BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 32,					\
		.storagebits = 32,				\
	},							\
}

static const struct iio_chan_spec ads1294_channels[] = {
	ADS129X_CHAN(0),
	ADS129X_CHAN(1),
	ADS129X_CHAN(2),
	ADS129X_CHAN(3),
};

static const struct iio_chan_spec ads1296_channels[] = {
	ADS129X_CHAN(0),
	ADS129X_CHAN(1),
	ADS129X_CHAN(2),
	ADS129X_CHAN(3),
	ADS129X_CHAN(4),
	ADS129X_CHAN(5),
};

static const struct iio_chan_spec ads1298_channels[] = {
	ADS129X_CHAN(0),
	ADS129X_CHAN(1),
	ADS129X_CHAN(2),
	ADS129X_CHAN(3),
	ADS129X_CHAN(4),
	ADS129X_CHAN(5),
	ADS129X_CHAN(6),
	ADS129X_CHAN(7),
};

static const struct ads129x_chip_info ads129x_chip_info_tbl[] = {
	[ADS1294_ID] = {
		.channels = ads1294_channels,
		.num_channels = ARRAY_SIZE(ads1294_channels),
		.device_id = ADS1294_DEV_ID,
	},
	[ADS1294R_ID] = {
		.channels = ads1294_channels,
		.num_channels = ARRAY_SIZE(ads1294_channels),
		.device_id = ADS1294R_DEV_ID,
	},
	[ADS1296_ID] = {
		.channels = ads1296_channels,
		.num_channels = ARRAY_SIZE(ads1296_channels),
		.device_id = ADS1296_DEV_ID,
	},
	[ADS1296R_ID] = {
		.channels = ads1296_channels,
		.num_channels = ARRAY_SIZE(ads1296_channels),
		.device_id = ADS1296R_DEV_ID,
	},
	[ADS1298_ID] = {
		.channels = ads1298_channels,
		.num_channels = ARRAY_SIZE(ads1298_channels),
		.device_id = ADS1298_DEV_ID,
	},
	[ADS1298R_ID] = {
		.channels = ads1298_channels,
		.num_channels = ARRAY_SIZE(ads1298_channels),
		.device_id = ADS1298R_DEV_ID,
	},
};


static ssize_t ads129x_show_lead_p_status(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ads129x_private *priv = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "0x%X\n", priv->lead_off_stat_p);
}

static ssize_t ads129x_show_lead_n_status(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ads129x_private *priv = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "0x%X\n", priv->lead_off_stat_n);
}

static IIO_DEVICE_ATTR(in_lead_sensp_status, S_IRUGO,
		       ads129x_show_lead_p_status, NULL, 0);
static IIO_DEVICE_ATTR(in_lead_sensn_status, S_IRUGO,
		       ads129x_show_lead_n_status, NULL, 0);

static struct attribute *ads129x_attributes[] = {
	&iio_dev_attr_in_lead_sensp_status.dev_attr.attr,
	&iio_dev_attr_in_lead_sensn_status.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads129x_attribute_group = {
	.attrs = ads129x_attributes,
};

static int ads129x_write_cmd(struct iio_dev *indio_dev, u8 command)
{
	struct ads129x_private *priv = iio_priv(indio_dev);

	priv->data[0] = command;

	return spi_write(priv->spi, &priv->data[0], 1);
}

static int ads129x_write_reg(struct iio_dev *indio_dev, u8 reg, u8 data)
{
	struct ads129x_private *priv = iio_priv(indio_dev);

	priv->data[0] = ADS129X_CMD_WREG | reg;
	priv->data[1] = 1;
	priv->data[2] = data;

	return spi_write(priv->spi, &priv->data[0], 3);
}

static int ads129x_reset(struct iio_dev *indio_dev)
{
	struct ads129x_private *priv = iio_priv(indio_dev);

	if (priv->reset_gpio) {
		gpiod_set_value(priv->reset_gpio, 0);
		udelay(200);
		gpiod_set_value(priv->reset_gpio, 1);
	} else {
		return ads129x_write_cmd(indio_dev, ADS129X_CMD_RESET);
	}

	return 0;
};

static int ads129x_read_stream(struct iio_dev *indio_dev)
{
	struct ads129x_private *priv = iio_priv(indio_dev);
	int ret;

	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->stream_data[0],
			.len = 1,
			.cs_change = 1,
		}, {
			.tx_buf = &priv->stream_data[1],
			.rx_buf = &priv->stream_data[1],
			.len = 27,
		},
	};

	ads129x_write_cmd(indio_dev, ADS129X_CMD_SDATAC);
	priv->stream_data[0] = ADS129X_CMD_RDATA;
	memset(&priv->stream_data[1], ADS129X_CMD_NOP,
	       sizeof(priv->stream_data) - 1);

	ret = spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	priv->lead_off_stat_p = (priv->stream_data[1] & 0xf) |
				(priv->stream_data[2] & 0xf0) << 4;
	priv->lead_off_stat_n = (priv->stream_data[2] & 0xf) |
				(priv->stream_data[3] & 0xf0) << 4;

	ads129x_write_cmd(indio_dev, ADS129X_CMD_RDATAC);

	return ret;
}

static int ads129x_read_channel(struct iio_dev *indio_dev, unsigned int chan)
{
	struct ads129x_private *priv = iio_priv(indio_dev);
	int ret;

	ret = ads129x_read_stream(indio_dev);
	if (ret)
		return ret;

	return priv->stream_data[4 + chan] | priv->stream_data[5 + chan] << 8 |
	       priv->stream_data[6 + chan] << 16;
}

static int ads129x_read_reg(struct iio_dev *indio_dev, u8 reg)
{
	struct ads129x_private *priv = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->data[0],
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = &priv->data[2],
			.rx_buf = &priv->data[2],
			.len = 4,
		},
	};

	ads129x_write_cmd(indio_dev, ADS129X_CMD_SDATAC);

	priv->data[0] = ADS129X_CMD_RREG | reg;
	priv->data[1] = 1;
	memset(&priv->data[2], ADS129X_CMD_NOP, sizeof(priv->data) - 1);

	ret = spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	ads129x_write_cmd(indio_dev, ADS129X_CMD_RDATAC);

	return priv->data[2];
}

static int ads129x_update_reg(struct iio_dev *indio_dev, u8 reg,
			      u8 mask, u8 val)
{
	int ret;
	int tmp;

	ret = ads129x_read_reg(indio_dev, reg);
	if (ret < 0)
		return ret;

	tmp = ret & ~mask;
	tmp |= val & mask;

	return ads129x_write_reg(indio_dev, reg, val);
}

static int ads129x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ads129x_private *priv = iio_priv(indio_dev);
	int ret, i;
	int chan_power, gain_setting = 0;

	mutex_lock(&priv->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val)
			chan_power = 0x0;
		else
			chan_power = ADS129X_CHAN_PWR_DN;

		ret = ads129x_update_reg(indio_dev,
					ADS129X_CH_1_SET + chan->channel,
					ADS129X_CHAN_PWR_DN, chan_power);
		break;

	case IIO_CHAN_INFO_SCALE:
		if (val > ads129x_gain[6] || val < ads129x_gain[1]) {
			ret = -EINVAL;
			goto error_out;
		}

		for (i = 0; i < ADS129X_NUM_GAIN_VAL; i++) {
			if (val > ads129x_gain[i] &&
			    val < ads129x_gain[i + 1])
				gain_setting = i;
		}

		ret = ads129x_update_reg(indio_dev,
					ADS129X_CH_1_SET + chan->channel,
					ADS129X_GAIN_MASK, gain_setting);
		break;
	default:
		ret = -EINVAL;
		break;
	}

error_out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads129x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct ads129x_private *priv = iio_priv(indio_dev);
	int ret;

	mutex_lock(&priv->lock);
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ads129x_read_channel(indio_dev, chan->channel);
		if (ret < 0)
			goto out;

		*val = ret;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_ENABLE:
		ret = ads129x_read_reg(indio_dev,
				       ADS129X_CH_1_SET + chan->channel);
		if (ret < 0)
			goto out;

		*val = (ret & ~ADS129X_CHAN_PWR_DN) >> ADS129X_PWR_DN_SHIFT;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		ret = ads129x_read_reg(indio_dev,
				       ADS129X_CH_1_SET + chan->channel);
		if (ret < 0)
			goto out;

		*val = ads129x_gain[(ret & ADS129X_GAIN_MASK) >> ADS129X_GAIN_SHIFT];
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct iio_info ads129x_info = {
	.read_raw = &ads129x_read_raw,
	.write_raw = &ads129x_write_raw,
	.attrs = &ads129x_attribute_group,
};

static irqreturn_t ads129x_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads129x_private *priv = iio_priv(indio_dev);
	u32 buffer[ADS129X_MAX_CHANNELS + sizeof(s64)/sizeof(u16)];
	int ret, i, j = 0;

	ret = ads129x_read_stream(indio_dev);
	if (ret)
		goto irq_out;

	for (i = 0; i < indio_dev->masklength; i++) {
		if (!test_bit(i, indio_dev->active_scan_mask))
			continue;
		buffer[j] = priv->stream_data[4 + i] |
			    priv->stream_data[5 + i] << 8 |
			    priv->stream_data[6 + i] << 16;
		j++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buffer,
			pf->timestamp);

	iio_trigger_notify_done(indio_dev->trig);

irq_out:
	return IRQ_HANDLED;
}

static const struct iio_trigger_ops ads129x_trigger_ops = {
};


static int ads129x_probe(struct spi_device *spi)
{
	struct ads129x_private *ads129x_priv;
	struct iio_dev *indio_dev;
	const struct spi_device_id *spi_id = spi_get_device_id(spi);
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ads129x_priv));
	if (indio_dev == NULL)
		return -ENOMEM;

	ads129x_priv = iio_priv(indio_dev);

	ads129x_priv->reset_gpio = devm_gpiod_get_optional(&spi->dev,
						   "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ads129x_priv->reset_gpio))
		dev_info(&spi->dev, "Reset GPIO not defined\n");

	ads129x_priv->chip_info = &ads129x_chip_info_tbl[spi_id->driver_data];

	spi_set_drvdata(spi, indio_dev);

	ads129x_priv->spi = spi;

	indio_dev->name = spi_id->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads129x_priv->chip_info->channels;
	indio_dev->num_channels = ads129x_priv->chip_info->num_channels;
	indio_dev->info = &ads129x_info;

	mutex_init(&ads129x_priv->lock);

	if (spi->irq > 0) {
		ads129x_priv->irq = spi->irq;

		ads129x_priv->trig = devm_iio_trigger_alloc(&spi->dev,
						   "%s-dev%d",
						   indio_dev->name,
						   indio_dev->id);
		if (!ads129x_priv->trig) {
			dev_err(&spi->dev, "Unable to allocate IIO trigger\n");
			ret = -ENOMEM;
			return ret;
		}

		iio_trigger_set_drvdata(ads129x_priv->trig, indio_dev);

		ads129x_priv->trig->ops = &ads129x_trigger_ops;
		ads129x_priv->trig->dev.parent = &spi->dev;

		ret = iio_trigger_register(ads129x_priv->trig);
		if (ret) {
			dev_err(&spi->dev, "Unable to register IIO trigger\n");
			return ret;
		}

		ret = devm_request_threaded_irq(&spi->dev, ads129x_priv->irq,
						iio_trigger_generic_data_rdy_poll,
						NULL,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						spi_id->name,
						ads129x_priv->trig);
		if (ret) {
			dev_err(&spi->dev, "Unable to request IRQ\n");
			return ret;
		}
	}

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      iio_pollfunc_store_time,
					      ads129x_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev, "iio triggered buffer setup failed\n");
		return ret;
	}

	ads129x_reset(indio_dev);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ads129x_id[] = {
	{ "ads1294", ADS1294_ID },
	{ "ads1294r", ADS1294R_ID },
	{ "ads1296", ADS1296_ID },
	{ "ads1296r", ADS1296R_ID },
	{ "ads1298", ADS1298_ID },
	{ "ads1298r", ADS1298R_ID },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads129x_id);

static const struct of_device_id ads129x_of_table[] = {
	{ .compatible = "ti,ads1294" },
	{ .compatible = "ti,ads1294r" },
	{ .compatible = "ti,ads1296" },
	{ .compatible = "ti,ads1296r" },
	{ .compatible = "ti,ads1298" },
	{ .compatible = "ti,ads1298r" },
	{ },
};
MODULE_DEVICE_TABLE(of, ads129x_of_table);

static struct spi_driver ads129x_driver = {
	.driver = {
		.name	= "ads129x",
		.of_match_table = ads129x_of_table,
	},
	.probe		= ads129x_probe,
	.id_table	= ads129x_id,
};
module_spi_driver(ads129x_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TI ADS129X Biometric Device");
MODULE_LICENSE("GPL v2");
