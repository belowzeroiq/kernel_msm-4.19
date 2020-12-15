// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DP83TC811 PHY
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define DP83TC811_PHY_ID	0x2000a253
#define DP83TC812_PHY_ID	0x2000a270
#define DP83TC813_PHY_ID	0x2000a210
#define DP83TC814_PHY_ID	0x2000a260
#define DP83TC815_PHY_ID	0x2000a200
#define DP83811_DEVADDR		0x1f
#define DP83811_PMD_DEVADDR	0x1

#define MII_DP83811_SGMII_CTRL	0x09
#define MII_DP83811_INT_STAT1	0x12
#define MII_DP83811_INT_STAT2	0x13
#define MII_DP83811_INT_STAT3	0x18
#define MII_DP83811_RESET_CTRL	0x1f
#define DP83811_PMD_CTRL	0x834

#define DP83811_HW_RESET	BIT(15)
#define DP83811_SW_RESET	BIT(14)

/* INT_STAT1 bits */
#define DP83811_RX_ERR_HF_INT_EN	BIT(0)
#define DP83811_MS_TRAINING_INT_EN	BIT(1)
#define DP83811_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83811_ESD_EVENT_INT_EN	BIT(3)
#define DP83811_WOL_INT_EN		BIT(4)
#define DP83811_LINK_STAT_INT_EN	BIT(5)
#define DP83811_ENERGY_DET_INT_EN	BIT(6)
#define DP83811_LINK_QUAL_INT_EN	BIT(7)

/* INT_STAT2 bits */
#define DP83811_JABBER_DET_INT_EN	BIT(0)
#define DP83811_POLARITY_INT_EN		BIT(1)
#define DP83811_SLEEP_MODE_INT_EN	BIT(2)
#define DP83811_OVERTEMP_INT_EN		BIT(3)
#define DP83811_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83811_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83811_LPS_INT_EN	BIT(0)
#define DP83811_NO_FRAME_INT_EN	BIT(3)
#define DP83811_POR_DONE_INT_EN	BIT(4)

#define MII_DP83811_RXSOP1	0x04a5
#define MII_DP83811_RXSOP2	0x04a6
#define MII_DP83811_RXSOP3	0x04a7

/* WoL Registers */
#define MII_DP83811_WOL_CFG	0x04a0
#define MII_DP83811_WOL_STAT	0x04a1
#define MII_DP83811_WOL_DA1	0x04a2
#define MII_DP83811_WOL_DA2	0x04a3
#define MII_DP83811_WOL_DA3	0x04a4

/* WoL bits */
#define DP83811_WOL_MAGIC_EN	BIT(0)
#define DP83811_WOL_SECURE_ON	BIT(5)
#define DP83811_WOL_EN		BIT(7)
#define DP83811_WOL_INDICATION_SEL BIT(8)
#define DP83811_WOL_CLR_INDICATION BIT(11)

/* SGMII CTRL bits */
#define DP83811_TDR_AUTO		BIT(8)
#define DP83811_SGMII_EN		BIT(12)
#define DP83811_SGMII_AUTO_NEG_EN	BIT(13)
#define DP83811_SGMII_TX_ERR_DIS	BIT(14)
#define DP83811_SGMII_SOFT_RESET	BIT(15)

#define DP83811_MASTER_MODE	BIT(14)

const int dp83tc811_feature_array[3] = {
	//ETHTOOL_LINK_MODE_100baseT1_Half_BIT,
	ETHTOOL_LINK_MODE_100baseT1_Full_BIT,
	ETHTOOL_LINK_MODE_TP_BIT,
	ETHTOOL_LINK_MODE_Autoneg_BIT,
};

struct dp83812_init_reg {
	int reg;
	int val;
};
static const struct dp83812_init_reg dp83811_init[] = {
	{0x0475, 0x0008},
	{0x0485, 0x11ff},
	{0x0462, 0x0600},
	{0x010F, 0x0100},
	{0x0410, 0x6000},
	{0x0479, 0x0442},
	{0x0466, 0x8000},
	{0x0107, 0x2605},
	{0x0106, 0xb8bb},
	{0x0116, 0x03CA},
	{0x0114, 0xC00A},
	{0x010b, 0x0700},
	{0x0132, 0x01EE},
	{0x04de, 0x03f0},
	{0x003e, 0x000d},
	{0x0129, 0x009F},
	{0x04d5, 0xFEA4},
	{0x0111, 0x6009},
	{0x04d6, 0x0EA4},
	{0x0120, 0x0067},
	{0x0125, 0x7a56},
	{0x0461, 0x0408},
	{0x0400, 0x1300},
	{0x0403, 0x0030},
	{0x0404, 0x0008},
	{0x048a, 0x0d02},
	{0x048b, 0x350f},
	{0x048c, 0x0033},
	{0x048d, 0x010d},
	{0x0475, 0x0000},
};

static const struct dp83812_init_reg dp83811_master_init[] = {
	{0x0121, 0x1500},
	{0x0122, 0x1000},
	{0x04D4, 0x7522},
	{0x0130, 0xc720},
	{0x0126, 0x0515},
	{0x0119, 0x00a4},
	{0x0109, 0x095d},
	{0x010e, 0x3219},
	{0x010c, 0x1996},
};

static const struct dp83812_init_reg dp83811_slave_init[] = {
	{0x0121, 0x1500},
	{0x0122, 0x1450},
	{0x04D4, 0x7322},
	{0x0130, 0xC780},
	{0x0126, 0x0495},
	{0x0115, 0x8ac8},
	{0x0109, 0x095d},
	{0x010e, 0xfafb},
	{0x010c, 0x19fa},
	{0x0101, 0x2082},
};

static const struct dp83812_init_reg dp8381x_master_init[] = {
	{0x523, 0x001},
	{0x800, 0xf864},
	{0x803, 0x1552},
	{0x804, 0x1a66},
	{0x805, 0x1f7B},
	{0x81f, 0x2a88},
	{0x825, 0x40e5},
	{0x82b, 0x7f3f},
	{0x830, 0x543},
	{0x836, 0x5008},
	{0x83A, 0x8e0},
	{0x83B, 0x845},
	{0x83e, 0x445},
	{0x855, 0x9B9a},
	{0x85f, 0x2010},
	{0x860, 0x6040},
	{0x86c, 0x1333},
	{0x86b, 0x3e10},
	{0x872, 0x88c0},
	{0x873, 0x003},
	{0x879, 0x00f},
	{0x87b, 0x070},
	{0x87c, 0x002},
	{0x897, 0x03f},
	{0x89e, 0x0aa},
	{0x510, 0x00f},
	{0x01f, 0x4000},
	{0x523, 0x000},
};

static const struct dp83812_init_reg dp8381x_slave_init[] = {
	{0x523, 0x001},
	{0x803, 0x1B52},
	{0x804, 0x2166},
	{0x805, 0x277B},
	{0x827, 0x3000},
	{0x830, 0x543},
	{0x83a, 0x020},
	{0x83c, 0x001},
	{0x855, 0x9B9A},
	{0x85f, 0x2010},
	{0x860, 0x6040},
	{0x86c, 0x333},
	{0x872, 0x88C0},
	{0x873, 0x021},
	{0x879, 0x00f},
	{0x87b, 0x070},
	{0x87c, 0x002},
	{0x897, 0x03f},
	{0x89e, 0x0a2},
	{0x510, 0x00f},
	{0x01f, 0x4000},
	{0x523, 0x000},
};

struct dp83811_private {
	int chip;
	bool is_master;
};


static int dp83811_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write(phydev, MII_DP83811_RESET_CTRL,
				DP83811_HW_RESET);
	else
		ret = phy_write(phydev, MII_DP83811_RESET_CTRL,
				DP83811_SW_RESET);

	if (ret)
		return ret;

	mdelay(100);

	return 0;
}

static irqreturn_t dp83811_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	/* The INT_STAT registers 1, 2 and 3 are holding the interrupt status
	 * in the upper half (15:8), while the lower half (7:0) is used for
	 * controlling the interrupt enable state of those individual interrupt
	 * sources. To determine the possible interrupt sources, just read the
	 * INT_STAT* register and use it directly to know which interrupts have
	 * been enabled previously or not.
	 */
	irq_status = phy_read(phydev, MII_DP83811_INT_STAT1);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83811_INT_STAT2);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	irq_status = phy_read(phydev, MII_DP83811_INT_STAT3);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}
	if (irq_status & ((irq_status & GENMASK(7, 0)) << 8))
		goto trigger_machine;

	return IRQ_NONE;

trigger_machine:
	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int dp83811_set_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	u16 value;

	if (wol->wolopts & (WAKE_MAGIC | WAKE_MAGICSECURE)) {
		mac = (const u8 *)ndev->dev_addr;

		if (!is_valid_ether_addr(mac))
			return -EINVAL;

		/* MAC addresses start with byte 5, but stored in mac[0].
		 * 811 PHYs store bytes 4|5, 2|3, 0|1
		 */
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA1,
			      (mac[1] << 8) | mac[0]);
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA2,
			      (mac[3] << 8) | mac[2]);
		phy_write_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_DA3,
			      (mac[5] << 8) | mac[4]);

		value = phy_read_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG);
		if (wol->wolopts & WAKE_MAGIC)
			value |= DP83811_WOL_MAGIC_EN;
		else
			value &= ~DP83811_WOL_MAGIC_EN;

		if (wol->wolopts & WAKE_MAGICSECURE) {
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP1,
				      (wol->sopass[1] << 8) | wol->sopass[0]);
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP2,
				      (wol->sopass[3] << 8) | wol->sopass[2]);
			phy_write_mmd(phydev, DP83811_DEVADDR,
				      MII_DP83811_RXSOP3,
				      (wol->sopass[5] << 8) | wol->sopass[4]);
			value |= DP83811_WOL_SECURE_ON;
		} else {
			value &= ~DP83811_WOL_SECURE_ON;
		}

		/* Clear any pending WoL interrupt */
		phy_read(phydev, MII_DP83811_INT_STAT1);

		value |= DP83811_WOL_EN | DP83811_WOL_INDICATION_SEL |
			 DP83811_WOL_CLR_INDICATION;

		return phy_write_mmd(phydev, DP83811_DEVADDR,
				     MII_DP83811_WOL_CFG, value);
	} else {
		return phy_clear_bits_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_WOL_CFG, DP83811_WOL_EN);
	}

}

static void dp83811_get_wol(struct phy_device *phydev,
			    struct ethtool_wolinfo *wol)
{
	u16 sopass_val;
	int value;

	wol->supported = (WAKE_MAGIC | WAKE_MAGICSECURE);
	wol->wolopts = 0;

	value = phy_read_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG);

	if (value & DP83811_WOL_MAGIC_EN)
		wol->wolopts |= WAKE_MAGIC;

	if (value & DP83811_WOL_SECURE_ON) {
		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP1);
		wol->sopass[0] = (sopass_val & 0xff);
		wol->sopass[1] = (sopass_val >> 8);

		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP2);
		wol->sopass[2] = (sopass_val & 0xff);
		wol->sopass[3] = (sopass_val >> 8);

		sopass_val = phy_read_mmd(phydev, DP83811_DEVADDR,
					  MII_DP83811_RXSOP3);
		wol->sopass[4] = (sopass_val & 0xff);
		wol->sopass[5] = (sopass_val >> 8);

		wol->wolopts |= WAKE_MAGICSECURE;
	}

	/* WoL is not enabled so set wolopts to 0 */
	if (!(value & DP83811_WOL_EN))
		wol->wolopts = 0;
}

static int dp83811_config_intr(struct phy_device *phydev)
{
	int misr_status, err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = dp83811_handle_interrupt(phydev);
		if (err)
			return err;

		misr_status = phy_read(phydev, MII_DP83811_INT_STAT1);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_RX_ERR_HF_INT_EN |
				DP83811_MS_TRAINING_INT_EN |
				DP83811_ANEG_COMPLETE_INT_EN |
				DP83811_ESD_EVENT_INT_EN |
				DP83811_WOL_INT_EN |
				DP83811_LINK_STAT_INT_EN |
				DP83811_ENERGY_DET_INT_EN |
				DP83811_LINK_QUAL_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT1, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83811_INT_STAT2);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_JABBER_DET_INT_EN |
				DP83811_POLARITY_INT_EN |
				DP83811_SLEEP_MODE_INT_EN |
				DP83811_OVERTEMP_INT_EN |
				DP83811_OVERVOLTAGE_INT_EN |
				DP83811_UNDERVOLTAGE_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT2, misr_status);
		if (err < 0)
			return err;

		misr_status = phy_read(phydev, MII_DP83811_INT_STAT3);
		if (misr_status < 0)
			return misr_status;

		misr_status |= (DP83811_LPS_INT_EN |
				DP83811_NO_FRAME_INT_EN |
				DP83811_POR_DONE_INT_EN);

		err = phy_write(phydev, MII_DP83811_INT_STAT3, misr_status);

	} else {
		err = phy_write(phydev, MII_DP83811_INT_STAT1, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83811_INT_STAT2, 0);
		if (err < 0)
			return err;

		err = phy_write(phydev, MII_DP83811_INT_STAT3, 0);
		if (err < 0)
			return err;

		err = dp83811_handle_interrupt(phydev);
	}

	return err;
}

static int dp83811_write_seq(struct phy_device *phydev,
			     const struct dp83812_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
	        ret = phy_write_mmd(phydev, DP83811_DEVADDR, init_data[i].reg,
				init_data[i].val);
	        if (ret)
	                return ret;
	}

	return 0;
}

static int dp83811_set_master_slave(struct phy_device *phydev)
{
	int mst_slave_cfg;

	mst_slave_cfg = phy_read_mmd(phydev, DP83811_PMD_DEVADDR,
				     DP83811_PMD_CTRL);
	if (mst_slave_cfg < 0)
		return mst_slave_cfg;

	if (mst_slave_cfg & DP83811_MASTER_MODE) {
		if (phydev->autoneg) {
			phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_PREFERRED;
			phydev->master_slave_set = MASTER_SLAVE_CFG_MASTER_PREFERRED;
		} else {
			phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_FORCE;
			phydev->master_slave_set = MASTER_SLAVE_CFG_MASTER_FORCE;
		}
	} else {
		if (phydev->autoneg) {
			phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_PREFERRED;
			phydev->master_slave_set = MASTER_SLAVE_CFG_SLAVE_PREFERRED;
		} else {
			phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_FORCE;
			phydev->master_slave_set = MASTER_SLAVE_CFG_SLAVE_FORCE;
		}
	}

	return 0;
}

static int dp8381x_chip_init(struct phy_device *phydev)
{
	struct dp83811_private *dp83811 = phydev->priv;
	int err;

	err = dp83811_reset(phydev, true);
	if (err)
		return err;

	if (phydev->phy_id != DP83TC811_PHY_ID) {
		err = phy_write_mmd(phydev, DP83811_DEVADDR, 0x573, 0x101);
		if (err)
			return err;

		if (dp83811->is_master) {
			err = phy_write_mmd(phydev, DP83811_PMD_DEVADDR,
					    DP83811_PMD_CTRL,
					    0x8001 | DP83811_MASTER_MODE);
			if (err)
				return err;

			err = dp83811_write_seq(phydev, dp8381x_master_init,
						ARRAY_SIZE(dp8381x_master_init));
		} else {
			err = phy_write_mmd(phydev, DP83811_PMD_DEVADDR,
					    DP83811_PMD_CTRL, 0x8001);
			if (err)
				return err;

			err = dp83811_write_seq(phydev, dp8381x_slave_init,
						ARRAY_SIZE(dp8381x_slave_init));
		}

	} else {
		err = dp83811_write_seq(phydev, dp83811_init,
					ARRAY_SIZE(dp83811_init));
		if (dp83811->is_master) {
			err = phy_write_mmd(phydev, DP83811_PMD_DEVADDR,
					    DP83811_PMD_CTRL,
					    0x8001 | DP83811_MASTER_MODE);
			if (err)
				return err;

			err = dp83811_write_seq(phydev, dp83811_master_init,
						ARRAY_SIZE(dp83811_master_init));
		} else {
			err = phy_write_mmd(phydev, DP83811_PMD_DEVADDR,
					    DP83811_PMD_CTRL, 0x8001);
			if (err)
				return err;

			err = dp83811_write_seq(phydev, dp83811_slave_init,
						ARRAY_SIZE(dp83811_slave_init));
		}
	}

	err = dp83811_reset(phydev, false);
	if (err)
		return err;

	return 0;
}

static int dp8381x_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return dp83811_set_master_slave(phydev);
}

static int dp83811_config_aneg(struct phy_device *phydev)
{
	int value, err;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII) {
		value = phy_read(phydev, MII_DP83811_SGMII_CTRL);
		if (phydev->autoneg == AUTONEG_ENABLE) {
			err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
					(DP83811_SGMII_AUTO_NEG_EN | value));
			if (err < 0)
				return err;
		} else {
			err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
					(~DP83811_SGMII_AUTO_NEG_EN & value));
			if (err < 0)
				return err;
		}
	}

	return genphy_config_aneg(phydev);
}

static int dp83811_config_init(struct phy_device *phydev)
{
	int value, err;

	value = phy_read(phydev, MII_DP83811_SGMII_CTRL);
	if (phydev->interface == PHY_INTERFACE_MODE_SGMII)
		err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
					(DP83811_SGMII_EN | value));
	else
		err = phy_write(phydev, MII_DP83811_SGMII_CTRL,
				(~DP83811_SGMII_EN & value));
	if (err < 0)

		return err;

	value = DP83811_WOL_MAGIC_EN | DP83811_WOL_SECURE_ON | DP83811_WOL_EN;

	return phy_clear_bits_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG,
				  value);
}

static int dp83811_phy_reset(struct phy_device *phydev)
{
	int err;

	err = phy_write(phydev, MII_DP83811_RESET_CTRL, DP83811_HW_RESET);
	if (err < 0)
		return err;

	return 0;
}

static int dp83811_suspend(struct phy_device *phydev)
{
	int value;

	value = phy_read_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG);

	if (!(value & DP83811_WOL_EN))
		genphy_suspend(phydev);

	return 0;
}

static int dp83811_resume(struct phy_device *phydev)
{
	genphy_resume(phydev);

	phy_set_bits_mmd(phydev, DP83811_DEVADDR, MII_DP83811_WOL_CFG,
			 DP83811_WOL_CLR_INDICATION);

	return 0;
}

static int dp83tc811_get_features(struct phy_device *phydev)
{
	genphy_read_abilities(phydev);

	linkmode_set_bit_array(dp83tc811_feature_array,
			       ARRAY_SIZE(dp83tc811_feature_array),
			       phydev->supported);

	/* Only allow advertising what this PHY supports */
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	return 0;
}

static int dp83811_probe(struct phy_device *phydev)
{
	struct dp83811_private *dp83811;

	dp83811 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83811),
			       GFP_KERNEL);
	if (!dp83811)
		return -ENOMEM;

	phydev->priv = dp83811;

	return dp8381x_chip_init(phydev);
}

#define DP83TC81X_PHY_DRIVER(_id, _name)			\
	{							\
		PHY_ID_MATCH_MODEL(_id),			\
		.name		= (_name),			\
		.probe          = dp83811_probe,		\
		.config_init = dp83811_config_init,		\
		.config_aneg = dp83811_config_aneg,		\
		.soft_reset = dp83811_phy_reset,		\
		.get_wol = dp83811_get_wol,			\
		.set_wol = dp83811_set_wol,			\
		.read_status = dp8381x_read_status,		\
		.handle_interrupt = dp83811_handle_interrupt,	\
		.config_intr = dp83811_config_intr,		\
		.suspend = dp83811_suspend,			\
		.resume = dp83811_resume,			\
		.get_features	= dp83tc811_get_features,	\
	 }

static struct phy_driver dp83tc811_driver[] = {
	DP83TC81X_PHY_DRIVER(DP83TC811_PHY_ID, "TI DP83TC811"),
	DP83TC81X_PHY_DRIVER(DP83TC812_PHY_ID, "TI DP83TC812"),
	DP83TC81X_PHY_DRIVER(DP83TC813_PHY_ID, "TI DP83TC813"),
	DP83TC81X_PHY_DRIVER(DP83TC814_PHY_ID, "TI DP83TC814"),
	DP83TC81X_PHY_DRIVER(DP83TC815_PHY_ID, "TI DP83TC815"),
};
module_phy_driver(dp83tc811_driver);

static struct mdio_device_id __maybe_unused dp83811_tbl[] = {
	{ DP83TC811_PHY_ID, 0xfffffff0 },
	{ DP83TC812_PHY_ID, 0xfffffff0 },
	{ DP83TC813_PHY_ID, 0xfffffff0 },
	{ DP83TC814_PHY_ID, 0xfffffff0 },
	{ DP83TC815_PHY_ID, 0xfffffff0 },
	{ },
};
MODULE_DEVICE_TABLE(mdio, dp83811_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TC811 PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
