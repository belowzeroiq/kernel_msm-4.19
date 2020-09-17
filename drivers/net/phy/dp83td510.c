// SPDX-License-Identifier: GPL-2.0
/* Driver for the Texas Instruments DP83TD510 PHY
 * Copyright (C) 2020 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define DP83TD510E_PHY_ID	0x20000180
#define DP83TD510E_V2_PHY_ID	0x20000181
#define DP83TD510_DEVADDR_AN	0x7
#define DP83TD510_DEVADDR	0x1f
#define DP83TD510_PMD_DEVADDR	0x1

#define DP83TD510_PHY_STAT	0x10
#define DP83TD510_GEN_CFG	0x11
#define DP83TD510_INT_REG1	0x12
#define DP83TD510_INT_REG2	0x13
#define DP83TD510_MAC_CFG_1	0x17
#define DP83TD510_CTRL_REG	0x1f

#define DP83TD510_ANEG_CTRL	0x200
#define DP83TD510_PMD_CTRL	0x834
#define DP83TD510_M_S_CTRL	0x8f6

#define DP83TD510_SOR_1		0x467

#define DP83TD510_HW_RESET	BIT(15)
#define DP83TD510_SW_RESET	BIT(14)

#define DP83TD510_LINK_STS	BIT(0)

/* GEN CFG bits */
#define DP83TD510_INT_OE	BIT(0)
#define DP83TD510_INT_EN	BIT(1)

/* INT REG 1 bits */
#define DP83TD510_INT1_ESD_EN	BIT(3)
#define DP83TD510_INT1_LINK_EN	BIT(5)
#define DP83TD510_INT1_RHF_EN	BIT(7)
#define DP83TD510_INT1_ESD	BIT(11)
#define DP83TD510_INT1_LINK	BIT(13)
#define DP83TD510_INT1_RHF	BIT(15)

/* INT REG 2 bits */
#define DP83TD510_INT2_POR_EN	BIT(0)
#define DP83TD510_INT2_POL_EN	BIT(1)
#define DP83TD510_INT2_PAGE_EN	BIT(5)
#define DP83TD510_INT2_POR	BIT(8)
#define DP83TD510_INT2_POL	BIT(9)
#define DP83TD510_INT2_PAGE	BIT(13)

/* MAC CFG bits */
#define DP83TD510_RX_TX_DELAY_MASK GENMASK(12, 11)
#define DP83TD510_RX_CLK_SHIFT	BIT(12)
#define DP83TD510_TX_CLK_SHIFT	BIT(11)

#define DP83TD510_MASTER_MODE	BIT(14)
#define DP83TD510_AUTO_NEG_EN	BIT(12)
#define DP83TD510_RGMII		BIT(8)

#define DP83TD510_FIFO_DEPTH_MASK	GENMASK(6, 5)
#define DP83TD510_FIFO_DEPTH_4_B_NIB	0
#define DP83TD510_FIFO_DEPTH_5_B_NIB	BIT(5)
#define DP83TD510_FIFO_DEPTH_6_B_NIB	BIT(6)
#define DP83TD510_FIFO_DEPTH_8_B_NIB	(BIT(5) | BIT(6))

#define DP83TD510_2_4V		BIT(12)
#define DP83TD510_2_4V_P2P	2400
#define DP83TD510_1_1V_P2P	1100
#define DP83TD510_AUTO_NEG_P2P	0

const int dp83td510_feature_array[4] = {
	ETHTOOL_LINK_MODE_Autoneg_BIT,
	ETHTOOL_LINK_MODE_10baseT1L_Half_BIT,
	ETHTOOL_LINK_MODE_10baseT1L_Full_BIT,
	ETHTOOL_LINK_MODE_TP_BIT,
};

struct dp83td510_private {
	u32 hi_diff_output;
	u32 tx_fifo_depth;
	u32 rgmii_delay;
	bool is_rgmii;
};

struct dp83td510_init_reg {
	int reg;
	int val;
};

static struct dp83td510_init_reg dp83td510_errata[] = {
	{ 0x608, 0x003b }, /* disable_0_transition */
	{ 0x862, 0x39f8 }, /* AGC Gain during Autoneg */
	{ 0x81a, 0x67c0 }, /* deq offset for 1V swing */
	{ 0x81c, 0xfb62 }, /* deq offset for 2.4V swing */
	{ 0x830, 0x05a3 }, /* Enable energy lost fallback */
	{ 0x855, 0x1b55 }, /* MSE Threshold change */
	{ 0x831, 0x0403 }, /* energy detect threshold */
	{ 0x856, 0x1800 }, /* good1 MSE threshold change */
	{ 0x857, 0x8fa0 }, /* Enable fallback to phase 1 on watchdog trigger */
	{ 0x871, 0x000c }, /* TED input changed to slicer_in without FFE */
	{ 0x883, 0x022e }, /* Enable Rx Filter for Short Cable detection */
	{ 0x402, 0x1800 }, /* Adjust LD swing */
	{ 0x878, 0x2248 }, /* Change PI up/down polarity */
	{ 0x10c, 0x0008 }, /* tx filter coefficient */
	{ 0x112, 0x1212 }, /* tx filter scaling factor */
	{ 0x809, 0x5c80 }, /* AGC retrain */
	{ 0x803, 0x1529 }, /* Master Ph1 Back-off */
	{ 0x804, 0x1a33 }, /* Master Ph1 Back-off */
	{ 0x805, 0x1f3d }, /* Master Ph1 Back-off */
	{ 0x850, 0x045b }, /* hybrid gain & delay */
	{ 0x874, 0x6967 }, /* kp step 0 for master */
	{ 0x852, 0x7800 }, /* FAGC init gain */
	{ 0x806, 0x1e1e }, /* Master/Slave Ph2 Back-off */
	{ 0x807, 0x2525 }, /* Master/Slave Ph2 Back-off */
	{ 0x808, 0x2c2c }, /* Master/Slave Ph2 Back-off */
	{ 0x850, 0x0590 }, /* Hybrid Gain/Delay Code */
	{ 0x827, 0x4000 }, /* Echo Fixed Delay */
	{ 0x849, 0x0fe4 }, /* Hybrid Cal enable */
	{ 0x84b, 0x04b5 }, /* Echo Score Sel */
	{ 0x018, 0x0043 }, /* CRS/RX_DV pin as RX_DV for RMII repeater mode */
};

static int dp83td510_ack_interrupt(struct phy_device *phydev)
{
	int ret;

	ret = phy_read(phydev, DP83TD510_INT_REG1);
	if (ret < 0)
		return ret;

	if (!(ret & (DP83TD510_INT1_ESD | DP83TD510_INT1_LINK |
	    DP83TD510_INT1_RHF)))
		return IRQ_NONE;

	ret = phy_read(phydev, DP83TD510_INT_REG2);
	if (ret < 0)
		return ret;

	if (!(ret & (DP83TD510_INT2_POR | DP83TD510_INT2_POL |
	    DP83TD510_INT2_PAGE)))
		return IRQ_NONE;

	phy_trigger_machine(phydev);

	return 0;
}

static irqreturn_t dp83td510_handle_interrupt(struct phy_device *phydev)
{
	int ret;

	ret = dp83td510_ack_interrupt(phydev);
	if (ret)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int dp83td510_config_intr(struct phy_device *phydev)
{
	int gen_cfg_val;
	int int_status;
	int ret;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		int_status = phy_read(phydev, DP83TD510_INT_REG1);
		if (int_status < 0)
			return int_status;

		int_status = (DP83TD510_INT1_ESD_EN | DP83TD510_INT1_LINK_EN |
			      DP83TD510_INT1_RHF_EN);

		ret = phy_write(phydev, DP83TD510_INT_REG1, int_status);
		if (ret)
			return ret;

		int_status = phy_read(phydev, DP83TD510_INT_REG2);
		if (int_status < 0)
			return int_status;

		int_status = (DP83TD510_INT2_POR_EN | DP83TD510_INT2_POL_EN |
				DP83TD510_INT2_PAGE_EN);

		ret = phy_write(phydev, DP83TD510_INT_REG2, int_status);
		if (ret)
			return ret;

		gen_cfg_val = phy_read(phydev, DP83TD510_GEN_CFG);
		if (gen_cfg_val < 0)
			return gen_cfg_val;

		gen_cfg_val |= DP83TD510_INT_OE | DP83TD510_INT_EN;

	} else {
		ret = phy_write(phydev, DP83TD510_INT_REG1, 0);
		if (ret)
			return ret;

		ret = phy_write(phydev, DP83TD510_INT_REG2, 0);
		if (ret)
			return ret;

		gen_cfg_val = phy_read(phydev, DP83TD510_GEN_CFG);
		if (gen_cfg_val < 0)
			return gen_cfg_val;

		gen_cfg_val &= ~DP83TD510_INT_EN;
	}

	ret = phy_write(phydev, DP83TD510_GEN_CFG, gen_cfg_val);
	if (ret)
		return ret;

	return dp83td510_ack_interrupt(phydev);
}

static int dp83td510_configure_mode(struct phy_device *phydev, int pmd_ctrl)
{
	struct dp83td510_private *dp83td510 = phydev->priv;
	int ret;
	int i;

	ret = phy_set_bits(phydev, DP83TD510_CTRL_REG, DP83TD510_HW_RESET);
	if (ret < 0)
		return ret;

	if (dp83td510->hi_diff_output == DP83TD510_2_4V_P2P)
		ret = phy_write_mmd(phydev, DP83TD510_DEVADDR,
				    DP83TD510_M_S_CTRL, DP83TD510_2_4V);
	else
		ret = phy_write_mmd(phydev, DP83TD510_DEVADDR,
				    DP83TD510_M_S_CTRL, 0);

	if (phydev->autoneg) {
		ret = phy_modify_mmd(phydev, DP83TD510_DEVADDR_AN,
				     DP83TD510_ANEG_CTRL,
				     DP83TD510_AUTO_NEG_EN,
				     DP83TD510_AUTO_NEG_EN);
		if (ret)
			return ret;
	} else {
		ret = phy_modify_mmd(phydev, DP83TD510_DEVADDR_AN,
				     DP83TD510_ANEG_CTRL,
				     DP83TD510_AUTO_NEG_EN, 0);
		if (ret)
			return ret;
	}

	ret = phy_modify_mmd(phydev, DP83TD510_PMD_DEVADDR,
			     DP83TD510_PMD_CTRL,
			     DP83TD510_MASTER_MODE, pmd_ctrl);
	if (ret)
		return ret;

	/* Only write the Errata for version 1.0 of the PHY */
	if (phydev->phy_id == DP83TD510E_PHY_ID) {
		for (i = 0; i < ARRAY_SIZE(dp83td510_errata); i++) {
			ret = phy_write_mmd(phydev, DP83TD510_DEVADDR,
					    dp83td510_errata[i].reg,
					    dp83td510_errata[i].val);

			if (ret)
				return ret;
		}
	}

	return phy_set_bits(phydev, DP83TD510_CTRL_REG, DP83TD510_SW_RESET);
}

static int dp83td510_set_master_slave(struct phy_device *phydev)
{
	int mst_slave_cfg;

	mst_slave_cfg = phy_read_mmd(phydev, DP83TD510_PMD_DEVADDR,
				     DP83TD510_PMD_CTRL);
	if (mst_slave_cfg < 0)
		return mst_slave_cfg;

	if (mst_slave_cfg & DP83TD510_MASTER_MODE) {
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

static int dp83td510_read_status(struct phy_device *phydev)
{
	int phy_status;
	int phy_bmcr;

	phy_status = phy_read(phydev, DP83TD510_PHY_STAT);
	if (phy_status < 0)
		return phy_status;

	phy_bmcr = phy_read(phydev, MII_BMCR);
	if (phy_bmcr < 0)
		return phy_bmcr;

	phydev->link = phy_status & DP83TD510_LINK_STS;
	if (phydev->link) {
		phydev->duplex = phy_bmcr & BMCR_FULLDPLX ? DUPLEX_FULL : DUPLEX_HALF;
		phydev->speed = SPEED_10;
	} else {
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_UNKNOWN;
	}

	return dp83td510_set_master_slave(phydev);
}

static int dp83td510_get_features(struct phy_device *phydev)
{
	linkmode_set_bit_array(dp83td510_feature_array,
			       ARRAY_SIZE(dp83td510_feature_array),
			       phydev->supported);

	return 0;
}

static int dp83td510_config_aneg(struct phy_device *phydev)
{
	int pmd_ctrl;

	switch (phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
	case MASTER_SLAVE_CFG_MASTER_FORCE:
		pmd_ctrl = DP83TD510_MASTER_MODE;
		break;
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
		pmd_ctrl = 0;
		break;
	case MASTER_SLAVE_CFG_UNKNOWN:
	case MASTER_SLAVE_CFG_UNSUPPORTED:
	default:
		phydev_warn(phydev, "Unsupported Master/Slave mode\n");
		return -ENOTSUPP;
	}

	return dp83td510_configure_mode(phydev, pmd_ctrl);
}

static int dp83td510_config_init(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510 = phydev->priv;
	int ret = 0;

	if (phy_interface_is_rgmii(phydev)) {
		ret = phy_modify_mmd(phydev, DP83TD510_DEVADDR,
				     DP83TD510_MAC_CFG_1,
				     DP83TD510_RX_TX_DELAY_MASK,
				     dp83td510->rgmii_delay);
		if (ret)
			return ret;
	}

	if (phydev->interface == PHY_INTERFACE_MODE_RMII) {
		ret = phy_modify(phydev, DP83TD510_GEN_CFG,
				 DP83TD510_FIFO_DEPTH_MASK,
				 dp83td510->tx_fifo_depth);
		if (ret)
			return ret;
	}

	return dp83td510_set_master_slave(phydev);
}

static int dp83td510_phy_reset(struct phy_device *phydev)
{
	int ret;

	ret = phy_set_bits(phydev, DP83TD510_CTRL_REG, DP83TD510_SW_RESET);
	if (ret < 0)
		return ret;

	usleep_range(10, 20);

	return dp83td510_config_init(phydev);
}

#if IS_ENABLED(CONFIG_OF_MDIO)
static int dp83td510_of_init(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510 = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	s32 rx_int_delay;
	s32 tx_int_delay;
	int ret;

	if (!phydev->mdio.dev.of_node)
		return -ENODEV;

	ret = of_property_read_u32(phydev->mdio.dev.of_node,
				   "max-tx-rx-p2p-microvolt",
				   &dp83td510->hi_diff_output);
	if (ret)
		dp83td510->hi_diff_output = DP83TD510_2_4V_P2P;

	if (dp83td510->hi_diff_output != DP83TD510_2_4V_P2P &&
	    dp83td510->hi_diff_output != DP83TD510_1_1V_P2P)
		return -EINVAL;

	if (of_property_read_u32(phydev->mdio.dev.of_node, "tx-fifo-depth",
				 &dp83td510->tx_fifo_depth))
		dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_5_B_NIB;

	switch (dp83td510->tx_fifo_depth) {
	case 4:
		dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_4_B_NIB;
		break;
	case 6:
		dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_6_B_NIB;
		break;
	case 8:
		dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_8_B_NIB;
		break;
	case 5:
	default:
		dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_5_B_NIB;
	}

	rx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0, true);
	if (rx_int_delay <= 0)
		dp83td510->rgmii_delay = DP83TD510_RX_CLK_SHIFT;
	else
		dp83td510->rgmii_delay = 0;

	tx_int_delay = phy_get_internal_delay(phydev, dev, NULL, 0, false);
	if (tx_int_delay <= 0)
		dp83td510->rgmii_delay |= DP83TD510_TX_CLK_SHIFT;
	else
		dp83td510->rgmii_delay &= ~DP83TD510_TX_CLK_SHIFT;

	return 0;
}
#else
static int dp83869_of_init(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510 = phydev->priv;

	dp83td510->hi_diff_output = DP83TD510_2_4V_P2P
	dp83td510->tx_fifo_depth = DP83TD510_FIFO_DEPTH_5_B_NIB
	dp83td510->rgmii_delay = 0;

	return 0;
}
#endif /* CONFIG_OF_MDIO */

static int dp83td510_probe(struct phy_device *phydev)
{
	struct dp83td510_private *dp83td510;
	int ret;

	dp83td510 = devm_kzalloc(&phydev->mdio.dev, sizeof(*dp83td510),
				 GFP_KERNEL);
	if (!dp83td510)
		return -ENOMEM;

	phydev->priv = dp83td510;

	ret = dp83td510_of_init(phydev);
	if (ret)
		return ret;

	return dp83td510_config_init(phydev);
}

static struct phy_driver dp83td510_driver[] = {
	{
		PHY_ID_MATCH_MODEL(DP83TD510E_PHY_ID),
		.name		= "TI DP83TD510E",
		.probe          = dp83td510_probe,
		.config_init	= dp83td510_config_init,
		.soft_reset	= dp83td510_phy_reset,

		/* IRQ related */
		.handle_interrupt = dp83td510_handle_interrupt,
		.config_intr	= dp83td510_config_intr,
		.config_aneg	= dp83td510_config_aneg,
		.read_status	= dp83td510_read_status,

		.get_features	= dp83td510_get_features,

		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	},
};
module_phy_driver(dp83td510_driver);

static struct mdio_device_id __maybe_unused dp83td510_tbl[] = {
	{ PHY_ID_MATCH_MODEL(DP83TD510E_PHY_ID) },
	{ PHY_ID_MATCH_MODEL(DP83TD510E_V2_PHY_ID) },
	{ }
};
MODULE_DEVICE_TABLE(mdio, dp83td510_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83TD510E PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL v2");
