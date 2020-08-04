/* 
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 * 
 * PHY management interface implementation
 * Models currently supported:
 * - Marvell Alaska 88E1510/88E1518/88E1512/88E1514 (Zedboard)
 * 
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>

#include "eth_xlnx_gem_priv.h"

#define LOG_MODULE_NAME phy_xlnx_gem
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* Basic MDIO read / write functions for PHY access */

static int phy_xlnx_gem_mdio_read (
	uint32_t base_addr, uint8_t phy_addr,
	uint8_t reg_addr)
{
	uint32_t reg_val = 0;

	/* MDIO read operation as described in Zynq-7000 TRM, Chapter 16.3.4, 
	 * p. 517 */

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 before issuing the 
	 * current command. TODO: This should preferrably have a time-out! */
	do {
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0);

	/* Assemble & write the current read command to the gem.phy_maint 
	 * register */

	/* Set the bits constant for any operation */
	reg_val  = ETH_XLNX_GEM_PHY_MAINT_CONST_BITS;
	/* Indicate a read operation */
	reg_val |= ETH_XLNX_GEM_PHY_MAINT_READ_OP_BIT;
	/* PHY address */
	reg_val |= (
		((uint32_t)phy_addr & ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_MASK)
		<< ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_SHIFT);
	/* Register address */
	reg_val |= (
		((uint32_t)reg_addr & ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_MASK) 
		<< ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_SHIFT);

	sys_write32(reg_val, base_addr + ETH_XLNX_GEM_PHY_MAINTENANCE_OFFSET);

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 -> current command com-
	 * pleted. TODO: This should preferrably have a time-out! */
	do {
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0);

	/* Read the data returned by the PHY -> lower 16 bits of the PHY main-
	 * tenance register */

	reg_val = sys_read32(base_addr + ETH_XLNX_GEM_PHY_MAINTENANCE_OFFSET);
	return (uint16_t)reg_val;
}

static void phy_xlnx_gem_mdio_write (
	uint32_t base_addr, uint8_t phy_addr,
	uint8_t reg_addr, uint16_t value)
{
	uint32_t reg_val = 0;

	/* MDIO write operation as described in Zynq-7000 TRM, chapter 16.3.4,
	 * p. 517 */

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 before issuing the
	 * current command. TODO: This should preferrably have a time-out! */
	do {
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0);

	/* Assemble & write the current write command to the gem.phy_maint
	 * register */

	/* Set the bits constant for any operation */
	reg_val  = ETH_XLNX_GEM_PHY_MAINT_CONST_BITS;
	/* Indicate a read operation */
	reg_val |= ETH_XLNX_GEM_PHY_MAINT_WRITE_OP_BIT;
	/* PHY address */
	reg_val |= (
		((uint32_t)phy_addr & ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_MASK)
		<< ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_SHIFT);
	/* Register address */
	reg_val |= (
		((uint32_t)reg_addr & ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_MASK)
		<< ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_SHIFT);
	/* 16 bits of data for the destination register */
	reg_val |= ((uint32_t)value & ETH_XLNX_GEM_PHY_MAINT_DATA_MASK);

	sys_write32(reg_val, base_addr + ETH_XLNX_GEM_PHY_MAINTENANCE_OFFSET);

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 -> current command com-
	 * pleted. TODO: This should preferrably have a time-out! */
	do {
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0);
}

/* Vendor-specific PHY management functions for:
 * Marvell Alaska 88E1510/88E1518/88E1512/88E1514 (Zedboard)
 * Register IDs & procedures are based on the corresponding datasheet:
 * https://www.marvell.com/documents/eoxwrbluvwybgxvagkkf/ */

static void phy_xlnx_gem_marvell_alaska_reset (struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data = 0;

	/* Page 0, register address 0 = Copper control register, 
	 * bit [15] = PHY reset. Register 0/0 access is R/M/W. 
	 * Comp. datasheet chapter 2.6 */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	phy_data |= 0x8000;  /* Reset bit */
	phy_data &= ~0x1000; /* Auto-neg disable (for now) */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete.
	 * TODO: This should preferrably have a time-out! */
	while ((phy_data & 0x8000) != 0) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}
}

static void phy_xlnx_gem_marvell_alaska_cfg (struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data      = 0;
	uint16_t phy_data_gbit = 0;

	/* Configure the system interface and media type (e.g. "RGMII 
	 * to Copper"). TODO: Make this value configurable via KConfig?
	 * Page 18, register address 20 = General Control Register 1,
	 * bits [2..0] = mode configuration
	 * NOTICE: a change of this value requires a subsequent software
	 * reset command via the same register's bit [15] */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_PAGE_SWITCH_REGISTER, PHY_GENERAL_CONTROL_1_PAGE);

	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_PAGE_SWITCH_REGISTER);
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_GENERAL_CONTROL_1_REGISTER);

	/* [2..0] 000 = RGMII (System Mode) to Copper */
	phy_data &= ~(PHY_MODE_CONFIG_MASK << PHY_MODE_CONFIG_SHIFT);
	phy_data |= 0;
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_GENERAL_CONTROL_1_REGISTER, phy_data);

	/* [15] Mode Software Reset bit, affecting pages 6 and 18
	 * Reset is performed immediately, bit [15] is self-clearing */
	phy_data |= 0x8000;
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_GENERAL_CONTROL_1_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete.
	 * TODO: This should preferrably have a time-out! */
	while ((phy_data & 0x8000) != 0) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_GENERAL_CONTROL_1_REGISTER);
	}

	/* Revert to register page 0 */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_PAGE_SWITCH_REGISTER, PHY_BASE_REGISTERS_PAGE);

	/* Configure MDIX
	 * TODO: Make this value configurable via KConfig?
	 * Page 0, register address 16 = Copper specific control register 1,
	 * bits [6..5] = MDIO crossover mode.
	 * NOTICE: a change of this value requires a subsequent software
	 * reset command via Copper Control Register's bit [15]. */

	/* [6..5] 11 = Enable auto cross over detection */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_1_REGISTER);
	phy_data &= ~(PHY_MDIX_CONFIG_MASK << PHY_MDIX_CONFIG_SHIFT);
	phy_data |= (0x03 << PHY_MDIX_CONFIG_SHIFT);
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_CONTROL_1_REGISTER, phy_data);

	/* Configure the Copper Interrupt Enable Register.
	 * The interrupt status register provides a convenient way to
	 * detect relevant state changes, also, PHY management could
	 * eventually be changed from polling to interrupt-driven.
	 * There's just one big catch: at least on the Zedboard, the
	 * PHY interrupt line isn't wired up, therefore, the GEM can
	 * never trigger a PHY interrupt. Still, the PHY interrupts
	 * are configured & enabled in order to obtain all relevant
	 * status data from a single source.
	 * 
	 * -> all bits contained herein will be retained during the 
	 * upcoming software reset operation.
	 * Page 0, register address 18 = Copper Specific Interrupt Enable
	 * Register,
	 * bit [14] = Speed changed interrupt enable,
	 * bit [13] = Duplex changed interrupt enable,
	 * bit [11] = Auto-negotiation completed interrupt enable,
	 * bit [10] = Link status changed interrupt enable */
	phy_data = PHY_COPPER_SPEED_CHANGED_INTERRUPT_BIT
		| PHY_COPPER_DUPLEX_CHANGED_INTERRUPT_BIT
		| PHY_COPPER_AUTONEG_COMPLETED_INTERRUPT_BIT
		| PHY_COPPER_LINK_STATUS_CHANGED_INTERRUPT_BIT;
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_INTERRUPT_ENABLE_REGISTER, phy_data);

	/* Page 0 / Reg 0 [15] Copper Software Reset bit, affecting pages
	 * 0, 2, 3, 5, 7. Reset is performed immediately, bit [15] is
	 * self-clearing. */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	phy_data |= 0x8000; 
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_CONTROL_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete.
	 * TODO: This should preferrably have a time-out! */
	while ((phy_data & 0x8000) != 0)
	{
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}

	/* Clear the interrupt status register before advertising the
	 * supported link speed(s) */
	phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr, 
		dev_data->phy_addr, PHY_COPPER_INTERRUPT_STATUS_REGISTER);

	/* Set what link speeds shall be advertised during auto-negotiation,
	 * re-enable auto-negotiation. PHY link speed advertisement
	 * configuration as described in Zynq-7000 TRM, chapter 16.3.4,
	 * p. 517 */

	/* Advertise the link speed from the device configuration & perform
	 * auto-negotiation. This process involves:
	 * 
	 * Page 0, register address 4 =
	 *     Copper Auto-Negotiation Advertisement Register,
	 * Page 0, register address 0 =
	 *     Copper Control Register, bit [15] = Reset -> apply all changes
	 *     made regarding advertisement,
	 * Page 0, register address 9 =
	 *     1000BASE-T Control Register (if link speed = 1GBit/s),
	 * Page 0, register address 1 =
	 *     Copper Status Register, bit [5] = Copper Auto-Negotiation
	 *     Complete */

	/* Advertise the speed & duplex specified in the device configuration
	 * data -> targets: registers 4 & 9 */
	phy_data       = 0x01; /* [4..0] = Selector field, 00001 = 802.3 */
	phy_data_gbit  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_1000BASET_CONTROL_REGISTER);

	if (dev_conf->enable_fdx == 1) {
		if (dev_conf->max_link_speed == LINK_1GBIT) {
			/* Advertise 1 GBit/s, full duplex */
			phy_data_gbit  = (1 << 9);
			if (dev_conf->phy_advertise_lower == 1) {
				/* + 100BASE-TX, full duplex */
				phy_data  |= (1 << 8);
				/* + 10BASE-TX, full duplex */
				phy_data  |= (1 << 6);
			}
		} else if (dev_conf->max_link_speed == LINK_100MBIT) {
			/* Advertise 100BASE-TX, full duplex */
			phy_data      |= (1 << 8);
			if (dev_conf->phy_advertise_lower == 1) {
				/* + 10BASE-TX, full duplex */
				phy_data  |= (1 << 6);
			}
			/* Clear 1000BASE-TX advertisement bits */
			phy_data_gbit &= ~(0x0300);
		} else if (dev_conf->max_link_speed == LINK_10MBIT) {
			/* Advertise 10BASE-TX, full duplex */
			phy_data      |= (1 << 6);
			/* Clear 1000BASE-TX advertisement bits */
			phy_data_gbit &= ~(0x0300);
		}
	} else {
		if (dev_conf->max_link_speed == LINK_1GBIT) {
			/* Advertise 1 GBit/s, half duplex */ 
			phy_data_gbit  = (1 << 8);
			if (dev_conf->phy_advertise_lower == 1) {
				/* + 100BASE-TX, half duplex */
				phy_data  |= (1 << 7);
				/* + 10BASE-TX, half duplex */
				phy_data  |= (1 << 5);
			}
		} else if (dev_conf->max_link_speed == LINK_100MBIT) {
			/* Advertise 100BASE-TX, half duplex */
			phy_data      |= (1 << 7);
			if (dev_conf->phy_advertise_lower == 1) {
				/* + 10BASE-TX, half duplex */
				phy_data  |= (1 << 5);
			}
			/* Clear 1000BASE-TX advertisement bits */
			phy_data_gbit &= ~(0x0300);
		} else if (dev_conf->max_link_speed == LINK_10MBIT) {
			/* Advertise 10BASE-TX, half duplex */
			phy_data      |= (1 << 5);
			/* Clear 1000BASE-TX advertisement bits */
			phy_data_gbit &= ~(0x0300);
		}
	}

	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_1000BASET_CONTROL_REGISTER, phy_data_gbit);
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_AUTONEG_ADV_REGISTER, phy_data);

	/* Page 0 / Reg 0 [15] Copper Software Reset bit, affecting pages
	 * 0, 2, 3, 5, 7. Reset is performed immediately, bit [15] is self-
	 * clearing */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	phy_data |= 0x8000; /* Reset bit */
	phy_data |= 0x1000; /* Enable auto-negotiation */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_CONTROL_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete.
	 * TODO: This should preferrably have a time-out! */
	while ((phy_data & 0x8000) != 0) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}

	/* Set the link speed to 'link down' for now, once auto-negotiation
	 * is complete, the result will be handled by the auxiliary thread */
	dev_data->eff_link_speed = LINK_DOWN;
}

static uint16_t phy_xlnx_gem_marvell_alaska_poll_sc (struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data   = 0;
	uint16_t phy_status = 0;

	/* PHY status change detection is implemented by reading the
	 * interrupt status register.
	 * Page 0, register address 19 = Copper Interrupt Status Register
	 * bit [14] = Speed changed interrupt,
	 * bit [13] = Duplex changed interrupt,
	 * bit [11] = Auto-negotiation completed interrupt,
	 * bit [10] = Link status changed interrupt */
	phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_INTERRUPT_STATUS_REGISTER);

	if ((phy_data & PHY_COPPER_AUTONEG_COMPLETED_INTERRUPT_BIT) != 0) {
		phy_status |= PHY_XLNX_GEM_EVENT_AUTONEG_COMPLETE;
	}
	if (((phy_data & PHY_COPPER_DUPLEX_CHANGED_INTERRUPT_BIT) != 0)
		|| ((phy_data & PHY_COPPER_LINK_STATUS_CHANGED_INTERRUPT_BIT) != 0)) {
		phy_status |= PHY_XLNX_GEM_EVENT_LINK_STATE_CHANGED;
	}
	if ((phy_data & PHY_COPPER_SPEED_CHANGED_INTERRUPT_BIT) != 0) {
		phy_status |= PHY_XLNX_GEM_EVENT_LINK_SPEED_CHANGED;
	}

	return phy_status;
}

static uint8_t phy_xlnx_gem_marvell_alaska_poll_lsts (struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);
	uint16_t phy_data = 0;

	/* Current link status is obtained from:
	 * Page 0, register address 1 = Copper Status Register
	 * bit [2] = Copper Link Status */
	phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_STATUS_REGISTER);

	return ((phy_data >> PHY_COPPER_LINK_STATUS_BIT_SHIFT) & 0x01);
}

static enum eth_xlnx_link_speed phy_xlnx_gem_marvell_alaska_poll_lspd (
	struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);
	enum eth_xlnx_link_speed link_speed = LINK_DOWN;
	uint16_t phy_data = 0;

	/* Current link speed is obtained from:
	 * Page 0, register address 17 = Copper Specifc Status Register 1
	 * bits [15 .. 14] = Speed */
	phy_data   = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_STATUS_1_REGISTER);
	phy_data >>= PHY_LINK_SPEED_SHIFT;
	phy_data  &= PHY_LINK_SPEED_MASK;

	/* Link speed bit masks: comp. datasheet, table 77 @ description
	 * of the 'Speed' bits. */
	switch (phy_data) {
		case 0:
			link_speed = LINK_10MBIT;
			break;
		case 1:
			link_speed = LINK_100MBIT;
			break;
		case 2:
			link_speed = LINK_1GBIT;
			break;
		default:
			link_speed = LINK_DOWN;
			break;
	};

	return link_speed;
}

static struct phy_xlnx_gem_api phy_xlnx_gem_marvell_alaska_api = {
	.phy_reset_func = phy_xlnx_gem_marvell_alaska_reset,
	.phy_configure_func = phy_xlnx_gem_marvell_alaska_cfg,
	.phy_poll_status_change_func = phy_xlnx_gem_marvell_alaska_poll_sc,
	.phy_poll_link_status_func = phy_xlnx_gem_marvell_alaska_poll_lsts,
	.phy_poll_link_speed_func = phy_xlnx_gem_marvell_alaska_poll_lspd
};

/* All vendor-specific API structs & code are located above
 * -> assemble the top-level list of supported devices the
 * upcoming function phy_xlnx_gem_detect will work with. */

static struct phy_xlnx_gem_supported_dev phy_xlnx_gem_supported_devs[] = {
	{
		.phy_id      = 0x01410DD0, 
		.phy_id_mask = 0xFFFFFFF0, /* [3..0] = revision -> discard */
		.api         = &phy_xlnx_gem_marvell_alaska_api,
		.identifier  = "Marvell Alaska 88E151x"
	}
};

/* PHY detection function. This generic function exposed to the GEM driver
 * writes the pointer to the applicable function table into the driver's
 * run-time data struct if a compatible PHY is found. */

int phy_xlnx_gem_detect (struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);

	uint8_t  phy_addr  = 0;
	uint32_t phy_id    = 0;
	uint16_t phy_data  = 0;
	uint32_t list_iter = 0;

	if (dev == NULL) {
		return -EINVAL;
	}

	if (dev_conf->init_phy == 0) {
		dev_data->phy_id = 0;
		dev_data->phy_addr = 0;
		dev_data->phy_access_api = NULL;
		return -ENOTSUP;
	}

	/* PHY detection as described in Zynq-7000 TRM, chapter 16.3.4,
	 * p. 517 */
	for (phy_addr = 1; phy_addr <= 32; phy_addr++) {
		phy_data  = phy_xlnx_gem_mdio_read(
			dev_conf->base_addr, phy_addr,
			PHY_IDENTIFIER_1_REGISTER);
		phy_id    = (((uint32_t)phy_data << 16) & 0xFFFF0000);
		phy_data  = phy_xlnx_gem_mdio_read(
			dev_conf->base_addr, phy_addr,
			PHY_IDENTIFIER_2_REGISTER);
		phy_id   |= ((uint32_t)phy_data & 0x0000FFFF);

		if (phy_id != 0x00000000 && phy_id != 0xFFFFFFFF) {
			LOG_DBG("PHY detected at address %hhu: ID 0x%08X",
				phy_addr, phy_id);

			/* Store the numeric values of the PHY ID and address
			 * in the device's run-time data struct. */
			dev_data->phy_addr = phy_addr;
			dev_data->phy_id   = phy_id;

			/* Iterate the list of all supported PHYs -> if the
			 * current PHY is supported, store all related data
			 * in the device's run-time data struct. */
			for (list_iter = 0; list_iter <
				(sizeof(phy_xlnx_gem_supported_devs) /
				sizeof(struct phy_xlnx_gem_supported_dev));
				list_iter++)
			{
				if (phy_xlnx_gem_supported_devs[list_iter].phy_id ==
					(phy_xlnx_gem_supported_devs[list_iter].phy_id_mask
					& phy_id)) {
					LOG_DBG("Supported PHY identified: %s",
						phy_xlnx_gem_supported_devs[list_iter].identifier);
					dev_data->phy_access_api =
						phy_xlnx_gem_supported_devs[list_iter].api;
					return 0;
				}
			}
		}
	}

	LOG_DBG("PHY auto-detection failed - no reply in MDIO address "
		"range 1..32");

	dev_data->phy_id = 0;
	dev_data->phy_addr = 0;
	dev_data->phy_access_api = NULL;
	return -EIO;
}

/* EOF */