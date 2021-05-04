/* 
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 * 
 * PHY management interface implementation
 * Models currently supported:
 * - Marvell Alaska 88E1111 (QEMU simulated PHY)
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

/**
 * @brief Read PHY data via the MDIO interface
 * Reads data from a PHY attached to the respective GEM's MDIO interface
 *
 * @param base_addr Base address of the GEM's register space
 * @param phy_addr  MDIO address of the PHY to be accessed
 * @param reg_addr  Index of the PHY register to be read
 * @return          16-bit data word received from the PHY
 */
static uint16_t phy_xlnx_gem_mdio_read(
	uint32_t base_addr, uint8_t phy_addr,
	uint8_t reg_addr)
{
	uint32_t reg_val  = 0;
	uint32_t poll_cnt = 0;

	/* MDIO read operation as described in Zynq-7000 TRM,
	 * chapter 16.3.4, p. 517.*/

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 before issuing the
	 * current command. */
	do {
		if (poll_cnt++ > 0) k_busy_wait(100);
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0 && poll_cnt < 10);
	if (poll_cnt == 10) {
		LOG_ERR("GEM@0x%08X read from PHY address %hhu, "
			"register address %hhu timed out",
			base_addr, phy_addr, reg_addr);
		return 0;
	}
	poll_cnt = 0;

	/* Assemble & write the read command to the gem.phy_maint register */

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

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 -> current command
	 * completed. */
	do {
		if (poll_cnt++ > 0) k_busy_wait(100);
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0 && poll_cnt < 10);
	if (poll_cnt == 10) {
		LOG_ERR("GEM@0x%08X read from PHY address %hhu, "
			"register address %hhu timed out",
			base_addr, phy_addr, reg_addr);
		return 0;
	}

	/* Read the data returned by the PHY -> lower 16 bits of the PHY main-
	 * tenance register */
	reg_val = sys_read32(base_addr + ETH_XLNX_GEM_PHY_MAINTENANCE_OFFSET);
	return (uint16_t)reg_val;
}

/**
 * @brief Writes PHY data via the MDIO interface
 * Writes data to a PHY attached to the respective GEM's MDIO interface
 *
 * @param base_addr Base address of the GEM's register space
 * @param phy_addr  MDIO address of the PHY to be accessed
 * @param reg_addr  Index of the PHY register to be written to
 * @param value     16-bit data word to be written to the target register
 */
static void phy_xlnx_gem_mdio_write(
	uint32_t base_addr, uint8_t phy_addr,
	uint8_t reg_addr, uint16_t value)
{
	uint32_t reg_val  = 0;
	uint32_t poll_cnt = 0;

	/* MDIO write operation as described in Zynq-7000 TRM,
	 * chapter 16.3.4, p. 517. */

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 before issuing the
	 * current command. */
	do {
		if (poll_cnt++ > 0) k_busy_wait(100);
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0 && poll_cnt < 10);
	if (poll_cnt == 10) {
		LOG_ERR("GEM@0x%08X write to PHY address %hhu, "
			"register address %hhu timed out",
			base_addr, phy_addr, reg_addr);
		return;
	}
	poll_cnt = 0;

	/* Assemble & write the read command to the gem.phy_maint register */

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

	/* Wait until gem.net_status[phy_mgmt_idle] == 1 -> current command
	 * completed. */
	do {
		if (poll_cnt++ > 0) k_busy_wait(100);
		reg_val = sys_read32(base_addr + ETH_XLNX_GEM_NWSR_OFFSET);
	} while ((reg_val & ETH_XLNX_GEM_MDIO_IDLE_BIT) == 0 && poll_cnt < 10);
	if (poll_cnt == 10) {
		LOG_ERR("GEM@0x%08X write to PHY address %hhu, "
			"register address %hhu timed out",
			base_addr, phy_addr, reg_addr);
	}
}

/* 
 * Vendor-specific PHY management functions for:
 * Marvell Alaska 88E1111 (QEMU simulated PHY)
 * Marvell Alaska 88E1510/88E1518/88E1512/88E1514 (Zedboard)
 * Register IDs & procedures are based on the corresponding datasheets:
 * https://www.marvell.com/content/dam/marvell/en/public-collateral/transceivers/marvell-phys-transceivers-alaska-88e1111-datasheet.pdf
 * https://www.marvell.com/content/dam/marvell/en/public-collateral/transceivers/marvell-phys-transceivers-alaska-88e151x-datasheet.pdf
 */

/**
 * @brief Marvell Alaska PHY reset function
 * Reset function for the Marvell Alaska PHY series
 *
 * @param dev Pointer to the device data struct
 */
static void phy_xlnx_gem_marvell_alaska_reset(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	uint16_t			phy_data  = 0;
	uint32_t			tries     = 0;

	/* Page 0, register address 0 = Copper control register,
	 * bit [15] = PHY reset. Register 0/0 access is R/M/W.
	 * Comp. datasheet chapter 2.6. */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	phy_data |= 0x8000;  /* Reset bit */
	phy_data &= ~0x1000; /* Auto-neg disable (for now) */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete. */
	while ((phy_data & 0x8000) != 0 && tries++ < 10) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}
	if (tries == 10) {
		LOG_ERR("GEM@0x%08X reset PHY address %hhu "
			"(Marvell Alaska) timed out",
			dev_conf->base_addr, dev_data->phy_addr);
	}
}

/**
 * @brief Marvell Alaska PHY configuration function
 * Configuration function for the Marvell Alaska PHY series
 *
 * @param dev Pointer to the device data struct
 */
static void phy_xlnx_gem_marvell_alaska_cfg(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf     = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data     = DEV_DATA(dev);
	uint16_t			phy_data      = 0;
	uint16_t			phy_data_gbit = 0;
	uint32_t			tries         = 0;

	/* Configure the system interface and media type (e.g. "RGMII
	 * to Copper").
	 * TODO: Make this value configurable via KConfig?
	 * Page 18, register address 20 = General Control Register 1,
	 * bits [2..0] = mode configuration
	 * NOTICE: a change of this value requires a subsequent software
	 * reset command via the same register's bit [15]. */
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
	 * Reset is performed immediately, bit [15] is self-clearing. */
	phy_data |= 0x8000;
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_GENERAL_CONTROL_1_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete. */
	while ((phy_data & 0x8000) != 0 && tries++ < 10) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_GENERAL_CONTROL_1_REGISTER);
	}
	if (tries == 10) {
		LOG_ERR("GEM@0x%08X configure PHY address %hhu "
			"(Marvell Alaska) timed out",
			dev_conf->base_addr, dev_data->phy_addr);
		return;
	}
	tries = 0;

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
	 * Page 0, register address 18 = Copper Specific Interrupt
	 * Enable Register,
	 * bit [14] = Speed changed interrupt enable,
	 * bit [13] = Duplex changed interrupt enable,
	 * bit [11] = Auto-negotiation completed interrupt enable,
	 * bit [10] = Link status changed interrupt enable. */
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

	/* Bit [15] reverts to 0 once the reset is complete. */
	while ((phy_data & 0x8000) != 0 && tries++ < 10)
	{
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}
	if (tries == 10) {
		LOG_ERR("GEM@0x%08X configure PHY address %hhu "
			"(Marvell Alaska) timed out",
			dev_conf->base_addr, dev_data->phy_addr);
		return;
	}
	tries = 0;

	/* Clear the interrupt status register before advertising the
	 * supported link speed(s).*/
	phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_INTERRUPT_STATUS_REGISTER);

	/* Set what link speeds shall be advertised during auto-negotiation,
	 * re-enable auto-negotiation. PHY link speed advertisement
	 * configuration as described in Zynq-7000 TRM, chapter 16.3.4,
	 * p. 517. */

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
	 *     Complete. */

	/* Advertise the speed & duplex specified in the device configuration
	 * data -> targets: registers 4 & 9. */
	phy_data       = 0x01; /* [4..0] = Selector field, 00001 = 802.3 */
	phy_data_gbit  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_1000BASET_CONTROL_REGISTER);

	if (dev_conf->enable_fdx == 1) {
		if (dev_conf->max_link_speed == LINK_1GBIT) {
			/* Advertise 1 GBit/s, full duplex */
			phy_data_gbit = (1 << 9);
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
	 * clearing. */
	phy_data  = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	phy_data |= 0x8000; /* Reset bit */
	phy_data |= 0x1000; /* Enable auto-negotiation */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_CONTROL_REGISTER, phy_data);

	/* Bit [15] reverts to 0 once the reset is complete. */
	while ((phy_data & 0x8000) != 0 && tries++ < 10) {
		phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
			dev_data->phy_addr, PHY_COPPER_CONTROL_REGISTER);
	}
	if (tries == 10) {
		LOG_ERR("GEM@0x%08X configure PHY address %hhu "
			"(Marvell Alaska) timed out",
			dev_conf->base_addr, dev_data->phy_addr);
		return;
	}

	/* Set the link speed to 'link down' for now, once auto-negotiation
	 * is complete, the result will be handled by the system work queue. */
	dev_data->eff_link_speed = LINK_DOWN;
}

/**
 * @brief Marvell Alaska PHY status change polling function
 * Status change polling function for the Marvell Alaska PHY series
 *
 * @param dev Pointer to the device data struct
 */
static uint16_t phy_xlnx_gem_marvell_alaska_poll_sc(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf  = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data  = DEV_DATA(dev);
	uint16_t			phy_data   = 0;
	uint16_t			phy_status = 0;

	/* PHY status change detection is implemented by reading the
	 * interrupt status register.
	 * Page 0, register address 19 = Copper Interrupt Status Register
	 * bit [14] = Speed changed interrupt,
	 * bit [13] = Duplex changed interrupt,
	 * bit [11] = Auto-negotiation completed interrupt,
	 * bit [10] = Link status changed interrupt. */
	phy_data = phy_xlnx_gem_mdio_read(dev_conf->base_addr,
		dev_data->phy_addr, PHY_COPPER_INTERRUPT_STATUS_REGISTER);

	if ((phy_data & PHY_COPPER_AUTONEG_COMPLETED_INTERRUPT_BIT) != 0) {
		phy_status |= PHY_XLNX_GEM_EVENT_AUTONEG_COMPLETE;
	}
	if (((phy_data & PHY_COPPER_DUPLEX_CHANGED_INTERRUPT_BIT) != 0) ||
		((phy_data & PHY_COPPER_LINK_STATUS_CHANGED_INTERRUPT_BIT) != 0)) {
		phy_status |= PHY_XLNX_GEM_EVENT_LINK_STATE_CHANGED;
	}
	if ((phy_data & PHY_COPPER_SPEED_CHANGED_INTERRUPT_BIT) != 0) {
		phy_status |= PHY_XLNX_GEM_EVENT_LINK_SPEED_CHANGED;
	}

	/* Clear the status register, preserve reserved bit [3] as indicated
	 * by the datasheet */
	phy_xlnx_gem_mdio_write(dev_conf->base_addr, dev_data->phy_addr,
		PHY_COPPER_INTERRUPT_STATUS_REGISTER, (phy_data & 0x8));

	return phy_status;
}

/**
 * @brief Marvell Alaska PHY configuration function
 * Configuration function for the Marvell Alaska PHY series
 *
 * @param dev Pointer to the device data struct
 */
static uint8_t phy_xlnx_gem_marvell_alaska_poll_lsts(struct device *dev)
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

/**
 * @brief Marvell Alaska PHY link speed polling function
 * Link speed polling function for the Marvell Alaska PHY series
 *
 * @param dev Pointer to the device data struct
 * @return    Enum containing the current link speed detected by the PHY
 */
static enum eth_xlnx_link_speed phy_xlnx_gem_marvell_alaska_poll_lspd(
	struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf  = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data  = DEV_DATA(dev);
	enum eth_xlnx_link_speed	link_speed = LINK_DOWN;
	uint16_t			phy_data   = 0;

	/* Current link speed is obtained from:
	 * Page 0, register address 17 = Copper Specifc Status Register 1
	 * bits [15 .. 14] = Speed. */
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

/**
 * @brief Marvell Alaska PHY function pointer table
 * Function pointer table for the Marvell Alaska PHY series
 * specific management functions
 */
static struct phy_xlnx_gem_api phy_xlnx_gem_marvell_alaska_api = {
	.phy_reset_func              = phy_xlnx_gem_marvell_alaska_reset,
	.phy_configure_func          = phy_xlnx_gem_marvell_alaska_cfg,
	.phy_poll_status_change_func = phy_xlnx_gem_marvell_alaska_poll_sc,
	.phy_poll_link_status_func   = phy_xlnx_gem_marvell_alaska_poll_lsts,
	.phy_poll_link_speed_func    = phy_xlnx_gem_marvell_alaska_poll_lspd
};

/* All vendor-specific API structs & code are located above
 * -> assemble the top-level list of supported devices the
 * upcoming function phy_xlnx_gem_detect will work with. */

 /**
 * @brief Top-level table of supported PHYs
 * Top-level table of PHYs supported by the GEM driver. Contains 1..n
 * supported PHY specifications, consisting of the PHY ID plus a mask
 * for masking out variable parts of the PHY ID such as hardware revisions,
 * as well as a textual description of the PHY model and a pointer to
 * the corresponding PHY management function pointer table.
 */
static struct phy_xlnx_gem_supported_dev phy_xlnx_gem_supported_devs[] = {
	{
		.phy_id      = 0x01410CC0, 
		.phy_id_mask = 0xFFFFFFF0, /* [3..0] = revision -> discard */
		.api         = &phy_xlnx_gem_marvell_alaska_api,
		.identifier  = "Marvell Alaska 88E1111"
	},
	{
		.phy_id      = 0x01410DD0, 
		.phy_id_mask = 0xFFFFFFF0, /* [3..0] = revision -> discard */
		.api         = &phy_xlnx_gem_marvell_alaska_api,
		.identifier  = "Marvell Alaska 88E151x"
	}
};

/**
 * @brief Top-level PHY detection function
 * Top-level PHY detection function called by the GEM driver if PHY management
 * is enabled for the current GEM device instance. This function is generic
 * and does not require any knowledge regarding PHY vendors, models etc.
 *
 * @param dev Pointer to the device data struct
 * @retval    -ENOTSUP if PHY management is disabled for the current GEM
 *            device instance
 * @retval    -EIO if no (supported) PHY was detected
 * @retval    0 if a supported PHY has been detected
 */
int phy_xlnx_gem_detect(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);

	uint8_t  phy_curr_addr  = 0;
	uint8_t  phy_first_addr = (dev_conf->phy_mdio_addr_fix != 0)
		? dev_conf->phy_mdio_addr_fix : 1;
	uint8_t  phy_last_addr  = (dev_conf->phy_mdio_addr_fix != 0)
		? dev_conf->phy_mdio_addr_fix : 32;
	uint32_t phy_id         = 0;
	uint16_t phy_data       = 0;
	uint32_t list_iter      = 0;

	/* Clear the PHY address & ID in the device data struct -> may be
	 * pre-initialized with a non-zero address meaning auto detection
	 * is disabled. If eventually a supported PHY is found, a non-
	 * zero address will be written back to the data struct. */
	dev_data->phy_addr       = 0;
	dev_data->phy_id         = 0;
	dev_data->phy_access_api = NULL;

	if (dev_conf->init_phy == 0) {
		return -ENOTSUP;
	}

	/* PHY detection as described in Zynq-7000 TRM, chapter 16.3.4,
	 * p. 517 */
	for (phy_curr_addr = phy_first_addr; 
		phy_curr_addr <= phy_last_addr;
		phy_curr_addr++) {
		/* Read the upper & lower PHY ID 16-bit words */
		phy_data  = phy_xlnx_gem_mdio_read(
			dev_conf->base_addr, phy_curr_addr,
			PHY_IDENTIFIER_1_REGISTER);
		phy_id    = (((uint32_t)phy_data << 16) & 0xFFFF0000);
		phy_data  = phy_xlnx_gem_mdio_read(
			dev_conf->base_addr, phy_curr_addr,
			PHY_IDENTIFIER_2_REGISTER);
		phy_id   |= ((uint32_t)phy_data & 0x0000FFFF);

		if (phy_id != 0x00000000 && phy_id != 0xFFFFFFFF) {
			LOG_DBG("GEM@0x%08X PHY detected at address %hhu: ID 0x%08X",
				dev_conf->base_addr,
				phy_curr_addr, phy_id);

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
					LOG_DBG("GEM@0x%08X Supported PHY identified: %s",
						dev_conf->base_addr,
						phy_xlnx_gem_supported_devs[list_iter].identifier);

					/* Store the numeric values of the PHY ID and address
					 * as well as the corresponding set of function pointers
					 * in the device's run-time data struct. */
					dev_data->phy_addr       = phy_curr_addr;
					dev_data->phy_id         = phy_id;
					dev_data->phy_access_api =
						phy_xlnx_gem_supported_devs[list_iter].api;

					return 0;
				}
			}
		}
	}

	LOG_ERR("GEM@0x%08X PHY detection failed", dev_conf->base_addr);
	return -EIO;
}

/* EOF */