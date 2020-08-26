/*
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 * 
 * PHY management interface and related data
 * 
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_
#define _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_

#include <kernel.h>
#include <zephyr/types.h>

/* Event codes used to indicate a particular state change to the driver */
#define PHY_XLNX_GEM_EVENT_LINK_SPEED_CHANGED		(1 << 0)
#define PHY_XLNX_GEM_EVENT_LINK_STATE_CHANGED		(1 << 1)
#define PHY_XLNX_GEM_EVENT_AUTONEG_COMPLETE		(1 << 2)

/* PHY register addresses & constants that are not vendor-specific */
#define PHY_IDENTIFIER_1_REGISTER			2
#define PHY_IDENTIFIER_2_REGISTER			3

/* PHY registers & constants -> Marvell Alaska specific */
#define PHY_BASE_REGISTERS_PAGE				0
#define PHY_COPPER_CONTROL_REGISTER			0
#define PHY_COPPER_STATUS_REGISTER			1
#define PHY_COPPER_AUTONEG_ADV_REGISTER			4
#define PHY_COPPER_LINK_PARTNER_ABILITY_REGISTER	5
#define PHY_1000BASET_CONTROL_REGISTER			9
#define PHY_COPPER_CONTROL_1_REGISTER			16
#define PHY_COPPER_STATUS_1_REGISTER			17
#define PHY_COPPER_INTERRUPT_ENABLE_REGISTER		18
#define PHY_COPPER_INTERRUPT_STATUS_REGISTER		19
#define PHY_COPPER_PAGE_SWITCH_REGISTER			22
#define PHY_GENERAL_CONTROL_1_REGISTER			20
#define PHY_GENERAL_CONTROL_1_PAGE			18

#define PHY_ADV_BIT_100BASET_FDX			(1 << 8)
#define PHY_ADV_BIT_100BASET_HDX			(1 << 7)
#define PHY_ADV_BIT_10BASET_FDX				(1 << 6)
#define PHY_ADV_BIT_10BASET_HDX				(1 << 5)

#define PHY_MDIX_CONFIG_MASK				0x0003
#define PHY_MDIX_CONFIG_SHIFT				5
#define PHY_MODE_CONFIG_MASK				0x0003
#define PHY_MODE_CONFIG_SHIFT				0

#define PHY_COPPER_SPEED_CHANGED_INTERRUPT_BIT		(1 << 14)
#define PHY_COPPER_DUPLEX_CHANGED_INTERRUPT_BIT		(1 << 13)
#define PHY_COPPER_AUTONEG_COMPLETED_INTERRUPT_BIT	(1 << 11)
#define PHY_COPPER_LINK_STATUS_CHANGED_INTERRUPT_BIT	(1 << 10)
#define PHY_COPPER_LINK_STATUS_BIT_SHIFT		5

#define PHY_LINK_SPEED_SHIFT				14
#define PHY_LINK_SPEED_MASK				0x3

/* Type definitions for PHY management functions */
typedef void (*phy_xlnx_gem_reset_t)(struct device *dev);
typedef void (*phy_xlnx_gem_configure_t)(struct device *dev);
typedef uint16_t (*phy_xlnx_gem_poll_status_change_t)(struct device *dev);
typedef uint8_t (*phy_xlnx_gem_poll_link_status_t)(struct device *dev);
typedef enum eth_xlnx_link_speed (*phy_xlnx_gem_poll_link_speed_t)
	(struct device *dev);

/**
 * @brief Vendor-specific PHY management function pointer table struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_gem_api {
	phy_xlnx_gem_reset_t phy_reset_func;
	phy_xlnx_gem_configure_t phy_configure_func;
	phy_xlnx_gem_poll_status_change_t phy_poll_status_change_func;
	phy_xlnx_gem_poll_link_status_t phy_poll_link_status_func;
	phy_xlnx_gem_poll_link_speed_t phy_poll_link_speed_func;
};

/**
 * @brief Supported PHY list entry struct
 *
 * Contains the PHY management function pointers for a specific PHY
 * make or model.
 */
struct phy_xlnx_gem_supported_dev {
	uint32_t phy_id;
	uint32_t phy_id_mask;
	struct phy_xlnx_gem_api *api;
	const char *identifier;
};

/* PHY identification function -> generic, not vendor-specific */
int phy_xlnx_gem_detect (struct device *dev);

#endif /* _ZEPHYR_DRIVERS_ETHERNET_PHY_XLNX_GEM_H_ */

/* EOF */