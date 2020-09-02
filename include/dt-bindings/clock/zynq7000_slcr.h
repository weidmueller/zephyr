/*
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ZYNQ7000_SLCR_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ZYNQ7000_SLCR_H_

/* Clock domain mode setting (6:3:2:1 or 4:2:2:1) aliases */
#define ZYNQ7000_CLK_DOMAIN_MODE_6321    0U
#define ZYNQ7000_CLK_DOMAIN_MODE_4221    1U

/* Reference clock source identifiers for peripheral components */
#define ZYNQ7000_SLCR_REFCLK_SRC_IOPLL   0U
#define ZYNQ7000_SLCR_REFCLK_SRC_ARMPLL  2U
#define ZYNQ7000_SLCR_REFCLK_SRC_DDRPLL  3U
#define ZYNQ7000_SLCR_REFCLK_SRC_EMIOCLK 4U

/* RX clock source identifiers for GEM Ethernet controller devices */
#define ZYNQ7000_SLCR_GEM_RXCLK_SRC_MIO  0U
#define ZYNQ7000_SLCR_GEM_RXCLK_SRC_EMIO 1U

/* Reference clock divisor0/1 'auto detect' alias */
#define ZYNQ7000_REFCLK_DIVISOR_AUTO     0U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ZYNQ7000_SLCR_H_ */
