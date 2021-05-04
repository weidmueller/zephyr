/* 
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 * 
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Current limitations / TODOs:
 * - Only supports 32-bit addresses in buffer descriptors, therefore
 *   currently only supports Zynq-7000 and ZynqMP RPU, not ZynqMP APU.
 * - Hardware timestamps not considered.
 * - Wake-on-LAN interrupt not supported.
 * - Send function is not SMP-capable (due to single TX done semaphore).
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sys/__assert.h>

#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>

#include "eth_xlnx_gem_priv.h"

#define DT_DRV_COMPAT xlnx_gem

#define LOG_MODULE_NAME eth_xlnx_gem
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int  eth_xlnx_gem_dev_init(struct device *dev);
static void eth_xlnx_gem_iface_init(struct net_if *iface);
static void eth_xlnx_gem_irq_config(struct device *dev);
static void eth_xlnx_gem_isr(void *arg);
static int  eth_xlnx_gem_send(struct device *dev, struct net_pkt *pkt);
static int  eth_xlnx_gem_start_device(struct device *dev);
static int  eth_xlnx_gem_stop_device(struct device *dev);
static enum ethernet_hw_caps eth_xlnx_gem_get_capabilities(struct device *dev);
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_xlnx_gem_stats(struct device *dev);
#endif

static void eth_xlnx_gem_reset_hw(struct device *dev);
static void eth_xlnx_gem_configure_clocks(struct device *dev);
static void eth_xlnx_gem_set_initial_nwcfg(struct device *dev);
static void eth_xlnx_gem_set_nwcfg_link_speed(struct device *dev);
static void eth_xlnx_gem_set_mac_address(struct device *dev);
static void eth_xlnx_gem_set_initial_dmacr(struct device *dev);
static void eth_xlnx_gem_init_phy(struct device *dev);
static void eth_xlnx_gem_poll_phy(struct k_work *item);
static void eth_xlnx_gem_configure_buffers(struct device *dev);
static void eth_xlnx_gem_rx_pending_work(struct k_work *item);
static void eth_xlnx_gem_handle_rx_pending(struct device *dev);
static void eth_xlnx_gem_tx_done_work(struct k_work *item);
static void eth_xlnx_gem_handle_tx_done(struct device *dev);

/* GEM Driver API declaration, required by the upcoming instances of
 * the ETH_NET_DEVICE_INIT macro for each activated device instance */
static const struct ethernet_api eth_xlnx_gem_apis = {
	.iface_api.init		= eth_xlnx_gem_iface_init,
	.get_capabilities	= eth_xlnx_gem_get_capabilities,
	.send			= eth_xlnx_gem_send,
	.start			= eth_xlnx_gem_start_device,
	.stop			= eth_xlnx_gem_stop_device,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats		= eth_xlnx_gem_stats,
#endif
};

#ifdef CONFIG_ETH_XLNX_GEM_PORT_0

/* GEM0 DMA area structure declaration */
struct eth_xlnx_dma_area_gem0 {
	struct eth_xlnx_gem_bd rx_bd[CONFIG_ETH_XLNX_GEM_PORT_0_RXBD_COUNT];
	struct eth_xlnx_gem_bd tx_bd[CONFIG_ETH_XLNX_GEM_PORT_0_TXBD_COUNT];
	uint8_t rx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_0_RXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_0_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
	uint8_t tx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_0_TXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_0_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
};

/* GEM0 device configuration data */
static struct eth_xlnx_gem_dev_cfg eth_xlnx_gem0_dev_cfg = {
	/* Controller base address -> from device tree data */
	.base_addr   = DT_REG_ADDR(DT_NODELABEL(gem0)),
	/* IRQ configuration function pointer */
	.config_func = eth_xlnx_gem_irq_config,
	/* Reference PLL clock frequency for TX clock generation */
	.pll_clock_frequency = DT_PROP(DT_NODELABEL(gem0), clock_frequency),
	/* TX clock control register address within the SLCR */
	.clk_ctrl_reg_address = DT_PROP(DT_NODELABEL(gem0), clk_ctrl_reg),
	/* MDC clock divider applied to cpu_1x clock */
	.mdc_divider = (enum eth_xlnx_mdc_clock_divider)(DT_PROP(DT_NODELABEL(gem0), mdc_divider)),
	/* Maximum supported link speed  */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_0_LINK_10MBIT)
	.max_link_speed = LINK_10MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_LINK_100MBIT)
	.max_link_speed = LINK_100MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_LINK_1GBIT)
	.max_link_speed = LINK_1GBIT,
#else
#error No valid maximum link speed setting found in GEM0 configuration data
#endif
	/* PHY initialization & management flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_INIT_PHY
	.init_phy = 1,
#else
	.init_phy = 0,
#endif
	/* PHY MDIO const address / auto detection swith */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_PHY_MDIO_ADDRESS
	.phy_mdio_addr_fix = CONFIG_ETH_XLNX_GEM_PORT_0_PHY_MDIO_ADDRESS,
#else
	.phy_mdio_addr_fix = 0,
#endif
	/* Advertise link speeds lower than nominal flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_PHY_ADVERTISE_LOWER
	.phy_advertise_lower = 1,
#else
	.phy_advertise_lower = 0,
#endif
	/* PHY status polling interval */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_PHY_POLL_INTERVAL
	.phy_poll_interval = CONFIG_ETH_XLNX_GEM_PORT_0_PHY_POLL_INTERVAL,
#else
	.phy_poll_interval = 0,
#endif
	/* Deferred processing settings */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DEFER_RX_PENDING
	.defer_rxp_to_queue = 1,
#else
	.defer_rxp_to_queue = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DEFER_TX_DONE
	.defer_txd_to_queue = 1,
#else
	.defer_txd_to_queue = 0,
#endif
	/* AMBA AHB data bus width setting */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_0_AMBAAHB_32BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_32BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_AMBAAHB_64BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_64BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_AMBAAHB_128BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_128BIT,
#else
#error No valid AMBA AHB data bus width setting found in \
	GEM0 configuration data
#endif
	/* AMBA AHB burst length */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_0_AHBBURST_SINGLE)
	.ahb_burst_length = AHB_BURST_SINGLE,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_AHBBURST_INCR4)
	.ahb_burst_length = AHB_BURST_INCR4,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_AHBBURST_INCR8)
	.ahb_burst_length = AHB_BURST_INCR8,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_AHBBURST_INCR16)
	.ahb_burst_length = AHB_BURST_INCR16,
#else
#error No valid AMBA AHB burst length setting found in \
	GEM0 configuration data
#endif
	/* Hardware RX buffer size */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_0_HWRX_BUFFER_SIZE_FULL)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_8KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_HWRX_BUFFER_SIZE_4KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_4KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_HWRX_BUFFER_SIZE_2KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_2KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_0_HWRX_BUFFER_SIZE_1KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_1KB,
#else
#error No valid Hardware RX buffer size setting found in \
	GEM0 configuration data
#endif
	/* RX buffer offset */
	.hw_rx_buffer_offset = CONFIG_ETH_XLNX_GEM_PORT_0_HWRX_BUFFER_OFFSET,
	/* DMA area receive / transmit buffer (descriptor) related data */
	.rxbd_count     = CONFIG_ETH_XLNX_GEM_PORT_0_RXBD_COUNT,
	.txbd_count     = CONFIG_ETH_XLNX_GEM_PORT_0_TXBD_COUNT,
	.rx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_0_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	.tx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_0_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	/* Feature flags, mostly targeting the gem.net_cfg register */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_IGNORE_IGP_RXER
	.ignore_igp_rxer = 1,
#else
	.ignore_igp_rxer = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISABLE_REJECT_NSP
	.disable_reject_nsp = 1,
#else
	.disable_reject_nsp = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_IGP_STRETCH
	.enable_igp_stretch = 1,
#else
	.enable_igp_stretch = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_SGMII_MODE
	.enable_sgmii_mode = 1,
#else
	.enable_sgmii_mode = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISABLE_REJECT_FCS_CRC_ERRORS
	.disable_reject_fcs_crc_errors = 1,
#else
	.disable_reject_fcs_crc_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_RX_HALFDUP_WHILE_TX
	.enable_rx_halfdup_while_tx = 1,
#else
	.enable_rx_halfdup_while_tx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_RX_CHKSUM_OFFLOAD
	.enable_rx_chksum_offload = 1,
#else
	.enable_rx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISABLE_PAUSE_COPY
	.disable_pause_copy = 1,
#else
	.disable_pause_copy = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISCARD_RX_FCS
	.discard_rx_fcs = 1,
#else
	.discard_rx_fcs = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISCARD_RX_LENGTH_ERRORS
	.discard_rx_length_errors = 1,
#else
	.discard_rx_length_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_PAUSE
	.enable_pause = 1,
#else
	.enable_pause = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_TBI
	.enable_tbi = 1,
#else
	.enable_tbi = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_EXT_ADDR_MATCH
	.ext_addr_match = 1,
#else
	.ext_addr_match = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_1536_FRAMES
	.enable_1536_frames = 1,
#else
	.enable_1536_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_UCAST_HASH
	.enable_ucast_hash = 1,
#else
	.enable_ucast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_MCAST_HASH
	.enable_mcast_hash = 1,
#else
	.enable_mcast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISABLE_BCAST
	.disable_bcast = 1,
#else
	.disable_bcast = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_COPY_ALL_FRAMES
	.copy_all_frames = 1,
#else
	.copy_all_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISCARD_NON_VLAN
	.discard_non_vlan = 1,
#else
	.discard_non_vlan = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_FDX
	.enable_fdx = 1,
#else
	.enable_fdx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DISC_RX_AHB_UNAVAIL
	.disc_rx_ahb_unavail = 1,
#else
	.disc_rx_ahb_unavail = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_TX_CHKSUM_OFFLOAD
	.enable_tx_chksum_offload = 1,
#else
	.enable_tx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_HWTX_BUFFER_SIZE_FULL
	.tx_buffer_size_full = 1,
#else
	.tx_buffer_size_full = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_AHB_PACKET_ENDIAN_SWAP
	.enable_ahb_packet_endian_swap = 1,
#else
	.enable_ahb_packet_endian_swap = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_ENABLE_AHB_MD_ENDIAN_SWAP
	.enable_ahb_md_endian_swap = 1
#else
	.enable_ahb_md_endian_swap = 0
#endif
};

/* GEM0 run-time device data */
static struct eth_xlnx_gem_dev_data eth_xlnx_gem0_dev_data = {
	.mac_addr        = DT_PROP(DT_NODELABEL(gem0), local_mac_address),
	.started         = 0,
	.eff_link_speed  = LINK_DOWN,
	.phy_addr        = 0,
	.phy_id          = 0,
	.phy_access_api  = NULL,
	.first_rx_buffer = NULL,
	.first_tx_buffer = NULL
};

/*
 * Declare the DMA area for this device locally if no fixed address
 * (e.g. OCM) was provided for the respective GEM controller.
 * WATCH OUT: No measures of any kind are taken in order to ensure that the
 * data structures declared below are located in non-cached, non-buffered
 * (strongly ordered) memory at this time!
 */
#ifndef CONFIG_ETH_XLNX_GEM_PORT_0_DMA_FIXED
static struct eth_xlnx_dma_area_gem0 dma_area_gem0;
#endif

/* GEM0 driver instance declaration */
ETH_NET_DEVICE_INIT(eth_xlnx_gem0, DT_LABEL(DT_NODELABEL(gem0)),
	eth_xlnx_gem_dev_init, device_pm_control_nop,
	&eth_xlnx_gem0_dev_data, &eth_xlnx_gem0_dev_cfg,
	CONFIG_ETH_INIT_PRIORITY,
	&eth_xlnx_gem_apis, NET_ETH_MTU);

#endif /* GEM0 active */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_1

/* GEM1 DMA area structure declaration */
struct eth_xlnx_dma_area_gem1 {
	struct eth_xlnx_gem_bd rx_bd[CONFIG_ETH_XLNX_GEM_PORT_1_RXBD_COUNT];
	struct eth_xlnx_gem_bd tx_bd[CONFIG_ETH_XLNX_GEM_PORT_1_TXBD_COUNT];
	uint8_t rx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_1_RXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_1_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
	uint8_t tx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_1_TXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_1_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
};

/* GEM1 device configuration data */
static struct eth_xlnx_gem_dev_cfg eth_xlnx_gem1_dev_cfg = {
	/* Controller base address -> from device tree data */
	.base_addr   = DT_REG_ADDR(DT_NODELABEL(gem1)),
	/* IRQ configuration function pointer */
	.config_func = eth_xlnx_gem_irq_config,
	/* Reference PLL clock frequency for TX clock generation */
	.pll_clock_frequency = DT_PROP(DT_NODELABEL(gem1), clock_frequency),
	/* TX clock control register address within the SLCR */
	.clk_ctrl_reg_address = DT_PROP(DT_NODELABEL(gem1), clk_ctrl_reg),
	/* MDC clock divider applied to cpu_1x clock */
	.mdc_divider = (enum eth_xlnx_mdc_clock_divider)(DT_PROP(DT_NODELABEL(gem1), mdc_divider)),
	/* Maximum supported link speed  */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_1_LINK_10MBIT)
	.max_link_speed = LINK_10MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_LINK_100MBIT)
	.max_link_speed = LINK_100MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_LINK_1GBIT)
	.max_link_speed = LINK_1GBIT,
#else
#error No valid maximum link speed setting found in GEM1 configuration data
#endif
	/* PHY initialization & management flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_INIT_PHY
	.init_phy = 1,
#else
	.init_phy = 0,
#endif
	/* PHY MDIO const address / auto detection swith */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_PHY_MDIO_ADDRESS
	.phy_mdio_addr_fix = CONFIG_ETH_XLNX_GEM_PORT_1_PHY_MDIO_ADDRESS,
#else
	.phy_mdio_addr_fix = 0,
#endif
	/* Advertise link speeds lower than nominal flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_PHY_ADVERTISE_LOWER
	.phy_advertise_lower = 1,
#else
	.phy_advertise_lower = 0,
#endif
	/* PHY status polling interval */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_PHY_POLL_INTERVAL
	.phy_poll_interval = CONFIG_ETH_XLNX_GEM_PORT_1_PHY_POLL_INTERVAL,
#else
	.phy_poll_interval = 0,
#endif
	/* Deferred processing settings */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DEFER_RX_PENDING
	.defer_rxp_to_queue = 1,
#else
	.defer_rxp_to_queue = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DEFER_TX_DONE
	.defer_txd_to_queue = 1,
#else
	.defer_txd_to_queue = 0,
#endif
	/* AMBA AHB data bus width setting */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_1_AMBAAHB_32BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_32BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_AMBAAHB_64BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_64BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_AMBAAHB_128BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_128BIT,
#else
#error No valid AMBA AHB data bus width setting found in \
	GEM1 configuration data
#endif
	/* AMBA AHB burst length */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_1_AHBBURST_SINGLE)
	.ahb_burst_length = AHB_BURST_SINGLE,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_AHBBURST_INCR4)
	.ahb_burst_length = AHB_BURST_INCR4,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_AHBBURST_INCR8)
	.ahb_burst_length = AHB_BURST_INCR8,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_AHBBURST_INCR16)
	.ahb_burst_length = AHB_BURST_INCR16,
#else
#error No valid AMBA AHB burst length setting found in \
	GEM1 configuration data
#endif
	/* Hardware RX buffer size */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_1_HWRX_BUFFER_SIZE_FULL)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_8KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_HWRX_BUFFER_SIZE_4KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_4KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_HWRX_BUFFER_SIZE_2KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_2KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_1_HWRX_BUFFER_SIZE_1KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_1KB,
#else
#error No valid Hardware RX buffer size setting found in \
	GEM1 configuration data
#endif
	/* RX buffer offset */
	.hw_rx_buffer_offset = CONFIG_ETH_XLNX_GEM_PORT_1_HWRX_BUFFER_OFFSET,
	/* DMA area receive / transmit buffer (descriptor) related data */
	.rxbd_count     = CONFIG_ETH_XLNX_GEM_PORT_1_RXBD_COUNT,
	.txbd_count     = CONFIG_ETH_XLNX_GEM_PORT_1_TXBD_COUNT,
	.rx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_1_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	.tx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_1_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	/* Feature flags, mostly targeting the gem.net_cfg register */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_IGNORE_IGP_RXER
	.ignore_igp_rxer = 1,
#else
	.ignore_igp_rxer = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISABLE_REJECT_NSP
	.disable_reject_nsp = 1,
#else
	.disable_reject_nsp = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_IGP_STRETCH
	.enable_igp_stretch = 1,
#else
	.enable_igp_stretch = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_SGMII_MODE
	.enable_sgmii_mode = 1,
#else
	.enable_sgmii_mode = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISABLE_REJECT_FCS_CRC_ERRORS
	.disable_reject_fcs_crc_errors = 1,
#else
	.disable_reject_fcs_crc_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_RX_HALFDUP_WHILE_TX
	.enable_rx_halfdup_while_tx = 1,
#else
	.enable_rx_halfdup_while_tx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_RX_CHKSUM_OFFLOAD
	.enable_rx_chksum_offload = 1,
#else
	.enable_rx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISABLE_PAUSE_COPY
	.disable_pause_copy = 1,
#else
	.disable_pause_copy = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISCARD_RX_FCS
	.discard_rx_fcs = 1,
#else
	.discard_rx_fcs = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISCARD_RX_LENGTH_ERRORS
	.discard_rx_length_errors = 1,
#else
	.discard_rx_length_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_PAUSE
	.enable_pause = 1,
#else
	.enable_pause = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_TBI
	.enable_tbi = 1,
#else
	.enable_tbi = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_EXT_ADDR_MATCH
	.ext_addr_match = 1,
#else
	.ext_addr_match = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_1536_FRAMES
	.enable_1536_frames = 1,
#else
	.enable_1536_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_UCAST_HASH
	.enable_ucast_hash = 1,
#else
	.enable_ucast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_MCAST_HASH
	.enable_mcast_hash = 1,
#else
	.enable_mcast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISABLE_BCAST
	.disable_bcast = 1,
#else
	.disable_bcast = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_COPY_ALL_FRAMES
	.copy_all_frames = 1,
#else
	.copy_all_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISCARD_NON_VLAN
	.discard_non_vlan = 1,
#else
	.discard_non_vlan = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_FDX
	.enable_fdx = 1,
#else
	.enable_fdx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DISC_RX_AHB_UNAVAIL
	.disc_rx_ahb_unavail = 1,
#else
	.disc_rx_ahb_unavail = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_TX_CHKSUM_OFFLOAD
	.enable_tx_chksum_offload = 1,
#else
	.enable_tx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_HWTX_BUFFER_SIZE_FULL
	.tx_buffer_size_full = 1,
#else
	.tx_buffer_size_full = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_AHB_PACKET_ENDIAN_SWAP
	.enable_ahb_packet_endian_swap = 1,
#else
	.enable_ahb_packet_endian_swap = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_ENABLE_AHB_MD_ENDIAN_SWAP
	.enable_ahb_md_endian_swap = 1
#else
	.enable_ahb_md_endian_swap = 0
#endif
};

/* GEM1 run-time device data */
static struct eth_xlnx_gem_dev_data eth_xlnx_gem1_dev_data = {
	.mac_addr        = DT_PROP(DT_NODELABEL(gem1), local_mac_address),
	.started         = 0,
	.eff_link_speed  = LINK_DOWN,
	.phy_addr        = 0,
	.phy_id          = 0,
	.phy_access_api  = NULL,
	.first_rx_buffer = NULL,
	.first_tx_buffer = NULL
};

/*
 * Declare the DMA area for this device locally if no fixed address
 * (e.g. OCM) was provided for the respective GEM controller.
 */
#ifndef CONFIG_ETH_XLNX_GEM_PORT_1_DMA_FIXED
static struct eth_xlnx_dma_area_gem1 dma_area_gem1;
#endif

/* GEM1 driver instance declaration */
ETH_NET_DEVICE_INIT(eth_xlnx_gem1, DT_LABEL(DT_NODELABEL(gem1)),
	eth_xlnx_gem_dev_init, device_pm_control_nop,
	&eth_xlnx_gem1_dev_data, &eth_xlnx_gem1_dev_cfg,
	CONFIG_ETH_INIT_PRIORITY,
	&eth_xlnx_gem_apis, NET_ETH_MTU);

#endif /* GEM1 active */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_2

/* GEM2 exists on UltraScale targets only! */
#ifdef CONFIG_SOC_XILINX_ZYNQ7000
#error GEM2 is not supported by the Zynq-7000
#endif

/* GEM2 DMA area structure declaration */
struct eth_xlnx_dma_area_gem2 {
	struct eth_xlnx_gem_bd rx_bd[CONFIG_ETH_XLNX_GEM_PORT_2_RXBD_COUNT];
	struct eth_xlnx_gem_bd tx_bd[CONFIG_ETH_XLNX_GEM_PORT_2_TXBD_COUNT];
	uint8_t rx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_2_RXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_2_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
	uint8_t tx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_2_TXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_2_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
};

/* GEM2 device configuration data */
static struct eth_xlnx_gem_dev_cfg eth_xlnx_gem2_dev_cfg = {
	/* Controller base address -> from device tree data */
	.base_addr   = DT_REG_ADDR(DT_NODELABEL(gem2)),
	/* IRQ configuration function pointer */
	.config_func = eth_xlnx_gem_irq_config,
	/* Reference PLL clock frequency for TX clock generation */
	.pll_clock_frequency = DT_PROP(DT_NODELABEL(gem2), clock_frequency),
	/* TX clock control register address within the SLCR */
	.clk_ctrl_reg_address = DT_PROP(DT_NODELABEL(gem2), clk_ctrl_reg),
	/* MDC clock divider applied to cpu_1x clock */
	.mdc_divider = (enum eth_xlnx_mdc_clock_divider)(DT_PROP(DT_NODELABEL(gem2), mdc_divider)),
	/* Maximum supported link speed */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_2_LINK_10MBIT)
	.max_link_speed = LINK_10MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_LINK_100MBIT)
	.max_link_speed = LINK_100MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_LINK_1GBIT)
	.max_link_speed = LINK_1GBIT,
#else
#error No valid maximum link speed setting found in GEM2 configuration data
#endif
	/* PHY initialization & management flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_INIT_PHY
	.init_phy = 1,
#else
	.init_phy = 0,
#endif
	/* PHY MDIO const address / auto detection swith */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_PHY_MDIO_ADDRESS
	.phy_mdio_addr_fix = CONFIG_ETH_XLNX_GEM_PORT_2_PHY_MDIO_ADDRESS,
#else
	.phy_mdio_addr_fix = 0,
#endif
	/* Advertise link speeds lower than nominal flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_PHY_ADVERTISE_LOWER
	.phy_advertise_lower = 1,
#else
	.phy_advertise_lower = 0,
#endif
	/* PHY status polling interval */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_PHY_POLL_INTERVAL
	.phy_poll_interval = CONFIG_ETH_XLNX_GEM_PORT_2_PHY_POLL_INTERVAL,
#else
	.phy_poll_interval = 0,
#endif
	/* Deferred processing settings */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DEFER_RX_PENDING
	.defer_rxp_to_queue = 1,
#else
	.defer_rxp_to_queue = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DEFER_TX_DONE
	.defer_txd_to_queue = 1,
#else
	.defer_txd_to_queue = 0,
#endif
	/* AMBA AHB data bus width setting */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_2_AMBAAHB_32BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_32BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_AMBAAHB_64BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_64BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_AMBAAHB_128BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_128BIT,
#else
#error No valid AMBA AHB data bus width setting found in \
	GEM2 configuration data
#endif
	/* AMBA AHB burst length */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_2_AHBBURST_SINGLE)
	.ahb_burst_length = AHB_BURST_SINGLE,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_AHBBURST_INCR4)
	.ahb_burst_length = AHB_BURST_INCR4,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_AHBBURST_INCR8)
	.ahb_burst_length = AHB_BURST_INCR8,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_AHBBURST_INCR16)
	.ahb_burst_length = AHB_BURST_INCR16,
#else
#error No valid AMBA AHB burst length setting found in \
	GEM2 configuration data
#endif
	/* Hardware RX buffer size */ \
#if defined(CONFIG_ETH_XLNX_GEM_PORT_2_HWRX_BUFFER_SIZE_FULL)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_8KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_HWRX_BUFFER_SIZE_4KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_4KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_HWRX_BUFFER_SIZE_2KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_2KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_2_HWRX_BUFFER_SIZE_1KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_1KB,
#else
#error No valid Hardware RX buffer size setting found in \
	GEM2 configuration data
#endif
	/* RX buffer offset */
	.hw_rx_buffer_offset = CONFIG_ETH_XLNX_GEM_PORT_2_HWRX_BUFFER_OFFSET,
	/* DMA area receive / transmit buffer (descriptor) related data */
	.rxbd_count     = CONFIG_ETH_XLNX_GEM_PORT_2_RXBD_COUNT,
	.txbd_count     = CONFIG_ETH_XLNX_GEM_PORT_2_TXBD_COUNT,
	.rx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_2_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	.tx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_2_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	/* Feature flags, mostly targeting the gem.net_cfg register */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_IGNORE_IGP_RXER
	.ignore_igp_rxer = 1,
#else
	.ignore_igp_rxer = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISABLE_REJECT_NSP
	.disable_reject_nsp = 1,
#else
	.disable_reject_nsp = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_IGP_STRETCH
	.enable_igp_stretch = 1,
#else
	.enable_igp_stretch = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_SGMII_MODE
	.enable_sgmii_mode = 1,
#else
	.enable_sgmii_mode = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISABLE_REJECT_FCS_CRC_ERRORS
	.disable_reject_fcs_crc_errors = 1,
#else
	.disable_reject_fcs_crc_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_RX_HALFDUP_WHILE_TX
	.enable_rx_halfdup_while_tx = 1,
#else
	.enable_rx_halfdup_while_tx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_RX_CHKSUM_OFFLOAD
	.enable_rx_chksum_offload = 1,
#else
	.enable_rx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISABLE_PAUSE_COPY
	.disable_pause_copy = 1,
#else
	.disable_pause_copy = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISCARD_RX_FCS
	.discard_rx_fcs = 1,
#else
	.discard_rx_fcs = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISCARD_RX_LENGTH_ERRORS
	.discard_rx_length_errors = 1,
#else
	.discard_rx_length_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_PAUSE
	.enable_pause = 1,
#else
	.enable_pause = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_TBI
	.enable_tbi = 1,
#else
	.enable_tbi = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_EXT_ADDR_MATCH
	.ext_addr_match = 1,
#else
	.ext_addr_match = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_1536_FRAMES
	.enable_1536_frames = 1,
#else
	.enable_1536_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_UCAST_HASH
	.enable_ucast_hash = 1,
#else
	.enable_ucast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_MCAST_HASH
	.enable_mcast_hash = 1,
#else
	.enable_mcast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISABLE_BCAST
	.disable_bcast = 1,
#else
	.disable_bcast = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_COPY_ALL_FRAMES
	.copy_all_frames = 1,
#else
	.copy_all_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISCARD_NON_VLAN
	.discard_non_vlan = 1,
#else
	.discard_non_vlan = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_FDX
	.enable_fdx = 1,
#else
	.enable_fdx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DISC_RX_AHB_UNAVAIL
	.disc_rx_ahb_unavail = 1,
#else
	.disc_rx_ahb_unavail = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_TX_CHKSUM_OFFLOAD
	.enable_tx_chksum_offload = 1,
#else
	.enable_tx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_HWTX_BUFFER_SIZE_FULL
	.tx_buffer_size_full = 1,
#else
	.tx_buffer_size_full = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_AHB_PACKET_ENDIAN_SWAP
	.enable_ahb_packet_endian_swap = 1,
#else
	.enable_ahb_packet_endian_swap = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_ENABLE_AHB_MD_ENDIAN_SWAP
	.enable_ahb_md_endian_swap = 1
#else
	.enable_ahb_md_endian_swap = 0
#endif
};

/* GEM2 run-time device data */
static struct eth_xlnx_gem_dev_data eth_xlnx_gem2_dev_data = {
	.mac_addr        = DT_PROP(DT_NODELABEL(gem2), local_mac_address),
	.started         = 0,
	.eff_link_speed  = LINK_DOWN,
	.phy_addr        = 0,
	.phy_id          = 0,
	.phy_access_api  = NULL,
	.first_rx_buffer = NULL,
	.first_tx_buffer = NULL
};

/*
 * Declare the DMA area for this device locally if no fixed address
 * (e.g. OCM) was provided for the respective GEM controller.
 */
#ifndef CONFIG_ETH_XLNX_GEM_PORT_2_DMA_FIXED
static struct eth_xlnx_dma_area_gem2 dma_area_gem2;
#endif

/* GEM2 driver instance declaration */
ETH_NET_DEVICE_INIT(eth_xlnx_gem2, DT_LABEL(DT_NODELABEL(gem2)),
	eth_xlnx_gem_dev_init, device_pm_control_nop,
	&eth_xlnx_gem2_dev_data, &eth_xlnx_gem2_dev_cfg,
	CONFIG_ETH_INIT_PRIORITY,
	&eth_xlnx_gem_apis, NET_ETH_MTU);

#endif /* GEM2 active */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_3

/* GEM3 exists on UltraScale targets only! */
#ifdef CONFIG_SOC_XILINX_ZYNQ7000
#error GEM3 is not supported by the Zynq-7000
#endif

/* GEM3 DMA area structure declaration */
struct eth_xlnx_dma_area_gem3 {
	struct eth_xlnx_gem_bd rx_bd[CONFIG_ETH_XLNX_GEM_PORT_3_RXBD_COUNT];
	struct eth_xlnx_gem_bd tx_bd[CONFIG_ETH_XLNX_GEM_PORT_3_TXBD_COUNT];
	uint8_t rx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_3_RXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_3_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
	uint8_t tx_buffer
		[CONFIG_ETH_XLNX_GEM_PORT_3_TXBD_COUNT]
		[((CONFIG_ETH_XLNX_GEM_PORT_3_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT - 1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT -1))];
};

/* GEM3 device configuration data */
static struct eth_xlnx_gem_dev_cfg eth_xlnx_gem3_dev_cfg = {
	/* Controller base address -> from device tree data */
	.base_addr   = DT_REG_ADDR(DT_NODELABEL(gem3)),
	/* IRQ configuration function pointer */
	.config_func = eth_xlnx_gem_irq_config,
	/* Reference PLL clock frequency for TX clock generation */
	.pll_clock_frequency = DT_PROP(DT_NODELABEL(gem3), clock_frequency),
	/* TX clock control register address within the SLCR */
	.clk_ctrl_reg_address = DT_PROP(DT_NODELABEL(gem3), clk_ctrl_reg),
	/* MDC clock divider applied to cpu_1x clock */
	.mdc_divider = (enum eth_xlnx_mdc_clock_divider)(DT_PROP(DT_NODELABEL(gem3), mdc_divider)),
	/* Maximum supported link speed */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_3_LINK_10MBIT)
	.max_link_speed = LINK_10MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_LINK_100MBIT)
	.max_link_speed = LINK_100MBIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_LINK_1GBIT)
	.max_link_speed = LINK_1GBIT,
#else
#error No valid maximum link speed setting found in GEM3 configuration data
#endif
	/* PHY initialization & management flag  */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_INIT_PHY
	.init_phy = 1,
#else
	.init_phy = 0,
#endif
	/* PHY MDIO const address / auto detection swith */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_PHY_MDIO_ADDRESS
	.phy_mdio_addr_fix = CONFIG_ETH_XLNX_GEM_PORT_3_PHY_MDIO_ADDRESS,
#else
	.phy_mdio_addr_fix = 0,
#endif
	/* Advertise link speeds lower than nominal flag */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_PHY_ADVERTISE_LOWER
	.phy_advertise_lower = 1,
#else
	.phy_advertise_lower = 0,
#endif
	/* PHY status polling interval */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_PHY_POLL_INTERVAL
	.phy_poll_interval   = CONFIG_ETH_XLNX_GEM_PORT_3_PHY_POLL_INTERVAL,
#else
	.phy_poll_interval   = 0,
#endif
	/* Deferred processing settings */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DEFER_RX_PENDING
	.defer_rxp_to_queue = 1,
#else
	.defer_rxp_to_queue = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DEFER_TX_DONE
	.defer_txd_to_queue = 1,
#else
	.defer_txd_to_queue = 0,
#endif
	/* AMBA AHB data bus width setting */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_3_AMBAAHB_32BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_32BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_AMBAAHB_64BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_64BIT,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_AMBAAHB_128BIT)
	.amba_dbus_width = AMBA_AHB_DBUS_WIDTH_128BIT,
#else
#error No valid AMBA AHB data bus width setting found in \
	GEM3 configuration data
#endif
	/* AMBA AHB burst length */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_3_AHBBURST_SINGLE)
	.ahb_burst_length = AHB_BURST_SINGLE,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_AHBBURST_INCR4)
	.ahb_burst_length = AHB_BURST_INCR4,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_AHBBURST_INCR8)
	.ahb_burst_length = AHB_BURST_INCR8,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_AHBBURST_INCR16)
	.ahb_burst_length = AHB_BURST_INCR16,
#else
#error No valid AMBA AHB burst length setting found in \
	GEM3 configuration data
#endif
	/* Hardware RX buffer size */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_3_HWRX_BUFFER_SIZE_FULL)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_8KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_HWRX_BUFFER_SIZE_4KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_4KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_HWRX_BUFFER_SIZE_2KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_2KB,
#elif defined(CONFIG_ETH_XLNX_GEM_PORT_3_HWRX_BUFFER_SIZE_1KB)
	.hw_rx_buffer_size = HWRX_BUFFER_SIZE_1KB,
#else
#error No valid Hardware RX buffer size setting found in \
	GEM3 configuration data
#endif
	/* RX buffer offset */
	.hw_rx_buffer_offset = CONFIG_ETH_XLNX_GEM_PORT_3_HWRX_BUFFER_OFFSET,
	/* DMA area receive / transmit buffer (descriptor) related data */
	.rxbd_count     = CONFIG_ETH_XLNX_GEM_PORT_3_RXBD_COUNT,
	.txbd_count     = CONFIG_ETH_XLNX_GEM_PORT_3_TXBD_COUNT,
	.rx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_3_RX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	.tx_buffer_size = ((CONFIG_ETH_XLNX_GEM_PORT_3_TX_BUFFER_SIZE
		+ (ETH_XLNX_BUFFER_ALIGNMENT-1))
		& ~(ETH_XLNX_BUFFER_ALIGNMENT-1)),
	/* Feature flags, mostly targeting the gem.net_cfg register */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_IGNORE_IGP_RXER
	.ignore_igp_rxer = 1,
#else
	.ignore_igp_rxer = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISABLE_REJECT_NSP
	.disable_reject_nsp = 1,
#else
	.disable_reject_nsp = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_IGP_STRETCH
	.enable_igp_stretch = 1,
#else
	.enable_igp_stretch = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_SGMII_MODE
	.enable_sgmii_mode = 1,
#else
	.enable_sgmii_mode = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISABLE_REJECT_FCS_CRC_ERRORS
	.disable_reject_fcs_crc_errors = 1,
#else
	.disable_reject_fcs_crc_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_RX_HALFDUP_WHILE_TX
	.enable_rx_halfdup_while_tx = 1,
#else
	.enable_rx_halfdup_while_tx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_RX_CHKSUM_OFFLOAD
	.enable_rx_chksum_offload = 1,
#else
	.enable_rx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISABLE_PAUSE_COPY
	.disable_pause_copy = 1,
#else
	.disable_pause_copy = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISCARD_RX_FCS
	.discard_rx_fcs = 1,
#else
	.discard_rx_fcs = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISCARD_RX_LENGTH_ERRORS
	.discard_rx_length_errors = 1,
#else
	.discard_rx_length_errors = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_PAUSE
	.enable_pause = 1,
#else
	.enable_pause = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_TBI
	.enable_tbi = 1,
#else
	.enable_tbi = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_EXT_ADDR_MATCH
	.ext_addr_match = 1,
#else
	.ext_addr_match = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_1536_FRAMES
	.enable_1536_frames = 1,
#else
	.enable_1536_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_UCAST_HASH
	.enable_ucast_hash = 1,
#else
	.enable_ucast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_MCAST_HASH
	.enable_mcast_hash = 1,
#else
	.enable_mcast_hash = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISABLE_BCAST
	.disable_bcast = 1,
#else
	.disable_bcast = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_COPY_ALL_FRAMES
	.copy_all_frames = 1,
#else
	.copy_all_frames = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISCARD_NON_VLAN
	.discard_non_vlan = 1,
#else
	.discard_non_vlan = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_FDX
	.enable_fdx = 1,
#else
	.enable_fdx = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DISC_RX_AHB_UNAVAIL
	.disc_rx_ahb_unavail = 1,
#else
	.disc_rx_ahb_unavail = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_TX_CHKSUM_OFFLOAD
	.enable_tx_chksum_offload = 1,
#else
	.enable_tx_chksum_offload = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_HWTX_BUFFER_SIZE_FULL
	.tx_buffer_size_full = 1,
#else
	.tx_buffer_size_full = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_AHB_PACKET_ENDIAN_SWAP
	.enable_ahb_packet_endian_swap = 1,
#else
	.enable_ahb_packet_endian_swap = 0,
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_ENABLE_AHB_MD_ENDIAN_SWAP
	.enable_ahb_md_endian_swap = 1
#else
	.enable_ahb_md_endian_swap = 0
#endif
};

/* GEM3 run-time device data */
static struct eth_xlnx_gem_dev_data eth_xlnx_gem3_dev_data = {
	.mac_addr        = DT_PROP(DT_NODELABEL(gem3), local_mac_address),
	.started         = 0,
	.eff_link_speed  = LINK_DOWN,
	.phy_addr        = 0,
	.phy_id          = 0,
	.phy_access_api  = NULL,
	.first_rx_buffer = NULL,
	.first_tx_buffer = NULL
};

/*
 * Declare the DMA area for this device locally if no fixed address
 * (e.g. OCM) was provided for the respective GEM controller.
 */
#ifndef CONFIG_ETH_XLNX_GEM_PORT_3_DMA_FIXED
static struct eth_xlnx_dma_area_gem3 dma_area_gem3;
#endif

/* GEM3 driver instance declaration */
ETH_NET_DEVICE_INIT(eth_xlnx_gem3, DT_LABEL(DT_NODELABEL(gem3)),
	eth_xlnx_gem_dev_init, device_pm_control_nop,
	&eth_xlnx_gem3_dev_data, &eth_xlnx_gem3_dev_cfg,
	CONFIG_ETH_INIT_PRIORITY,
	&eth_xlnx_gem_apis, NET_ETH_MTU);

#endif /* GEM3 active */

/**
 * @brief GEM device initialization function
 * Initializes the GEM itself, the DMA memory area used by the GEM and,
 * if enabled, an associated PHY attached to the GEM's MDIO interface.
 *
 * @param dev Pointer to the device data struct
 * @retval 0 if the device initialization completed successfully
 */
static int eth_xlnx_gem_dev_init(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	uint32_t 			reg_val   = 0;

	/* RX Buffer size must be a multiple of 64, as the size of the
	 * corresponding DMA receive buffer in AHB system memory is
	 * expressed as n * 64 bytes in the DMA configuration register. */
	__ASSERT(dev_conf->rx_buffer_size % 64 == 0,
		"GEM@0x%08X RX buffer size %u is not a multiple of 64 bytes",
		dev_conf->base_addr);

	/* Initialization procedure as described in the Zynq-7000 TRM,
	 * chapter 16.3.x. */
	eth_xlnx_gem_reset_hw(dev);		/* Chapter 16.3.1 */
	eth_xlnx_gem_set_initial_nwcfg(dev);	/* Chapter 16.3.2 */
	eth_xlnx_gem_set_mac_address(dev);	/* Chapter 16.3.2 */
	eth_xlnx_gem_set_initial_dmacr(dev);	/* Chapter 16.3.2 */

	/* Enable MDIO -> set gem.net_ctrl[mgmt_port_en] */
	if (dev_conf->init_phy == 1) {
		reg_val = sys_read32(
			dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);
		reg_val |= ETH_XLNX_GEM_NWCTRL_MDEN_BIT;
		sys_write32(reg_val,
			dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);
	}

	eth_xlnx_gem_configure_clocks(dev);	/* Chapter 16.3.3 */
	if (dev_conf->init_phy == 1) {
		eth_xlnx_gem_init_phy(dev);	/* Chapter 16.3.4 */
	}
	eth_xlnx_gem_configure_buffers(dev);	/* Chapter 16.3.5 */

	return 0;
}

/**
 * @brief GEM associated interface initialization function
 * Initializes the interface associated with a GEM device.
 *
 * @param iface Pointer to the associated interface data struct
 */
static void eth_xlnx_gem_iface_init(struct net_if *iface)
{
	struct device			*dev      = net_if_get_device(iface);
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);

	/* Set the initial contents of the current instance's run-time data */
	dev_data->iface = iface;
	net_if_set_link_addr(iface, dev_data->mac_addr, 6, NET_LINK_ETHERNET);
	ethernet_init(iface);

	/* Initialize the (delayed) work structures for RX pending, TX done
	 * and PHY status polling handlers */
	k_work_init(&dev_data->tx_done_work, eth_xlnx_gem_tx_done_work);
	k_work_init(&dev_data->rx_pend_work, eth_xlnx_gem_rx_pending_work);
	k_delayed_work_init(&dev_data->phy_poll_delayed_work,
		eth_xlnx_gem_poll_phy);

	/* Initialize TX completion semaphore */
	k_sem_init(&dev_data->tx_done_sem, 0, 1);

	/* Initialize semaphores in the RX/TX BD ring values which have not
	 * yet been initialized */
	k_sem_init(&dev_data->rxbd_ring.ring_sem, 1, 1);
	k_sem_init(&dev_data->txbd_ring.ring_sem, 1, 1);

	/* Initialize interrupts */
	dev_conf->config_func(dev);
}

/**
 * @brief GEM IRQ initialization function
 * Initializes the IRQ associated with a GEM device.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_irq_config(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(dev);

	/* Attach to the respective GEM's primary IRQ.
	 * The GEMs' Wake-on-LAN IRQs are not yet supported. */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0
	ETH_XLNX_GEM_CONFIG_IRQ(0);
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1
	ETH_XLNX_GEM_CONFIG_IRQ(1);
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2
	ETH_XLNX_GEM_CONFIG_IRQ(2);
#endif
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3
	ETH_XLNX_GEM_CONFIG_IRQ(3);
#endif
}

/**
 * @brief GEM interrupt service routine
 * GEM interrupt service routine. Checks for indications of errors
 * and either immediately handles RX pending / TX complete notifications
 * or defers them to the system work queue.
 *
 * @param arg Argument pointer, effectively pointer to the device data struct
 */
static void eth_xlnx_gem_isr(void *arg)
{
	struct device *const 		dev       = (struct device *const)arg;
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t 			reg_val   = 0;

	/* Read the interrupt status register */
	reg_val = sys_read32(dev_conf->base_addr + ETH_XLNX_GEM_ISR_OFFSET);

	/* TODO: handling if one or more error flag(s) are set in the
	 * interrupt status register. -> For now, just log them */
	if (reg_val & 0x00000C60) {
		LOG_ERR("GEM@0x%08X error(s) set in ISR reg: 0x%08X",
			dev_conf->base_addr,
			reg_val);
	}

	/* Check for the following indications by the controller:
	 * reg_val & 0x00000080 -> gem.intr_status bit [7] = Frame TX complete
	 * reg_val & 0x00000002 -> gem.intr_status bit [1] = Frame received
	 * comp. Zynq-7000 TRM, Chapter B.18, p. 1289/1290.
	 * If the respective condition's handling is configured to be deferred
	 * to the work queue thread, submit the corresponding job to the work
	 * queue, otherwise, handle the condition immediately. */
	if ((reg_val & ETH_XLNX_GEM_IXR_TXCOMPL_BIT) != 0) {
		if (dev_conf->defer_txd_to_queue == 1) {
			k_work_submit(&dev_data->tx_done_work);
		} else {
			printk("TX: handle in ISR\n");
			eth_xlnx_gem_handle_tx_done(dev);
		}
	}
	if ((reg_val & ETH_XLNX_GEM_IXR_FRAMERX_BIT) != 0) {
		if (dev_conf->defer_rxp_to_queue == 1) {
			k_work_submit(&dev_data->rx_pend_work);
		} else {
			printk("RX: handle in ISR\n");
			eth_xlnx_gem_handle_rx_pending(dev);
		}
	}

	/* Clear all interrupt status bits so that the interrupt is de-asserted
	 * by the GEM. -> TXSR/RXSR are read/cleared by either eth_xlnx_gem_-
	 * handle_tx_done or eth_xlnx_gem_handle_rx_pending if those actions
	 * are not deferred to the system's work queue for the current inter-
	 * face. If the latter is the case, those registers will be read/
	 * cleared whenever the corresponding work item submitted from within
	 * this ISR is being processed. */
	sys_write32(0xFFFFFFFF, dev_conf->base_addr + ETH_XLNX_GEM_ISR_OFFSET);
}

/**
 * @brief GEM data send function
 * GEM data send function. Blocks until a TX complete notification has been
 * received & processed.
 *
 * @param dev Pointer to the device data struct
 * @param pkt Pointer to the data packet to be sent
 * @retval -EINVAL in case of invalid parameters, e.g. zero data length
 * @retval -EIO in case of:
 *         (1) the attempt to TX data while the link is down,
 *         (2) the attempt to TX data while no free buffers are available
 *             in the DMA memory area,
 *         (3) the transmission completion notification timing out
 * @retval 0 if the packet was transmitted successfully
 */
static int eth_xlnx_gem_send(struct device *dev, struct net_pkt *pkt)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf       = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data       = DEV_DATA(dev);
	struct net_buf			*frag           = NULL;
	uint16_t			pkt_len         = 0;
	uint16_t			used_in_buf     = 0;
	uint16_t			rem_frag_len    = 0;
	uint16_t			frag_data_offs  = 0;
	uint8_t				bds_reqd        = 0;
	uint8_t				curr_bd_idx     = 0;
	uint8_t				*tx_buffer_offs = NULL;
	uint32_t			reg_val         = 0;

	pkt_len = net_pkt_get_len(pkt);
	if (pkt_len == 0) {
		LOG_ERR("GEM@0x%08X cannot TX, zero packet length",
			dev_conf->base_addr);
		return -EINVAL;
	}

	/* Check if enough buffer descriptors are available for the amount of
	 * data to be transmitted */
	bds_reqd = (uint8_t)((pkt_len + (dev_conf->tx_buffer_size - 1))
		/ dev_conf->tx_buffer_size);
	k_sem_take(&(dev_data->txbd_ring.ring_sem), K_FOREVER);
	if (bds_reqd > dev_data->txbd_ring.free_bds) {
		LOG_ERR(
			"GEM@0x%08X cannot TX, packet length %hu requires"
			"%hhu BDs, current free count is only %hhu",
			dev_conf->base_addr,
			pkt_len,
			bds_reqd,
			dev_data->txbd_ring.free_bds);

		k_sem_give(&(dev_data->txbd_ring.ring_sem));
		return -EIO;
	}
	curr_bd_idx = dev_data->txbd_ring.next_to_use;
	dev_data->txbd_ring.next_to_use =
		(curr_bd_idx + bds_reqd) % dev_conf->txbd_count;
	dev_data->txbd_ring.free_bds -= bds_reqd;
	k_sem_give(&(dev_data->txbd_ring.ring_sem));
	printk("\nTX: %u bytes = %u BDs, first [%u], next [%u], now free %u\n",
		pkt_len,
		bds_reqd,
		curr_bd_idx,
		(curr_bd_idx + bds_reqd) % dev_conf->txbd_count,
		dev_data->txbd_ring.free_bds);

	/* Calculate the base pointer of the target TX buffer */
	tx_buffer_offs = (uint8_t*)(dev_data->first_tx_buffer +
		(dev_conf->tx_buffer_size * curr_bd_idx));

	/* Copy packet data to the target TX data buffers, prepare BDs for
	 * transmission */
	uint32_t dbg_frag_iter = 0;
	for (frag = pkt->frags; frag; frag = frag->frags) {
		rem_frag_len   = frag->len;
		frag_data_offs = 0;
		printk("Frag [%02u] len %u\n", dbg_frag_iter++, rem_frag_len);
		while (rem_frag_len > 0) {
			if ((used_in_buf + rem_frag_len)
				<= dev_conf->tx_buffer_size) {
				/* The current packet fragment fits into the
				 * buffer pointed to by the current BD */
				printk("  %u from frag offs [%u] to BD[%u] offs [%u]\n",
					rem_frag_len, frag_data_offs, curr_bd_idx, used_in_buf);
				memcpy((void*)(&tx_buffer_offs[used_in_buf]),
					(void*)(&frag->data[frag_data_offs]),
					rem_frag_len);
				used_in_buf  += rem_frag_len;
				rem_frag_len  = 0;
			} else {
				/* Only a part of the current packet fragment
				 * still fits into the buffer pointed to by
				 * the current BD -> copy the first part, set
				 * up the BD control word, move on to the next
				 * buffer descriptor */
				printk("  %u from frag offs [%u] to BD[%u] offs [%u]\n",
					(dev_conf->tx_buffer_size - used_in_buf), frag_data_offs, curr_bd_idx, used_in_buf);
				memcpy((void*)(&tx_buffer_offs[used_in_buf]),
					(void*)(&frag->data[frag_data_offs]),
					(dev_conf->tx_buffer_size - used_in_buf));
				rem_frag_len   -= (dev_conf->tx_buffer_size - used_in_buf);
				frag_data_offs += (dev_conf->tx_buffer_size - used_in_buf);

				/* Read the current BD's control word, set the
				 * length information, update BD */
				reg_val = sys_read32(
					(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));

				/* Preserve the 'wrap' bit of the current
				 * control word */
				reg_val &= ETH_XLNX_GEM_TXBD_WRAP_BIT;
				reg_val |= ((reg_val & ~ETH_XLNX_GEM_TXBD_LEN_MASK) |
					dev_conf->tx_buffer_size);
					printk("  BD[%u] len for TX %u\n", curr_bd_idx, dev_conf->tx_buffer_size);
				sys_write32(reg_val,
					(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));

				/* Move on to the next BD */
				curr_bd_idx    = (curr_bd_idx + 1) % dev_conf->txbd_count;
				used_in_buf    = 0;
				tx_buffer_offs = (uint8_t*)(dev_data->first_tx_buffer +
					(dev_conf->tx_buffer_size * curr_bd_idx));
			}
		}
	}

	/* All fragments transferred to the buffers, configure the current
	 * (= last) BD */
	reg_val  = sys_read32(
		(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));

	/* Preserve the 'wrap' bit of the current control word */
	reg_val &= ETH_XLNX_GEM_TXBD_WRAP_BIT;
	reg_val |= (((reg_val & ~ETH_XLNX_GEM_TXBD_LEN_MASK) | used_in_buf)
		| ETH_XLNX_GEM_TXBD_LAST_BIT); /* Set length + 'last' bit */
	sys_write32(reg_val,
		(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));
	printk("  BD[%u] len for TX %u\n", curr_bd_idx, used_in_buf);
	/* Set the start TX bit in the gem.net_ctrl register */
	reg_val  = sys_read32(
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);
	reg_val |= ETH_XLNX_GEM_NWCTRL_STARTTX_BIT;
	sys_write32(reg_val,
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);

	/* Block until TX has completed */
	int rc = k_sem_take(&dev_data->tx_done_sem, K_MSEC(5000));
	if (rc < 0) {
		LOG_ERR("GEM@0x%08X TX confirmation timed out",
			dev_conf->base_addr);
		return -EIO;
	}
	printk("TX done\n");

#ifdef CONFIG_NET_STATISTICS_ETHERNET
	dev_data->stats.bytes.sent += pkt_len;
	dev_data->stats.pkts.tx++;
#endif
	return 0;
}

/**
 * @brief GEM device start function
 * GEM device start function. Clears all status registers and any
 * pending interrupts, enables RX and TX, enables interrupts. If
 * no PHY is managed by the current driver instance, this function
 * also declares the physical link up at the configured nominal
 * link speed.
 *
 * @param dev Pointer to the device data struct
 * @retval    0 upon successful completion
 */
static int eth_xlnx_gem_start_device(struct device *dev) 
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t 			reg_val   = 0;

	if (dev_data->started == 1) {
		return 0;
	} else {
		dev_data->started = 1;
	}

	/* Disable & clear all the MAC interrupts */
	sys_write32(ETH_XLNX_GEM_IXR_ALL_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_IDR_OFFSET);
	sys_write32(ETH_XLNX_GEM_IXR_ALL_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_ISR_OFFSET);

	/* Clear RX & TX status registers */
	sys_write32(0xFFFFFFFF,
		dev_conf->base_addr + ETH_XLNX_GEM_TXSR_OFFSET);
	sys_write32(0xFFFFFFFF,
		dev_conf->base_addr + ETH_XLNX_GEM_RXSR_OFFSET);

	/* RX and TX enable */
	reg_val  = sys_read32(
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);
	reg_val |= (ETH_XLNX_GEM_NWCTRL_RXEN_BIT
		| ETH_XLNX_GEM_NWCTRL_TXEN_BIT);
	sys_write32(reg_val,
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);

	/* Enable all the MAC interrupts */
	sys_write32(ETH_XLNX_GEM_IXR_ALL_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_IER_OFFSET);

	/* Queue the link status monitoring delayed work item */
	if (k_delayed_work_remaining_get(
		&dev_data->phy_poll_delayed_work) == 0) {
		k_delayed_work_submit(&dev_data->phy_poll_delayed_work,
			K_NO_WAIT);
	}

	LOG_DBG("GEM@0x%08X started", dev_conf->base_addr);
	return 0;
}

/**
 * @brief GEM device stop function
 * GEM device stop function. Disables all interrupts, disables
 * RX and TX, clears all status registers. If no PHY is managed
 * by the current driver instance, this function also declares
 * the physical link down.
 *
 * @param dev Pointer to the device data struct
 * @retval    0 upon successful completion
 */
static int eth_xlnx_gem_stop_device(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t 			reg_val   = 0;

	if (dev_data->started == 0) {
		return 0;
	} else {
		dev_data->started = 0;
	}

	/* Cancel the delayed work that triggers the link monitoring */
	if (k_delayed_work_remaining_get(
		&dev_data->phy_poll_delayed_work) != 0) {
		k_delayed_work_cancel(&dev_data->phy_poll_delayed_work);
	}

	/* RX and TX disable */
	reg_val  = sys_read32(
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);
	reg_val &= (~(ETH_XLNX_GEM_NWCTRL_RXEN_BIT
		| ETH_XLNX_GEM_NWCTRL_TXEN_BIT));
	sys_write32(reg_val, 
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);

	/* Disable & clear all the MAC interrupts */
	sys_write32(ETH_XLNX_GEM_IXR_ALL_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_IDR_OFFSET);
	sys_write32(ETH_XLNX_GEM_IXR_ALL_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_ISR_OFFSET);

	/* Clear RX & TX status registers */
	sys_write32(0xFFFFFFFF,
		dev_conf->base_addr + ETH_XLNX_GEM_TXSR_OFFSET);
	sys_write32(0xFFFFFFFF,
		dev_conf->base_addr + ETH_XLNX_GEM_RXSR_OFFSET);

	LOG_DBG("GEM@0x%08X stopped", dev_conf->base_addr);
	return 0;
}

/**
 * @brief GEM capability request function
 * Returns the capabilities of the GEM controller as an enumeration.
 * All of the data returned is derived from the device configuration
 * of the current GEM device instance.
 *
 * @param dev Pointer to the device data struct
 * @return Enumeration containing the current GEM device's capabilities
 */
static enum ethernet_hw_caps eth_xlnx_gem_get_capabilities(
	struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	enum ethernet_hw_caps 		caps      = (enum ethernet_hw_caps)0;

	if (dev_conf->max_link_speed == LINK_1GBIT) {
		if (dev_conf->phy_advertise_lower == 1) {
			caps |= (
				  ETHERNET_LINK_1000BASE_T
				| ETHERNET_LINK_100BASE_T
				| ETHERNET_LINK_10BASE_T);
		} else {
			caps |= ETHERNET_LINK_1000BASE_T;
		}
	} else if (dev_conf->max_link_speed == LINK_100MBIT) {
		if (dev_conf->phy_advertise_lower == 1) {
			caps |= (
				  ETHERNET_LINK_100BASE_T
				| ETHERNET_LINK_10BASE_T);
		} else {
			caps |= ETHERNET_LINK_100BASE_T;
		}
	} else {
		caps |= ETHERNET_LINK_10BASE_T;
	}

	if (dev_conf->enable_rx_chksum_offload == 1) {
		caps |= ETHERNET_HW_RX_CHKSUM_OFFLOAD;
	}

	if (dev_conf->enable_tx_chksum_offload == 1) {
		caps |= ETHERNET_HW_TX_CHKSUM_OFFLOAD;
	}

	if (dev_conf->enable_fdx == 1) {
		caps |= ETHERNET_DUPLEX_SET;
	}

	if (dev_conf->copy_all_frames == 1) {
		caps |= ETHERNET_PROMISC_MODE;
	}

	/* TODO: does the VLAN capability flag imply specific configuration
	 * of the controller? The GEM itself is VLAN-capable. */
	//caps |= ETHERNET_HW_VLAN;
	return caps;
}

#ifdef CONFIG_NET_STATISTICS_ETHERNET
/**
 * @brief GEM statistics data request function
 * Returns a pointer to the statistics data of the current GEM controller.
 *
 * @param dev Pointer to the device data struct
 * @return Pointer to the current GEM device's statistics data
 */
static struct net_stats_eth *eth_xlnx_gem_stats(struct device *dev)
{
	return &(DEV_DATA(dev)->stats);
}
#endif

/**
 * @brief GEM Hardware reset function
 * Resets the current GEM device. Called from within the device
 * initialization function.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_reset_hw(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(dev);

	/* Controller reset sequence as described in the Zynq-7000 TRM,
	 * chapter 16.3.1. */

	/* Clear the NWCTRL register */
	sys_write32(0x00000000,
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);

	/* Clear the statistics counters */
	sys_write32(ETH_XLNX_GEM_STATCLR_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_NWCTRL_OFFSET);

	/* Clear the RX/TX status registers */
	sys_write32(ETH_XLNX_GEM_TXSRCLR_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_TXSR_OFFSET);
	sys_write32(ETH_XLNX_GEM_RXSRCLR_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_RXSR_OFFSET);

	/* Disable all interrupts */
	sys_write32(ETH_XLNX_GEM_IDRCLR_MASK,
		dev_conf->base_addr + ETH_XLNX_GEM_IDR_OFFSET);

	/* Clear the buffer queues */
	sys_write32(0x00000000,
		dev_conf->base_addr + ETH_XLNX_GEM_RXQBASE_OFFSET);
	sys_write32(0x00000000,
		dev_conf->base_addr + ETH_XLNX_GEM_TXQBASE_OFFSET);
}

/**
 * @brief GEM clock configuration function
 * Calculates the pre-scalers for the TX clock to match the current
 * (if an associated PHY is managed) or nominal link speed. Called
 * from within the device initialization function.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_configure_clocks(struct device *dev)
{
	/* Clock source configuration for the respective GEM as described
	 * in the Zynq-7000 TRM, chapter 16.3.3, is not tackled here. This
	 * is performed by the PS7Init code. Only the DIVISOR and DIVISOR1
	 * values for the respective GEM's TX clock are calculated here. */

	struct eth_xlnx_gem_dev_cfg  *dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data *dev_data = DEV_DATA(dev);

	uint32_t div0         = 0;
	uint32_t div1         = 0;
	uint32_t target       = 0;
	uint32_t tmp          = 0;
	uint32_t clk_ctrl_reg = 0;

	if (dev_conf->init_phy == 0
		|| dev_data->eff_link_speed == LINK_DOWN) {
		/* Run-time data indicates 'link down' or PHY management
		 * is disabled for the current device -> this indicates the
		 * initial device initialization. Once the PHY status polling
		 * delayed work handler has picked up the result of the auto-
		 * negotiation (if enabled), this if-statement will evaluate
		 * to false. */
		if (dev_conf->max_link_speed == LINK_10MBIT) {
			target = 2500000;   /* Target frequency: 2.5 MHz */
		} else if (dev_conf->max_link_speed == LINK_100MBIT) {
			target = 25000000;  /* Target frequency: 25 MHz */
		} else if (dev_conf->max_link_speed == LINK_1GBIT) {
			target = 125000000; /* Target frequency: 125 MHz */
		}
	} else if (dev_data->eff_link_speed != LINK_DOWN) {
		/* Use the effective link speed instead of the maximum/nominal
		 * link speed for clock configuration. */
		if (dev_data->eff_link_speed == LINK_10MBIT) {
			target = 2500000;   /* Target frequency: 2.5 MHz */
		} else if (dev_data->eff_link_speed == LINK_100MBIT) {
			target = 25000000;  /* Target frequency: 25 MHz */
		} else if (dev_data->eff_link_speed == LINK_1GBIT) {
			target = 125000000; /* Target frequency: 125 MHz */
		}
	}

	/* Caclculate the divisors for the target frequency.
	 * The frequency of the PLL to which the divisors shall be applied are
	 * provided in the respective GEM's device tree data. */
	for (div0 = 1; div0 < 64; div0++) {
		for (div1 = 1; div1 < 64; div1++) {
			tmp = ((dev_conf->pll_clock_frequency / div0) / div1);
			if (tmp >= (target - 2) && tmp <= (target + 2)) {
				break;
			}
		}
		if (tmp >= (target - 2) && tmp <= (target + 2)) {
			break;
		}
	}

	clk_ctrl_reg = sys_read32(dev_conf->clk_ctrl_reg_address);
#if defined(CONFIG_SOC_XILINX_ZYNQ7000)
	/* Zynq-7000: slcr.GEMx_CLK_CTRL
	 * div0 bits [13..8], div1 bits [25..20]
	 * Requires unlock/lock commands to the SLCR */
	clk_ctrl_reg &= ~((ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK <<
		ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT) |
		(ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK <<
		ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT));
	clk_ctrl_reg |=
		((div0 & ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK) <<
		ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT) |
		((div1 & ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK) <<
		ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT);

	sys_write32(ETH_XLNX_SLCR_UNLOCK_KEY,
		ETH_XLNX_SLCR_UNLOCK_REGISTER_ADDRESS);
	sys_write32(clk_ctrl_reg, dev_conf->clk_ctrl_reg_address);
	sys_write32(ETH_XLNX_SLCR_LOCK_KEY,
		ETH_XLNX_SLCR_LOCK_REGISTER_ADDRESS);
#elif defined(CONFIG_SOC_XILINX_ZYNQMP)
	/* ZynqMP: crl_apb.GEMx_REF_CTRL
	 * div0 bits [13..8], div1 bits [21..16]
	 * Unlock CRL_APB write access if the write protect bit
	 * is currently set, restore it afterwards. */
	clk_ctrl_reg &= ~((ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK <<
		ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT) |
		(ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK <<
		ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT));
	clk_ctrl_reg |=
		((div0 & ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK) <<
		ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT) |
		((div1 & ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK) <<
		ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT);

	tmp = sys_read32(ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	if ((tmp & ETH_XLNX_CRL_APB_WPROT_BIT) > 0) {
		sys_write32((tmp & ~ETH_XLNX_CRL_APB_WPROT_BIT),
			ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	}
	sys_write32(clk_ctrl_reg, dev_conf->clk_ctrl_reg_address);
	if ((tmp & ETH_XLNX_CRL_APB_WPROT_BIT) > 0) {
		sys_write32(tmp, ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	}
#endif /* CONFIG_SOC_XILINX_ZYNQ7000 / CONFIG_SOC_XILINX_ZYNQMP */

	LOG_DBG("GEM@0x%08X set clock dividers div0/1 %u/%u for target "
		"frequency %u Hz", dev_conf->base_addr, div0, div1, target);
}

/**
 * @brief GEM initial Network Configuration Register setup function
 * Writes the contents of the current GEM device's Network Configuration
 * Register (NWCFG / gem.net_cfg). Called from within the device
 * initialization function. Implementation differs depending on whether
 * the current target is a Zynq-7000 or a ZynqMP.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_set_initial_nwcfg(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf   = DEV_CFG(dev);
	uint32_t 			reg_val     = 0;

	if (dev_conf->ignore_igp_rxer == 1) {
		/* [30]     ignore IPG rx_er */
		reg_val |= ETH_XLNX_GEM_NWCFG_IGNIPGRXERR_BIT;
	}
	if (dev_conf->disable_reject_nsp == 1) {
		/* [29]     disable rejection of non-standard preamble */
		reg_val |= ETH_XLNX_GEM_NWCFG_BADPREAMBEN_BIT;
	}
	if (dev_conf->enable_igp_stretch == 1) {
		/* [28]     enable IPG stretch */
		reg_val |= ETH_XLNX_GEM_NWCFG_IPDSTRETCH_BIT;
	}
	if (dev_conf->enable_sgmii_mode == 1) {
		/* [27]     SGMII mode enable */
		reg_val |= ETH_XLNX_GEM_NWCFG_SGMIIEN_BIT;
	}
	if (dev_conf->disable_reject_fcs_crc_errors == 1) {
		/* [26]     disable rejection of FCS/CRC errors */
		reg_val |= ETH_XLNX_GEM_NWCFG_FCSIGNORE_BIT;
	}
	if (dev_conf->enable_rx_halfdup_while_tx == 1) {
		/* [25]     RX half duplex while TX enable */
		reg_val |= ETH_XLNX_GEM_NWCFG_HDRXEN_BIT;
	}
	if (dev_conf->enable_rx_chksum_offload == 1) {
		/* [24]     enable RX IP/TCP/UDP checksum offload */
		reg_val |= ETH_XLNX_GEM_NWCFG_RXCHKSUMEN_BIT;
	}
	if (dev_conf->disable_pause_copy == 1) {
		/* [23]     Do not copy pause Frames to memory */
		reg_val |= ETH_XLNX_GEM_NWCFG_PAUSECOPYDI_BIT;
	}
	/* [22..21] Data bus width */
	reg_val |= (((uint32_t)(dev_conf->amba_dbus_width)
		& ETH_XLNX_GEM_NWCFG_DBUSW_MASK)
		<< ETH_XLNX_GEM_NWCFG_DBUSW_SHIFT);
	/* [20..18] MDC clock divider */
	reg_val |= (((uint32_t)dev_conf->mdc_divider
		& ETH_XLNX_GEM_NWCFG_MDC_MASK)
		<< ETH_XLNX_GEM_NWCFG_MDC_SHIFT);
	if (dev_conf->discard_rx_fcs == 1) {
		/* [17]     Discard FCS from received frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_FCSREM_BIT;
	}
	if (dev_conf->discard_rx_length_errors == 1) {
		/* [16]     RX length error discard */
		reg_val |= ETH_XLNX_GEM_NWCFG_LENGTHERRDSCRD_BIT;
	}
	/* [15..14] RX buffer offset */
	reg_val |= (((uint32_t)dev_conf->hw_rx_buffer_offset
		& ETH_XLNX_GEM_NWCFG_RXOFFS_MASK)
		<< ETH_XLNX_GEM_NWCFG_RXOFFS_SHIFT);
	if (dev_conf->enable_pause == 1) {
		/* [13]     Enable pause TX */
		reg_val |= ETH_XLNX_GEM_NWCFG_PAUSEEN_BIT;
	}
	if (dev_conf->enable_tbi == 1) {
		/* [11]     enable TBI instead of GMII/MII */
		reg_val |= ETH_XLNX_GEM_NWCFG_TBIINSTEAD_BIT;
	}
	if (dev_conf->ext_addr_match == 1) {
		/* [09]     External address match enable */
		reg_val |= ETH_XLNX_GEM_NWCFG_EXTADDRMATCHEN_BIT;
	}
	if (dev_conf->enable_1536_frames == 1) {
		/* [08]     Enable 1536 byte frames reception */
		reg_val |= ETH_XLNX_GEM_NWCFG_1536RXEN_BIT;
	}
	if (dev_conf->enable_ucast_hash == 1) {
		/* [07]     Receive unicast hash frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_UCASTHASHEN_BIT;
	}
	if (dev_conf->enable_mcast_hash == 1) {
		/* [06]     Receive multicast hash frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT;
	}
	if (dev_conf->disable_bcast == 1) {
		/* [05]     Do not receive broadcast frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_BCASTDIS_BIT;
	}
	if (dev_conf->copy_all_frames == 1) {
		/* [04]     Copy all frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_COPYALLEN_BIT;
	}
	if (dev_conf->discard_non_vlan == 1) {
		/* [02]     Receive only VLAN frames */
		reg_val |= ETH_XLNX_GEM_NWCFG_NVLANDISC_BIT;
	}
	if (dev_conf->enable_fdx == 1) {
		/* [01]     enable Full duplex */
		reg_val |= ETH_XLNX_GEM_NWCFG_FDEN_BIT;
	}
	if (dev_conf->max_link_speed == LINK_100MBIT) {
		/* [00]     10 or 100 Mbps */
		reg_val |= ETH_XLNX_GEM_NWCFG_100_BIT;
	} else if (dev_conf->max_link_speed == LINK_1GBIT) {
		/* [10]     Gigabit mode enable */
		reg_val |= ETH_XLNX_GEM_NWCFG_1000_BIT;
	}
	/* No else-branch for 10Mbit/s mode:
	 * in 10 MBit/s mode, both bits [00] and [10] remain 0 */

	/* Write the assembled register contents to gem.net_cfg */
	sys_write32(reg_val, dev_conf->base_addr + ETH_XLNX_GEM_NWCFG_OFFSET);
}

/**
 * @brief GEM Network Configuration Register link speed update function
 * Updates only the link speed-related bits of the Network Configuration
 * register. This is called from within #eth_xlnx_gem_poll_phy.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_set_nwcfg_link_speed(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf  = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data  = DEV_DATA(dev);
	uint32_t 			reg_val    = 0;

	/* Read the current gem.net_cfg register contents and mask out
	 * the link speed-related bits */
	reg_val = sys_read32(dev_conf->base_addr + ETH_XLNX_GEM_NWCFG_OFFSET);
	reg_val &=
		~(ETH_XLNX_GEM_NWCFG_1000_BIT | ETH_XLNX_GEM_NWCFG_100_BIT);

	/* No bits to set for 10 Mbps. 100 Mbps and 1 Gbps set one bit each. */
	if (dev_data->eff_link_speed == LINK_100MBIT) {
		reg_val |= ETH_XLNX_GEM_NWCFG_100_BIT;
	} else if (dev_data->eff_link_speed == LINK_1GBIT) {
		reg_val |= ETH_XLNX_GEM_NWCFG_1000_BIT;
	}

	/* Write the assembled register contents to gem.net_cfg */
	sys_write32(reg_val, dev_conf->base_addr + ETH_XLNX_GEM_NWCFG_OFFSET);
}

/**
 * @brief GEM MAC address setup function
 * Acquires the MAC address to be assigned to the current GEM device
 * from the device configuration data which in turn acquires it from
 * the device tree data, then writes it to the gem.spec_addr1_bot/LADDR1L
 * and gem.spec_addr1_top/LADDR1H registers. Called from within the device
 * initialization function.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_set_mac_address(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf  = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data  = DEV_DATA(dev);
	uint32_t			regval_top = 0;
	uint32_t			regval_bot = 0;

	/* Reverse the order of the MAC bytes: if the high byte of the
	 * address is specified first in the array, the bytes will end up
	 * in the config registers (gem.spec_addr1_bot, gem.spec_addr1_top)
	 * in reverse order, prompting the controller to discard any non-
	 * bcast packets since the packets specifically addressed to us
	 * don't make it past the MAC address filter which uses the values
	 * from the config registers. */
	regval_bot  = (dev_data->mac_addr[0] & 0xFF);
	regval_bot |= (dev_data->mac_addr[1] & 0xFF) << 8;
	regval_bot |= (dev_data->mac_addr[2] & 0xFF) << 16;
	regval_bot |= (dev_data->mac_addr[3] & 0xFF) << 24;

	regval_top  = (dev_data->mac_addr[4] & 0xFF);
	regval_top |= (dev_data->mac_addr[5] & 0xFF) << 8;

	sys_write32(regval_bot,
		dev_conf->base_addr + ETH_XLNX_GEM_LADDR1L_OFFSET);
	sys_write32(regval_top,
		dev_conf->base_addr + ETH_XLNX_GEM_LADDR1H_OFFSET);

	LOG_DBG(
		"GEM@0x%08X MAC %02X:%02X:%02X:%02X:%02X:%02X",
		dev_conf->base_addr,
		dev_data->mac_addr[0],
		dev_data->mac_addr[1],
		dev_data->mac_addr[2],
		dev_data->mac_addr[3],
		dev_data->mac_addr[4],
		dev_data->mac_addr[5]);
}

/**
 * @brief GEM initial DMA Control Register setup function
 * Writes the contents of the current GEM device's DMA Control Register
 * (DMACR / gem.dma_cfg). Called from within the device initialization
 * function.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_set_initial_dmacr(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	uint32_t 			reg_val   = 0;

	/* gem.dma_cfg register bit (field) definitions:
	 * comp. Zynq-7000 TRM, p. 1278 ff. */

	if (dev_conf->disc_rx_ahb_unavail == 1) {
		/* [24] Discard RX packet when AHB unavailable */
		reg_val |= ETH_XLNX_GEM_DMACR_DISCNOAHB_BIT;
	}
	/* [23..16] DMA RX buffer size in AHB system memory
	 *    e.g.: 0x02 = 128, 0x18 = 1536, 0xA0 = 10240 */
	reg_val |= (
		((dev_conf->rx_buffer_size / 64)
		& ETH_XLNX_GEM_DMACR_RX_BUF_MASK)
		<< ETH_XLNX_GEM_DMACR_RX_BUF_SHIFT);
	if (dev_conf->enable_tx_chksum_offload == 1) {
		/* [11] TX TCP/UDP/IP checksum offload to GEM */
		reg_val |= ETH_XLNX_GEM_DMACR_TCP_CHKSUM_BIT;
	}
	if (dev_conf->tx_buffer_size_full == 1) {
		/* [10] TX buffer memory size select */
		reg_val |= ETH_XLNX_GEM_DMACR_TX_SIZE_BIT;
	}
	/* [09..08] RX packet buffer memory size select
	 *          0 = 1kB, 1 = 2kB, 2 = 4kB, 3 = 8kB */
	reg_val |= (
		((uint32_t)dev_conf->hw_rx_buffer_size
		<< ETH_XLNX_GEM_DMACR_RX_SIZE_SHIFT)
		& ETH_XLNX_GEM_DMACR_RX_SIZE_MASK);
	if (dev_conf->enable_ahb_packet_endian_swap == 1) {
		/* [07] AHB packet data endian swap enable */
		reg_val |= ETH_XLNX_GEM_DMACR_ENDIAN_BIT;
	}
	if (dev_conf->enable_ahb_md_endian_swap == 1) {
		/* [06] AHB mgmt descriptor endian swap enable */
		reg_val |= ETH_XLNX_GEM_DMACR_DESCR_ENDIAN_BIT;
	}
	/* [04..00] AHB fixed burst length for DMA ops.
	 *          00001 = single AHB bursts,
	 *          001xx = attempt to use INCR4  bursts,
	 *          01xxx = attempt to use INCR8  bursts,
	 *          1xxxx = attempt to use INCR15 bursts */
	reg_val |= ((uint32_t)dev_conf->ahb_burst_length
		& ETH_XLNX_GEM_DMACR_AHB_BURST_LENGTH_MASK);

	/* Write the assembled register contents */
	sys_write32(reg_val, dev_conf->base_addr + ETH_XLNX_GEM_DMACR_OFFSET);
}

/**
 * @brief GEM associated PHY detection and setup function
 * If the current GEM device shall manage an associated PHY, its detection
 * and configuration is performed from within this function. Called from
 * within the device initialization function. This function refers to
 * functionality implemented in the phy_xlnx_gem module.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_init_phy(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	int 				detect_rc = 0;

	LOG_DBG("GEM@0x%08X attempting to initialize associated PHY",
		dev_conf->base_addr);

	/* The phy_xlnx_gem_detect function checks if a valid PHY
	 * ID is returned when reading the corresponding high / low
	 * ID registers for all valid MDIO addresses. If a compatible
	 * PHY is detected, the function writes a pointer to the
	 * vendor-specific implementations of the PHY management
	 * functions to the run-time device data struct, along with
	 * the ID and the MDIO address of the detected PHY (dev_data->
	 * phy_id, dev_data->phy_addr, dev_data->phy_access_api). */
	detect_rc = phy_xlnx_gem_detect(dev);

	if (detect_rc == 0 &&
		dev_data->phy_id != 0x00000000 &&
		dev_data->phy_id != 0xFFFFFFFF &&
		dev_data->phy_access_api != NULL)
	{
		/* A compatible PHY was detected -> reset & configure it */
		dev_data->phy_access_api->phy_reset_func(dev);
		dev_data->phy_access_api->phy_configure_func(dev);
	} else {
		LOG_WRN("GEM@0x%08X no compatible PHY detected",
			dev_conf->base_addr);
	}
}

/**
 * @brief GEM associated PHY status polling function
 * This handler of a delayed work item is called from the context of
 * the system work queue, once it reaches the end of its execution, it
 * re-enqueues itself. It is only active if the current GEM device
 * manages its associated PHY. Link state / speed changes are polled,
 * which may result in the link state change being propagated (carrier
 * on/off) and / or the TX clock being reconfigured to match the current
 * link speed. This function refers to functionality implemented in
 * the phy_xlnx_gem module.
 *
 * @param item Pointer to the delayed work item which facilitates
 *             access to the current device's configuration data
 */
static void eth_xlnx_gem_poll_phy(struct k_work *item)
{
	struct eth_xlnx_gem_dev_data *dev_data = CONTAINER_OF(
		item, struct eth_xlnx_gem_dev_data, phy_poll_delayed_work);
	struct device *dev = net_if_get_device(dev_data->iface);
	struct eth_xlnx_gem_dev_cfg *dev_conf = DEV_CFG(dev);

	uint16_t phy_status  = 0x0000;
	uint8_t  link_status = 0x00;

	if (dev_data->phy_access_api != NULL) {
		/* A supported PHY is managed by the driver */
		phy_status = dev_data->phy_access_api->
			phy_poll_status_change_func(dev);

		if ((phy_status & (
			PHY_XLNX_GEM_EVENT_LINK_SPEED_CHANGED |
			PHY_XLNX_GEM_EVENT_LINK_STATE_CHANGED |
			PHY_XLNX_GEM_EVENT_AUTONEG_COMPLETE)) != 0) {

			/* Read the PHY's link status. Handling a 'link down'
			 * event the simplest possible case. */
			link_status = dev_data->phy_access_api->
				phy_poll_link_status_func(dev);

			if (link_status == 0) {
				/* Link is down -> propagate to the Ethernet
				 * layer that the link has gone down. */
				dev_data->eff_link_speed = LINK_DOWN;
				net_eth_carrier_off(dev_data->iface);

				LOG_DBG("GEM@0x%08X link down",
					dev_conf->base_addr);
			} else {
				/* A link has been detected, which, depending
				 * on the driver's configuration, might have
				 * a different speed than the previous link.
				 * Therefore, the clock dividers must be ad-
				 * justed accordingly. */
				dev_data->eff_link_speed =
					dev_data->phy_access_api->
						phy_poll_link_speed_func(dev);

				eth_xlnx_gem_configure_clocks(dev);
				eth_xlnx_gem_set_nwcfg_link_speed(dev);
				net_eth_carrier_on(dev_data->iface);

				LOG_DBG("GEM@0x%08X link up, %s",
					dev_conf->base_addr,
					(dev_data->eff_link_speed   == LINK_1GBIT)
					? "1 GBit/s"
					: (dev_data->eff_link_speed == LINK_100MBIT)
					? "100 MBit/s"
					: (dev_data->eff_link_speed == LINK_10MBIT)
					? "10 MBit/s"
					: "undefined / link down");
			}
		}

		/* Re-submit the delayed work using the interval from the device
		* configuration data. */
		k_delayed_work_submit(&dev_data->phy_poll_delayed_work,
			K_MSEC(dev_conf->phy_poll_interval));

	} else {
		/* The current driver instance doesn't manage a PHY or no
		 * supported PHY was detected -> pretend the configured max.
		 * link speed is the effective link speed and that the link
		 * is up. The delayed work item won't be re-scheduled, as
		 * there isn't anything to poll for. */
		LOG_DBG("GEM@0x%08X PHY not managed by the driver or no "
			"compatible PHY detected, assuming link up at %s",
			dev_conf->base_addr,
			(dev_conf->max_link_speed == LINK_1GBIT)
			? "1 GBit/s"
			: (dev_conf->max_link_speed == LINK_100MBIT)
			? "100 MBit/s"
			: (dev_conf->max_link_speed == LINK_10MBIT)
			? "10 MBit/s"
			: "undefined");
		dev_data->eff_link_speed = dev_conf->max_link_speed;
		eth_xlnx_gem_configure_clocks(dev);
		eth_xlnx_gem_set_nwcfg_link_speed(dev);
		net_eth_carrier_on(dev_data->iface);
	}
}

/**
 * @brief GEM DMA memory area setup function
 * Sets up the DMA memory area to be used by the current GEM device.
 * Called from within the device initialization function or from within
 * the context of the PHY status polling delayed work handler.
 *
 * @param dev Pointer to the device data struct
 */
static void eth_xlnx_gem_configure_buffers(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data = DEV_DATA(dev);
	struct eth_xlnx_gem_bd		*bdptr    = NULL;
	uint32_t			buf_iter  = 0;

	/* Initial configuration of the RX/TX BD rings */
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0
	if (dev_conf->base_addr == DT_REG_ADDR(DT_NODELABEL(gem0))) {
#ifdef CONFIG_ETH_XLNX_GEM_PORT_0_DMA_FIXED
		struct eth_xlnx_dma_area_gem0 *dma_area_gem0 =
			(struct eth_xlnx_dma_area_gem0*)
				CONFIG_ETH_XLNX_GEM_PORT_0_DMA_BASE_ADDRESS;
		dev_data->rxbd_ring.first_bd = &(dma_area_gem0->rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem0->tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem0->rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem0->tx_buffer;
#else
		dev_data->rxbd_ring.first_bd = &(dma_area_gem0.rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem0.tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem0.rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem0.tx_buffer;
#endif /* CONFIG_ETH_XLNX_GEM_PORT_0_DMA_FIXED */
	}
#endif /* CONFIG_ETH_XLNX_GEM_PORT_0 */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_1
	if (dev_conf->base_addr == DT_REG_ADDR(DT_NODELABEL(gem1))) {
#ifdef CONFIG_ETH_XLNX_GEM_PORT_1_DMA_FIXED
		struct eth_xlnx_dma_area_gem1 *dma_area_gem1 =
			(struct eth_xlnx_dma_area_gem1*)
				CONFIG_ETH_XLNX_GEM_PORT_1_DMA_BASE_ADDRESS;
		dev_data->rxbd_ring.first_bd = &(dma_area_gem1->rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem1->tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem1->rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem1->tx_buffer;
#else
		dev_data->rxbd_ring.first_bd = &(dma_area_gem1.rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem1.tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem1.rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem1.tx_buffer;
#endif /* CONFIG_ETH_XLNX_GEM_PORT_1_DMA_FIXED */
	}
#endif /* CONFIG_ETH_XLNX_GEM_PORT_1 */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_2
	if (dev_conf->base_addr == DT_REG_ADDR(DT_NODELABEL(gem2))) {
#ifdef CONFIG_ETH_XLNX_GEM_PORT_2_DMA_FIXED
		struct eth_xlnx_dma_area_gem2 *dma_area_gem2 =
			(struct eth_xlnx_dma_area_gem2*)
				CONFIG_ETH_XLNX_GEM_PORT_2_DMA_BASE_ADDRESS;
		dev_data->rxbd_ring.first_bd = &(dma_area_gem2->rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem2->tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem2->rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem2->tx_buffer;
#else
		dev_data->rxbd_ring.first_bd = &(dma_area_gem2.rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem2.tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem2.rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem2.tx_buffer;
#endif /* CONFIG_ETH_XLNX_GEM_PORT_2_DMA_FIXED */
	}
#endif /* CONFIG_ETH_XLNX_GEM_PORT_2 */

#ifdef CONFIG_ETH_XLNX_GEM_PORT_3
	if (dev_conf->base_addr == DT_REG_ADDR(DT_NODELABEL(gem3))) {
#ifdef CONFIG_ETH_XLNX_GEM_PORT_3_DMA_FIXED
		struct eth_xlnx_dma_area_gem3 *dma_area_gem3 =
			(struct eth_xlnx_dma_area_gem3*)
				CONFIG_ETH_XLNX_GEM_PORT_3_DMA_BASE_ADDRESS;
		dev_data->rxbd_ring.first_bd = &(dma_area_gem3->rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem3->tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem3->rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem3->tx_buffer;
#else
		dev_data->rxbd_ring.first_bd = &(dma_area_gem3.rx_bd[0]);
		dev_data->txbd_ring.first_bd = &(dma_area_gem3.tx_bd[0]);
		dev_data->first_rx_buffer    = (uint8_t*)dma_area_gem3.rx_buffer;
		dev_data->first_tx_buffer    = (uint8_t*)dma_area_gem3.tx_buffer;
#endif /* CONFIG_ETH_XLNX_GEM_PORT_3_DMA_FIXED */
	}
#endif /* CONFIG_ETH_XLNX_GEM_PORT_3 */

	/* Set initial RX BD data -> comp. Zynq-7000 TRM, Chapter 16.3.5,
	 * "Receive Buffer Descriptor List". The BD ring data other than
	 * the base RX/TX buffer pointers will be set in eth_xlnx_gem_-
	 * iface_init() */
	bdptr = dev_data->rxbd_ring.first_bd;

	for (buf_iter = 0; buf_iter < (dev_conf->rxbd_count - 1); buf_iter++) {
		/* Clear 'used' bit -> BD is owned by the controller
		 * -> as per TRM */
		bdptr->ctrl = 0;
		bdptr->addr = (uint32_t)dev_data->first_rx_buffer +
			(buf_iter * (uint32_t)dev_conf->rx_buffer_size);
		++bdptr;
	}

	/* For the last BD, bit [1] must be OR'ed in the buffer memory
	 * address -> this is the 'wrap' bit indicating that this is the
	 * last BD in the ring. This location is used as bits [1..0] can't
	 * be part of the buffer address due to alignment requirements
	 * anyways. Watch out: TX BDs handle this differently, their wrap
	 * bit is located in the BD's control word! */
	bdptr->ctrl = 0; /* BD is owned by the controller */
	bdptr->addr = ((uint32_t)dev_data->first_rx_buffer +
		(buf_iter * (uint32_t)dev_conf->rx_buffer_size)) |
		ETH_XLNX_GEM_RXBD_WRAP_BIT;

	/* Set initial TX BD data -> comp. Zynq-7000 TRM, Chapter 16.3.5,
	 * "Transmit Buffer Descriptor List". TX BD ring data has already
	 * been set up in eth_xlnx_gem_iface_init() */
	bdptr = dev_data->txbd_ring.first_bd;

	for (buf_iter = 0; buf_iter < (dev_conf->txbd_count - 1); buf_iter++) {
		/* Set 'used' bit.
		 * This contradicts the TRM, according to which the bit
		 * should be cleared, indicating management by the controller.
		 * However, if the bit is cleared, each transmission completes
		 * with an error flag set in the TXSR.
		 * TODO: investigate further? */
		bdptr->ctrl = ETH_XLNX_GEM_TXBD_USED_BIT;
		bdptr->addr = (uint32_t)dev_data->first_tx_buffer +
			(buf_iter * (uint32_t)dev_conf->tx_buffer_size);
		++bdptr;
	}

	/* For the last BD, set the 'wrap' bit indicating to the controller
	 * that this BD is the last one in the ring. -> For TX BDs, the 'wrap'
	 * bit isn't located in the address word, but in the control word
	 * instead */
	bdptr->ctrl = (ETH_XLNX_GEM_TXBD_USED_BIT
		| ETH_XLNX_GEM_TXBD_WRAP_BIT);
	bdptr->addr = (uint32_t)dev_data->first_tx_buffer +
		(buf_iter * (uint32_t)dev_conf->tx_buffer_size);

	/* Set free count/current index in the RX/TX BD ring data */
	dev_data->rxbd_ring.next_to_process = 0;
	dev_data->rxbd_ring.next_to_use     = 0;
	dev_data->rxbd_ring.free_bds        = dev_conf->rxbd_count;
	dev_data->txbd_ring.next_to_process = 0;
	dev_data->txbd_ring.next_to_use     = 0;
	dev_data->txbd_ring.free_bds        = dev_conf->txbd_count;

	/* Write pointers to the first RX/TX BD to the controller */
	sys_write32((uint32_t)dev_data->rxbd_ring.first_bd,
		dev_conf->base_addr + ETH_XLNX_GEM_RXQBASE_OFFSET);
	sys_write32((uint32_t)dev_data->txbd_ring.first_bd,
		dev_conf->base_addr + ETH_XLNX_GEM_TXQBASE_OFFSET);
}

/**
 * @brief GEM RX data pending handler wrapper for the work queue
 * Wraps the RX data pending handler, eth_xlnx_gem_handle_rx_pending,
 * for the scenario in which the current GEM device is configured
 * to defer RX pending / TX done indication handling to the system
 * work queue. In this case, the work item received by this wrapper
 * function will be enqueued from within the ISR if the corresponding
 * bit is set within the controller's interrupt status register
 * (gem.intr_status).
 *
 * @param item Pointer to the work item enqueued by the ISR which
 *             facilitates access to the current device's data
 */
static void eth_xlnx_gem_rx_pending_work(struct k_work *item)
{
	struct eth_xlnx_gem_dev_data *dev_data = CONTAINER_OF(item,
		struct eth_xlnx_gem_dev_data, rx_pend_work);
	struct device *dev = net_if_get_device(dev_data->iface);
	
	eth_xlnx_gem_handle_rx_pending(dev);
}

/**
 * @brief GEM RX data pending handler
 * This handler is called either from within the ISR or from the
 * context of the system work queue whenever the RX data pending bit
 * is set in the controller's interrupt status register (gem.intr_status).
 * No further RX data pending interrupts will be triggered until this
 * handler has been executed, which eventually clears the corresponding
 * interrupt status bit. This function acquires the incoming packet
 * data from the DMA memory area via the RX buffer descriptors and copies
 * the data to a packet which will then be handed over to the network
 * stack.
 *
 * @param dev Pointer to the device configuration data
 */
static void eth_xlnx_gem_handle_rx_pending(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf         = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data         = DEV_DATA(dev);
	uint32_t			reg_val           = 0;
	uint32_t			reg_val_rxsr      = 0;
	uint8_t				first_bd_idx      = 0;
	uint8_t				last_bd_idx       = 0;
	uint8_t				curr_bd_idx       = 0;
	uint32_t			rx_data_length    = 0;
	uint32_t			rx_data_remaining = 0;
	struct net_buf			*pkt_buf          = NULL;
	struct net_pkt			*pkt              = NULL;
	uint32_t			frag_len          = 0;
	uint32_t			bd_remaining      = 0;
	uint32_t			bd_copied         = 0;
	uint32_t			src_buffer_offs   = 0;
	uint8_t				*data_dest        = NULL;

	/* Read the RX status register */
	reg_val_rxsr = sys_read32(
		dev_conf->base_addr + ETH_XLNX_GEM_RXSR_OFFSET);
	/* TODO Evaluate error flags from RX status register word
	 * here for proper error handling. */

	while (1) {
		curr_bd_idx  = dev_data->rxbd_ring.next_to_process;
		first_bd_idx = last_bd_idx = curr_bd_idx;

		/* Basic precondition checks for the current BD's
		 * address and control words */
		reg_val = sys_read32((uint32_t)
			(&dev_data->rxbd_ring.first_bd[first_bd_idx].addr));
		if ((reg_val & ETH_XLNX_GEM_RXBD_USED_BIT) == 0) {
			/* No new data contained in the current BD
			 * -> break out of the RX loop */
			break;
		}
		reg_val = sys_read32((uint32_t)
			(&dev_data->rxbd_ring.first_bd[first_bd_idx].ctrl));
		if ((reg_val & ETH_XLNX_GEM_RXBD_START_OF_FRAME_BIT) == 0) {
			/* Although the current BD is marked as 'used', it
			 * doesn't contain the SOF bit. */
			LOG_ERR("GEM@0x%08X unexpected missing "
				"SOF bit in RX BD [%u]",
				dev_conf->base_addr,
				first_bd_idx);
			break;
		}

		/* As long as the current BD doesn't have the EOF bit set,
		 * iterate forwards until the EOF bit is encountered. Only
		 * the BD containing the EOF bit also contains the length
		 * of the received packet which spans multiple buffers. */
		do {
			reg_val = sys_read32((uint32_t)
				(&dev_data->rxbd_ring.first_bd[last_bd_idx].ctrl));
			rx_data_length = rx_data_remaining =
				(reg_val & ETH_XLNX_GEM_RXBD_FRAME_LENGTH_MASK);
			if ((reg_val & ETH_XLNX_GEM_RXBD_END_OF_FRAME_BIT) == 0) {
				last_bd_idx = (last_bd_idx + 1) % dev_conf->rxbd_count;
			}
		} while ((reg_val & ETH_XLNX_GEM_RXBD_END_OF_FRAME_BIT) == 0);
		printk("\nRX: %u bytes in BDs [%u] to [%u]\n", rx_data_length,
			first_bd_idx, last_bd_idx);

		/* Allocate a destination packet from the network stack
		 * now that the total frame length is known. */
		pkt = net_pkt_rx_alloc_with_buffer(
			dev_data->iface,
			rx_data_length,
			AF_UNSPEC,
			0,
			K_NO_WAIT);
		if (pkt == NULL) {
			LOG_ERR("net_pkt_rx_alloc_with_buffer failed for %u bytes", rx_data_length);
			printk("ALLOC ERR\n");
		}
		pkt_buf = pkt->buffer;

		/* Copy data from all involved RX buffers to network
		 * stack's packet buffer. */
		do {
			src_buffer_offs = 0;
			bd_remaining =
				(rx_data_remaining <= dev_conf->rx_buffer_size)
				? rx_data_remaining : dev_conf->rx_buffer_size;
			printk("  RX remaining %u -> BD[%u] contains %u\n", rx_data_remaining, curr_bd_idx, bd_remaining);

			while (bd_remaining > 0) {
				data_dest = pkt_buf->data;
				frag_len  = net_buf_tailroom(pkt_buf);
				if (bd_remaining <= frag_len) {
					bd_copied = bd_remaining;
				} else {
					bd_copied = frag_len;
				}
				printk("    Frag tailroom %u, copying %u from BD offs [%u]\n", frag_len, bd_copied, src_buffer_offs);

				if (frag_len > 0) {
					memcpy((void*)data_dest,
						(void*)((dev_data->
							rxbd_ring.first_bd[curr_bd_idx].addr
							& ETH_XLNX_GEM_RXBD_BUFFER_ADDR_MASK)
							+ src_buffer_offs),
						bd_copied);
					net_buf_add(pkt_buf, bd_copied);

					bd_remaining    -= bd_copied;
					src_buffer_offs += bd_copied;
				}

				if (bd_remaining > 0) {
					pkt_buf = pkt_buf->frags;
				}
			}

			rx_data_remaining -=
				(rx_data_remaining <= dev_conf->rx_buffer_size)
				? rx_data_remaining : dev_conf->rx_buffer_size;

			/*The entire packet data of the current BD has been
			 * processed, on to the next BD -> preserve the RX BD's
			 * 'wrap' bit & address, but clear the 'used' bit. */
			reg_val	= sys_read32((uint32_t)
				(&dev_data->rxbd_ring.first_bd[curr_bd_idx].addr));
			reg_val &= ~ETH_XLNX_GEM_RXBD_USED_BIT;
			sys_write32(reg_val, (uint32_t)
				(&dev_data->rxbd_ring.first_bd[curr_bd_idx].addr));
			curr_bd_idx = (curr_bd_idx + 1) % dev_conf->rxbd_count;
		} while (curr_bd_idx !=
			((last_bd_idx + 1) % dev_conf->rxbd_count));

		/* Propagate the received packet to the network stack */
		if (net_recv_data(dev_data->iface, pkt) < 0) {
			LOG_ERR("GEM@0x%08X RX packet hand-over "
				"to IP stack failed",
				dev_conf->base_addr);
			net_pkt_unref(pkt);
		}
#ifdef CONFIG_NET_STATISTICS_ETHERNET
		dev_data->stats.bytes.received += rx_data_length;
		dev_data->stats.pkts.rx++;
#endif
		/* Store the position of the first BD behind the end of the
		 * frame that has just been processed as 'next to process' */
		dev_data->rxbd_ring.next_to_process = curr_bd_idx;
		printk("RX done, next first BD is [%u]\n", curr_bd_idx);
	}

	/* Clear the RX status register */
	sys_write32(0xFFFFFFFF, 
		dev_conf->base_addr + ETH_XLNX_GEM_RXSR_OFFSET);
}

/**
 * @brief GEM TX done handler wrapper for the work queue
 * Wraps the TX done handler, eth_xlnx_gem_handle_tx_done,
 * for the scenario in which the current GEM device is configured
 * to defer RX pending / TX done indication handling to the system
 * work queue. In this case, the work item received by this wrapper
 * function will be enqueued from within the ISR if the corresponding
 * bit is set within the controller's interrupt status register
 * (gem.intr_status).
 *
 * @param item Pointer to the work item enqueued by the ISR which
 *             facilitates access to the current device's data
 */
static void eth_xlnx_gem_tx_done_work(struct k_work *item)
{
	struct eth_xlnx_gem_dev_data *dev_data = CONTAINER_OF(item,
		struct eth_xlnx_gem_dev_data, tx_done_work);
	struct device *dev = net_if_get_device(dev_data->iface);
	
	eth_xlnx_gem_handle_tx_done(dev);
}

/**
 * @brief GEM TX done handler
 * This handler is called either from within the ISR or from the
 * context of the system work queue whenever the TX done bit is set
 * in the controller's interrupt status register (gem.intr_status).
 * No further TX done interrupts will be triggered until this handler
 * has been executed, which eventually clears the corresponding
 * interrupt status bit. Once this handler reaches the end of its
 * execution, the eth_xlnx_gem_send call which effectively triggered
 * it is unblocked by posting to the current GEM's TX done semaphore
 * on which the send function is blocking.
 *
 * @param dev Pointer to the device configuration data
 */
static void eth_xlnx_gem_handle_tx_done(struct device *dev)
{
	struct eth_xlnx_gem_dev_cfg	*dev_conf     = DEV_CFG(dev);
	struct eth_xlnx_gem_dev_data	*dev_data     = DEV_DATA(dev);
	uint32_t			reg_val       = 0;
	uint32_t			reg_val_txsr  = 0;
	uint8_t				curr_bd_idx   = 0;
	uint8_t				bds_processed = 0;
	uint8_t				bd_is_last    = 0;

	/* Read the TX status register */
	reg_val_txsr = sys_read32(
		dev_conf->base_addr + ETH_XLNX_GEM_TXSR_OFFSET);

	/* TODO Evaluate error flags from TX status register word
	 * here for proper error handling */
	curr_bd_idx = dev_data->txbd_ring.next_to_process;
	reg_val = sys_read32(
		(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));

	do {
		++bds_processed;

		/* TODO Evaluate error flags from current BD control word
		 * here for proper error handling */

		/* Check if the BD we're currently looking at is the last BD
		 * of the current transmission */
		bd_is_last =
			((reg_val & ETH_XLNX_GEM_TXBD_LAST_BIT) != 0) ? 1 : 0;

		/* Reset control word of the current BD */
		reg_val &= ETH_XLNX_GEM_TXBD_WRAP_BIT;
		reg_val |= ETH_XLNX_GEM_TXBD_USED_BIT;
		
		sys_write32(reg_val,
			(uint32_t)(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));

		/* Move on to the next BD or break out of the loop */
		if (bd_is_last == 1) {
			break;
		} else {
			curr_bd_idx = (curr_bd_idx + 1) % dev_conf->txbd_count;
			reg_val = sys_read32((uint32_t)
				(&dev_data->txbd_ring.first_bd[curr_bd_idx].ctrl));
		}
	} while (bd_is_last == 0);

	dev_data->txbd_ring.next_to_process =
			(dev_data->txbd_ring.next_to_process + bds_processed)
			% dev_conf->txbd_count;
	dev_data->txbd_ring.free_bds += bds_processed;

	/* Clear the TX status register */
	sys_write32(0xFFFFFFFF,
		dev_conf->base_addr + ETH_XLNX_GEM_TXSR_OFFSET);

	/* Indicate completion to a blocking eth_xlnx_gem_send() call */
	k_sem_give(&dev_data->tx_done_sem);
}

/* EOF */