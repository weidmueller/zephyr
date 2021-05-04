/*
 * Xilinx Processor System Gigabit Ethernet controller (GEM) driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 * 
 * Driver private data declarations
 * 
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_PRIV_H_
#define _ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_PRIV_H_

#include <kernel.h>
#include <zephyr/types.h>
#include <net/net_pkt.h>

#include "phy_xlnx_gem.h"

#define ETH_XLNX_BUFFER_ALIGNMENT			4 /* RX/TX buffer alignment (in bytes) */

/* Buffer descriptor (BD) related defines */
/* RX BD bits & masks: comp. Zynq-7000 TRM, Table 16-2. */
#define ETH_XLNX_GEM_RXBD_WRAP_BIT			0x00000002 /* Address word: Wrap bit, last BD */
#define ETH_XLNX_GEM_RXBD_USED_BIT			0x00000001 /* Address word: BD used bit */
#define ETH_XLNX_GEM_RXBD_BUFFER_ADDR_MASK		0xFFFFFFFC /* Address word: Mask for effective buffer address -> excludes [1..0] */
#define ETH_XLNX_GEM_RXBD_BCAST_BIT			0x80000000 /* Control word: Broadcast detected */
#define ETH_XLNX_GEM_RXBD_MCAST_HASH_MATCH_BIT		0x40000000 /* Control word: Multicast hash match detected */
#define ETH_XLNX_GEM_RXBD_UCAST_HASH_MATCH_BIT		0x20000000 /* Control word: Unicast hash match detected */
#define ETH_XLNX_GEM_RXBD_SPEC_ADDR_MATCH_BIT		0x08000000 /* Control word: Specific address match detected */
#define ETH_XLNX_GEM_RXBD_SPEC_ADDR_MASK		0x00000003 /* Control word: Bits indicating which specific address register was matched */
#define ETH_XLNX_GEM_RXBD_SPEC_ADDR_SHIFT		25         /* Control word: Shift for specific address register ID bits */
#define ETH_XLNX_GEM_RXBD_BIT24				0x01000000 /* Control word: Bit [24] - this bit has different semantics depending on whether RX checksum offloading is enabled or not */
#define ETH_XLNX_GEM_RXBD_BITS23_22_MASK		0x00000003 /* Control word: Bits [23..22] - these bits have different semantics depending on whether RX checksum offloading is enabled or not */
#define ETH_XLNX_GEM_RXBD_BITS23_22_SHIFT		22         /* Control word: Shift for multi-purpose bits [23..22] */
#define ETH_XLNX_GEM_RXBD_VLAN_TAG_DETECTED_BIT		0x00200000 /* Control word: VLAN tag (type ID 0x8100) detected */
#define ETH_XLNX_GEM_RXBD_PRIO_TAG_DETECTED_BIT		0x00100000 /* Control word: Priority tag: VLAN tag (type ID 0x8100) and null VLAN identifier detected */
#define ETH_XLNX_GEM_RXBD_VLAN_PRIORITY_MASK		0x00000007 /* Control word: Bits [19..17] contain the VLAN priority */
#define ETH_XLNX_GEM_RXBD_VLAN_PRIORITY_SHIFT		17         /* Control word: Shift for VLAN priority bits [19..17] */
#define ETH_XLNX_GEM_RXBD_CFI_BIT			0x00010000 /* Control word: Canonical format indicator bit */
#define ETH_XLNX_GEM_RXBD_END_OF_FRAME_BIT		0x00008000 /* Control word: End-of-frame bit */
#define ETH_XLNX_GEM_RXBD_START_OF_FRAME_BIT		0x00004000 /* Control word: Start-of-frame bit */
#define ETH_XLNX_GEM_RXBD_FCS_STATUS_BIT		0x00002000 /* Control word: FCS status bit for FCS ignore mode */
#define ETH_XLNX_GEM_RXBD_FRAME_LENGTH_MASK		0x00001FFF /* Control word: mask for data length of received frame */

/* RX BD bits & masks: comp. Zynq-7000 TRM, Table 16-3. */
#define ETH_XLNX_GEM_TXBD_USED_BIT			0x80000000 /* Control word: BD used marker */
#define ETH_XLNX_GEM_TXBD_WRAP_BIT			0x40000000 /* Control word: Wrap bit, last BD */
#define ETH_XLNX_GEM_TXBD_RETRY_BIT			0x20000000 /* Control word: Retry limit exceeded */
#define ETH_XLNX_GEM_TXBD_URUN_BIT			0x10000000 /* Control word: Transmit underrun occurred */
#define ETH_XLNX_GEM_TXBD_EXH_BIT			0x08000000 /* Control word: Buffers exhausted */
#define ETH_XLNX_GEM_TXBD_LAC_BIT			0x04000000 /* Control word: Late collision */
#define ETH_XLNX_GEM_TXBD_NOCRC_BIT			0x00010000 /* Control word: No CRC */
#define ETH_XLNX_GEM_TXBD_LAST_BIT			0x00008000 /* Control word: Last buffer */
#define ETH_XLNX_GEM_TXBD_LEN_MASK			0x00003FFF /* Control word: Mask for length field */
#define ETH_XLNX_GEM_TXBD_ERR_MASK			0x3C000000 /* Control word: Mask for error field */

/* TX clock configuration: Zynq-7000 */
#ifdef CONFIG_SOC_XILINX_ZYNQ7000
#define ETH_XLNX_SLCR_LOCK_REGISTER_ADDRESS		0xF8000004
#define ETH_XLNX_SLCR_UNLOCK_REGISTER_ADDRESS		0xF8000008
#define ETH_XLNX_SLCR_LOCK_KEY				0x767B
#define ETH_XLNX_SLCR_UNLOCK_KEY			0xDF0D
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK	0x0000003F
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT	20
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT	8
#endif /* CONFIG_SOC_XILINX_ZYNQ7000 */

/* TX clock configuration: UltraScale */
#ifdef CONFIG_SOC_XILINX_ZYNQMP
#define ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS		0xFF5E001C
#define ETH_XLNX_CRL_APB_WPROT_BIT			0x00000001
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK	0x0000003F
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT	16
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT	8
#endif /* CONFIG_SOC_XILINX_ZYNQMP */

/* Register offsets within the respective GEM's address space */
#define ETH_XLNX_GEM_NWCTRL_OFFSET			0x00000000 /* gem.net_ctrl       Network Control           register */
#define ETH_XLNX_GEM_NWCFG_OFFSET			0x00000004 /* gem.net_cfg        Network Configuration     register */
#define ETH_XLNX_GEM_NWSR_OFFSET			0x00000008 /* gem.net_status     Network Status            register */
#define ETH_XLNX_GEM_DMACR_OFFSET			0x00000010 /* gem.dma_cfg        DMA Control               register */
#define ETH_XLNX_GEM_TXSR_OFFSET			0x00000014 /* gem.tx_status      TX Status                 register */
#define ETH_XLNX_GEM_RXQBASE_OFFSET			0x00000018 /* gem.rx_qbar        RXQ base address          register */
#define ETH_XLNX_GEM_TXQBASE_OFFSET			0x0000001C /* gem.tx_qbar        TXQ base address          register */
#define ETH_XLNX_GEM_RXSR_OFFSET			0x00000020 /* gem.rx_status      RX Status                 register */
#define ETH_XLNX_GEM_ISR_OFFSET				0x00000024 /* gem.intr_status    Interrupt status          register */
#define ETH_XLNX_GEM_IER_OFFSET				0x00000028 /* gem.intr_en        Interrupt enable          register */
#define ETH_XLNX_GEM_IDR_OFFSET				0x0000002C /* gem.intr_dis       Interrupt disable         register */
#define ETH_XLNX_GEM_IMR_OFFSET				0x00000030 /* gem.intr_mask      Interrupt mask            register */
#define ETH_XLNX_GEM_PHY_MAINTENANCE_OFFSET		0x00000034 /* gem.phy_maint      PHY maintenance           register */
#define ETH_XLNX_GEM_LADDR1L_OFFSET			0x00000088 /* gem.spec_addr1_bot Specific address 1 bottom register */
#define ETH_XLNX_GEM_LADDR1H_OFFSET			0x0000008C /* gem.spec_addr1_top Specific address 1 top    register */
#define ETH_XLNX_GEM_LADDR2L_OFFSET			0x00000090 /* gem.spec_addr2_bot Specific address 2 bottom register */
#define ETH_XLNX_GEM_LADDR2H_OFFSET			0x00000094 /* gem.spec_addr2_top Specific address 2 top    register */
#define ETH_XLNX_GEM_LADDR3L_OFFSET			0x00000098 /* gem.spec_addr3_bot Specific address 3 bottom register */
#define ETH_XLNX_GEM_LADDR3H_OFFSET			0x0000009C /* gem.spec_addr3_top Specific address 3 top    register */
#define ETH_XLNX_GEM_LADDR4L_OFFSET			0x000000A0 /* gem.spec_addr4_bot Specific address 4 bottom register */
#define ETH_XLNX_GEM_LADDR4H_OFFSET			0x000000A4 /* gem.spec_addr4_top Specific address 4 top    register */

/* Masks for clearing registers during initialization */
#define ETH_XLNX_GEM_STATCLR_MASK			0x00000020 /* gem.net_ctrl  [clear_stat_regs] */
#define ETH_XLNX_GEM_TXSRCLR_MASK			0x000000FF /* gem.tx_status [7..0]            */
#define ETH_XLNX_GEM_RXSRCLR_MASK			0x0000000F /* gem.tx_status [3..0]            */
#define ETH_XLNX_GEM_IDRCLR_MASK			0x07FFFFFF /* gem.intr_dis  [26..0]           */

/* (Shift) masks for individual registers' fields */
#define ETH_XLNX_GEM_NWCTRL_RXTSTAMP_BIT		0x00008000 /* gem.net_ctrl: RX Timestamp in CRC */
#define ETH_XLNX_GEM_NWCTRL_ZEROPAUSETX_BIT		0x00001000 /* gem.net_ctrl: Transmit zero quantum pause frame */
#define ETH_XLNX_GEM_NWCTRL_PAUSETX_BIT			0x00000800 /* gem.net_ctrl: Transmit pause frame */
#define ETH_XLNX_GEM_NWCTRL_HALTTX_BIT			0x00000400 /* gem.net_ctrl: Halt transmission after current frame */
#define ETH_XLNX_GEM_NWCTRL_STARTTX_BIT			0x00000200 /* gem.net_ctrl: Start transmission (tx_go) */
#define ETH_XLNX_GEM_NWCTRL_STATWEN_BIT			0x00000080 /* gem.net_ctrl: Enable writing to stat counters */
#define ETH_XLNX_GEM_NWCTRL_STATINC_BIT			0x00000040 /* gem.net_ctrl: Increment statistic registers */
#define ETH_XLNX_GEM_NWCTRL_STATCLR_BIT			0x00000020 /* gem.net_ctrl: Clear statistic registers */
#define ETH_XLNX_GEM_NWCTRL_MDEN_BIT			0x00000010 /* gem.net_ctrl: Enable MDIO port */
#define ETH_XLNX_GEM_NWCTRL_TXEN_BIT			0x00000008 /* gem.net_ctrl: Enable transmit */
#define ETH_XLNX_GEM_NWCTRL_RXEN_BIT			0x00000004 /* gem.net_ctrl: Enable receive */
#define ETH_XLNX_GEM_NWCTRL_LOOPEN_BIT			0x00000002 /* gem.net_ctrl: local loopback */

#define ETH_XLNX_GEM_NWCFG_IGNIPGRXERR_BIT		0x40000000 /* gem.net_cfg: Ignore IPG RX Error */
#define ETH_XLNX_GEM_NWCFG_BADPREAMBEN_BIT		0x20000000 /* gem.net_cfg: Disable rejection of non-standard preamble */
#define ETH_XLNX_GEM_NWCFG_IPDSTRETCH_BIT		0x10000000 /* gem.net_cfg: Enable transmit IPG */
#define ETH_XLNX_GEM_NWCFG_SGMIIEN_BIT			0x08000000 /* gem.net_cfg: Enable SGMII mode */
#define ETH_XLNX_GEM_NWCFG_FCSIGNORE_BIT		0x04000000 /* gem.net_cfg: Disable rejection of FCS error */
#define ETH_XLNX_GEM_NWCFG_HDRXEN_BIT			0x02000000 /* gem.net_cfg: RX half duplex */
#define ETH_XLNX_GEM_NWCFG_RXCHKSUMEN_BIT		0x01000000 /* gem.net_cfg: Enable RX checksum offload */
#define ETH_XLNX_GEM_NWCFG_PAUSECOPYDI_BIT		0x00800000 /* gem.net_cfg: Do not copy pause Frames to memory */
#define ETH_XLNX_GEM_NWCFG_DBUSW_MASK			0x3        /* gem.net_cfg: Mask for data bus width */
#define ETH_XLNX_GEM_NWCFG_DBUSW_SHIFT			21         /* gem.net_cfg: Shift for data bus width */
#define ETH_XLNX_GEM_NWCFG_MDC_MASK			0x7        /* gem.net_cfg: Mask for MDC clock divider */
#define ETH_XLNX_GEM_NWCFG_MDC_SHIFT			18         /* gem.net_cfg: Shift for MDC clock divider */
#define ETH_XLNX_GEM_NWCFG_MDCCLKDIV_MASK		0x001C0000 /* gem.net_cfg: MDC Mask PCLK divider */
#define ETH_XLNX_GEM_NWCFG_FCSREM_BIT			0x00020000 /* gem.net_cfg: Discard FCS from received frames */
#define ETH_XLNX_GEM_NWCFG_LENGTHERRDSCRD_BIT		0x00010000 /* gem.net_cfg: RX length error discard */
#define ETH_XLNX_GEM_NWCFG_RXOFFS_MASK			0x00000003 /* gem.net_cfg: Mask for RX buffer offset */
#define ETH_XLNX_GEM_NWCFG_RXOFFS_SHIFT			14         /* gem.net_cfg: Shift for RX buffer offset*/
#define ETH_XLNX_GEM_NWCFG_PAUSEEN_BIT			0x00002000 /* gem.net_cfg: Enable pause TX */
#define ETH_XLNX_GEM_NWCFG_RETRYTESTEN_BIT		0x00001000 /* gem.net_cfg: Retry test */
#define ETH_XLNX_GEM_NWCFG_TBIINSTEAD_BIT		0x00000800 /* gem.net_cfg: Use TBI instead of the GMII/MII interface */
#define ETH_XLNX_GEM_NWCFG_1000_BIT			0x00000400 /* gem.net_cfg: Gigabit mode */
#define ETH_XLNX_GEM_NWCFG_EXTADDRMATCHEN_BIT		0x00000200 /* gem.net_cfg: External address match enable */
#define ETH_XLNX_GEM_NWCFG_1536RXEN_BIT			0x00000100 /* gem.net_cfg: Enable 1536 byte frames reception */
#define ETH_XLNX_GEM_NWCFG_UCASTHASHEN_BIT		0x00000080 /* gem.net_cfg: Receive unicast hash frames */
#define ETH_XLNX_GEM_NWCFG_MCASTHASHEN_BIT		0x00000040 /* gem.net_cfg: Receive multicast hash frames */
#define ETH_XLNX_GEM_NWCFG_BCASTDIS_BIT			0x00000020 /* gem.net_cfg: Do not receive broadcast frames */
#define ETH_XLNX_GEM_NWCFG_COPYALLEN_BIT		0x00000010 /* gem.net_cfg: Copy all frames = promiscuous mode */
#define ETH_XLNX_GEM_NWCFG_NVLANDISC_BIT		0x00000004 /* gem.net_cfg: Receive only VLAN frames */
#define ETH_XLNX_GEM_NWCFG_FDEN_BIT			0x00000002 /* gem.net_cfg: Full duplex */
#define ETH_XLNX_GEM_NWCFG_100_BIT			0x00000001 /* gem.net_cfg: 10 or 100 Mbs */

#define ETH_XLNX_GEM_DMACR_DISCNOAHB_BIT		0x01000000 /* gem.dma_cfg: Discard packets when AHB resource is unavailable */
#define ETH_XLNX_GEM_DMACR_RX_BUF_MASK			0x000000FF /* gem.dma_cfg: Mask for RX buffer size */
#define ETH_XLNX_GEM_DMACR_RX_BUF_SHIFT			16         /* gem.dma_cfg: Shift count for RX buffer size */
#define ETH_XLNX_GEM_DMACR_TCP_CHKSUM_BIT		0x00000800 /* gem.dma_cfg: Enable/disable TCP|UDP/IP TX checksum offload */
#define ETH_XLNX_GEM_DMACR_TX_SIZE_BIT			0x00000400 /* gem.dma_cfg: TX buffer half/full memory size */
#define ETH_XLNX_GEM_DMACR_RX_SIZE_MASK			0x00000300 /* gem.dma_cfg: Mask for RX buffer memory size */
#define ETH_XLNX_GEM_DMACR_RX_SIZE_SHIFT		8          /* gem.dma_cfg: Shift for for RX buffer memory size */
#define ETH_XLNX_GEM_DMACR_ENDIAN_BIT			0x00000080 /* gem.dma_cfg: Endianess configuration */
#define ETH_XLNX_GEM_DMACR_DESCR_ENDIAN_BIT		0x00000040 /* gem.dma_cfg: Descriptor access endianess configuration */
#define ETH_XLNX_GEM_DMACR_AHB_BURST_LENGTH_MASK	0x0000001F /* gem.dma_cfg: AHB burst length */

#define ETH_XLNX_GEM_IXR_PTPPSTX_BIT			0x02000000 /* gem.intr_*: PTP Psync transmitted */
#define ETH_XLNX_GEM_IXR_PTPPDRTX_BIT			0x01000000 /* gem.intr_*: PTP Pdelay_req transmitted */
#define ETH_XLNX_GEM_IXR_PTPSTX_BIT			0x00800000 /* gem.intr_*: PTP Sync transmitted */
#define ETH_XLNX_GEM_IXR_PTPDRTX_BIT			0x00400000 /* gem.intr_*: PTP Delay_req transmitted */
#define ETH_XLNX_GEM_IXR_PTPPSRX_BIT			0x00200000 /* gem.intr_*: PTP Psync received */
#define ETH_XLNX_GEM_IXR_PTPPDRRX_BIT			0x00100000 /* gem.intr_*: PTP Pdelay_req received */
#define ETH_XLNX_GEM_IXR_PTPSRX_BIT			0x00080000 /* gem.intr_*: PTP Sync received */
#define ETH_XLNX_GEM_IXR_PTPDRRX_BIT			0x00040000 /* gem.intr_*: PTP Delay_req received */
#define ETH_XLNX_GEM_IXR_PARTNER_PGRX_BIT		0x00020000 /* gem.intr_*: partner_pg_rx */
#define ETH_XLNX_GEM_IXR_AUTONEG_COMPLETE_BIT		0x00010000 /* gem.intr_*: Auto-negotiation completed */
#define ETH_XLNX_GEM_IXR_EXTERNAL_INT_BIT		0x00008000 /* gem.intr_*: External interrupt signal */
#define ETH_XLNX_GEM_IXR_PAUSETX_BIT			0x00004000 /* gem.intr_*: Pause frame transmitted */
#define ETH_XLNX_GEM_IXR_PAUSEZERO_BIT			0x00002000 /* gem.intr_*: Pause time has reached zero */
#define ETH_XLNX_GEM_IXR_PAUSENZERO_BIT			0x00001000 /* gem.intr_*: Pause frame received */
#define ETH_XLNX_GEM_IXR_HRESPNOK_BIT			0x00000800 /* gem.intr_*: hresp not ok */
#define ETH_XLNX_GEM_IXR_RXOVR_BIT			0x00000400 /* gem.intr_*: Receive overrun occurred */
#define ETH_XLNX_GEM_IXR_TXCOMPL_BIT			0x00000080 /* gem.intr_*: Frame transmitted ok */
#define ETH_XLNX_GEM_IXR_TXEXH_BIT			0x00000040 /* gem.intr_*: Transmit err occurred or no buffers*/
#define ETH_XLNX_GEM_IXR_RETRY_BIT			0x00000020 /* gem.intr_*: Retry limit exceeded */
#define ETH_XLNX_GEM_IXR_URUN_BIT			0x00000010 /* gem.intr_*: Transmit underrun */
#define ETH_XLNX_GEM_IXR_TXUSED_BIT			0x00000008 /* gem.intr_*: Tx buffer used bit read */
#define ETH_XLNX_GEM_IXR_RXUSED_BIT			0x00000004 /* gem.intr_*: Rx buffer used bit read */
#define ETH_XLNX_GEM_IXR_FRAMERX_BIT			0x00000002 /* gem.intr_*: Frame received ok */
#define ETH_XLNX_GEM_IXR_MGMNT_BIT			0x00000001 /* gem.intr_*: PHY management complete */
#define ETH_XLNX_GEM_IXR_ALL_MASK			0x03FC7FFE /* gem.intr_*: Bit mask for all handled interrupt sources */

/* Bits / bit masks relating to the GEM's MDIO interface */
#define ETH_XLNX_GEM_MDIO_IDLE_BIT			0x00000004 /* gem.net_status: PHY management idle bit */
#define ETH_XLNX_GEM_MDIO_IN_STATUS_BIT			0x00000002 /* gem.net_status: MDIO input status */

#define ETH_XLNX_GEM_PHY_MAINT_CONST_BITS		0x40020000 /* gem.phy_maint: Bits constant for every operation: [31:30], [17:16] */
#define ETH_XLNX_GEM_PHY_MAINT_READ_OP_BIT		0x20000000 /* gem.phy_maint: Read operation control bit */
#define ETH_XLNX_GEM_PHY_MAINT_WRITE_OP_BIT		0x10000000 /* gem.phy_maint: Write operation control bit */
#define ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_MASK		0x0000001F /* gem.phy_maint: PHY address bits mask */
#define ETH_XLNX_GEM_PHY_MAINT_PHY_ADDRESS_SHIFT	23         /* gem.phy_maint: Shift for PHY address bits */
#define ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_MASK		0x0000001F /* gem.phy_maint: PHY register bits mask */
#define ETH_XLNX_GEM_PHY_MAINT_REGISTER_ID_SHIFT	18         /* gem.phy_maint: Shift for PHY register bits */
#define ETH_XLNX_GEM_PHY_MAINT_DATA_MASK		0x0000FFFF /* gem.phy_maint: 16-bit data word */

/* AMBA clock configuration related defines */
#define ETH_XLNX_GEM_AMBA_CLK_ENABLE_BIT_GEM0		(1 << 6)
#define ETH_XLNX_GEM_AMBA_CLK_ENABLE_BIT_GEM1		(1 << 7)

/* Auxiliary thread trigger bits */
#define ETH_XLNX_GEM_AUX_THREAD_RXDONE_BIT		(1 << 0)
#define ETH_XLNX_GEM_AUX_THREAD_TXDONE_BIT		(1 << 1)
#define ETH_XLNX_GEM_AUX_THREAD_POLL_PHY_BIT		(1 << 7)

/* Device configuration / run-time data resolver macros */
#define DEV_CFG(dev) \
	((struct eth_xlnx_gem_dev_cfg *)(dev)->config_info)
#define DEV_DATA(dev) \
	((struct eth_xlnx_gem_dev_data *)(dev)->driver_data)

/* Device tree / Kconfig data availability checks for all enabled
 * device instances */
#if defined(CONFIG_ETH_XLNX_GEM_PORT_0) && \
	!DT_NODE_HAS_STATUS(DT_NODELABEL(gem0), okay)
#error Data missing for GEM0: device tree configuration data is unavailable!
#endif

#if !defined(CONFIG_ETH_XLNX_GEM_PORT_0) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(gem0), okay)
#error GEM0 is marked active in the current device tree, but is not \
	activated in Kconfig!
#endif

#if defined(CONFIG_ETH_XLNX_GEM_PORT_1)	&& \
	!DT_NODE_HAS_STATUS(DT_NODELABEL(gem1), okay)
#error Data missing for GEM1: device tree configuration data is unavailable!
#endif

#if !defined(CONFIG_ETH_XLNX_GEM_PORT_1) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(gem1), okay)
#error GEM1 is marked active in the current device tree, but is not \
	activated in Kconfig!
#endif

#if defined(CONFIG_ETH_XLNX_GEM_PORT_2) && \
	!DT_NODE_HAS_STATUS(DT_NODELABEL(gem2), okay)
#error Data missing for GEM2: device tree configuration data is unavailable!
#endif

#if !defined(CONFIG_ETH_XLNX_GEM_PORT_2) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(gem2), okay)
#error GEM2 is marked active in the current device tree, but is not \
	activated in Kconfig!
#endif

#if defined(CONFIG_ETH_XLNX_GEM_PORT_3) && \
	!DT_NODE_HAS_STATUS(DT_NODELABEL(gem3), okay)
#error Data missing for GEM3: device tree configuration data is unavailable!
#endif

#if !defined(CONFIG_ETH_XLNX_GEM_PORT_3) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(gem3), okay)
#error GEM3 is marked active in the current device tree, but is not \
	activated in Kconfig!
#endif

/* Macro for attaching to a GEM device's IRQ */
#define ETH_XLNX_GEM_CONFIG_IRQ(port) \
if (dev_conf->base_addr == DT_REG_ADDR(DT_NODELABEL(gem##port))) { \
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gem##port)), \
		DT_IRQ(DT_NODELABEL(gem##port), priority), \
		eth_xlnx_gem_isr, DEVICE_GET(eth_xlnx_gem##port), 0);\
	irq_enable(DT_IRQN(DT_NODELABEL(gem##port))); \
}

/* IRQ handler function type */
typedef void (*eth_xlnx_gem_config_irq_t)(struct device *dev);

/* Enums for bitfields representing configuration settings */

/**
 * @brief Link speed configuration enumeration type.
 *
 * Enumeration type for link speed indication, contains 'link down'
 * plus all link speeds supported by the controller (10/100/1000).
 */
enum eth_xlnx_link_speed
{
	/* The values of this enum are consecutively numbered */
	LINK_DOWN = 0,
	LINK_10MBIT,
	LINK_100MBIT,
	LINK_1GBIT
};

/**
 * @brief AMBA AHB data bus width configuration enumeration type.
 *
 * Enumeration type containing the supported width options for the
 * AMBA AHB data bus. This is a configuration item in the controller's
 * net_cfg register.
 */
enum eth_xlnx_amba_dbus_width
{
	/* The values of this enum are consecutively numbered */
	AMBA_AHB_DBUS_WIDTH_32BIT = 0,
	AMBA_AHB_DBUS_WIDTH_64BIT,
	AMBA_AHB_DBUS_WIDTH_128BIT
};

/**
 * @brief MDC clock divider configuration enumeration type.
 *
 * Enumeration type containing the supported clock divider values
 * used to generate the MDIO interface clock (MDC) from either the
 * cpu_1x clock (Zynq-7000) or the LPD LSBUS clock (UltraScale).
 * This is a configuration item in the controller's net_cfg register.
 */
enum eth_xlnx_mdc_clock_divider
{
	/* The values of this enum are consecutively numbered */
	MDC_DIVIDER_8 = 0,
	MDC_DIVIDER_16,
	MDC_DIVIDER_32,
	MDC_DIVIDER_48,
	MDC_DIVIDER_64,
	MDC_DIVIDER_96,
	MDC_DIVIDER_128,
	MDC_DIVIDER_224
};

/**
 * @brief DMA RX buffer size configuration enumeration type.
 *
 * Enumeration type containing the supported size options for the
 * DMA receive buffer size in AHB system memory. This is a configuration
 * item in the controller's dma_cfg register.
 */
enum eth_xlnx_hwrx_buffer_size
{
	/* The values of this enum are consecutively numbered */
	HWRX_BUFFER_SIZE_1KB = 0,
	HWRX_BUFFER_SIZE_2KB,
	HWRX_BUFFER_SIZE_4KB,
	HWRX_BUFFER_SIZE_8KB
};

/**
 * @brief AHB burst length configuration enumeration type.
 *
 * Enumeration type containing the supported burst length options
 * for the AHB fixed burst length for DMA data operations. This is a
 * configuration item in the controller's dma_cfg register.
 */
enum eth_xlnx_ahb_burst_length
{
	/* The values of this enum are one-hot encoded */
	AHB_BURST_SINGLE = 1,
	/* 2 = also AHB_BURST_SINGLE */
	AHB_BURST_INCR4  = 4,
	AHB_BURST_INCR8  = 8,
	AHB_BURST_INCR16 = 16
};

/**
 * @brief DMA memory area buffer descriptor.
 *
 * An array of these descriptors for each RX and TX is used to
 * describe the respective DMA memory area. Each address word
 * points to the start of a RX or TX buffer within the DMA memory
 * area, while the control word is used for buffer status exchange
 * with the controller.
 */
struct eth_xlnx_gem_bd
{
	/* TODO for Cortex-A53: 64-bit addressing
	 * TODO: timestamping support */
	/* Buffer physical address (absolute address) */
	uint32_t		addr;
	/* Buffer control word (different contents for RX and TX) */
	uint32_t		ctrl;
};

/**
 * @brief DMA memory area buffer descriptor ring management structure.
 *
 * The DMA memory area buffer descriptor ring management structure
 * is used to manage either the RX or TX buffer descriptor array
 * (while the buffer descriptors are just an array from the software
 * point of view, the controller treats them as a ring, in which the
 * last descriptor's control word has a special last-in-ring bit set).
 * It contains a pointer to the start of the descriptor array, a
 * semaphore as a means of preventing concurrent access, a free entry
 * counter as well as indices used to determine which BD shall be used
 * or evaluated for the next RX/TX operation.
 */
struct eth_xlnx_gem_bdring
{
	/* Concurrent modification protection */
	struct k_sem		ring_sem;
	/* Pointer to the first BD in the list */
	struct eth_xlnx_gem_bd	*first_bd;
	/* Index of the next BD to be used for TX */
	uint8_t			next_to_use;
	/* Index of the next BD to be processed (both RX/TX) */
	uint8_t			next_to_process;
	/* Number of currently available BDs in this ring */
	uint8_t			free_bds;
};

/**
 * @brief Constant device configuration data structure.
 *
 * This struct contains all device configuration data for a GEM
 * controller instance which is constant. The data herein is
 * either acquired from the generated header file based on the
 * data from Kconfig, or from header file based on the device tree
 * data. Some of the data contained, in particular data relating
 * to clock sources, is specific to either the Zynq-7000 or the
 * UltraScale SoCs, which both contain the GEM.
 */
struct eth_xlnx_gem_dev_cfg {
	uint32_t			base_addr;
	eth_xlnx_gem_config_irq_t	config_func;

	uint32_t			pll_clock_frequency;
	uint32_t			clk_ctrl_reg_address;
	enum eth_xlnx_mdc_clock_divider	mdc_divider;

	enum eth_xlnx_link_speed	max_link_speed;
	uint8_t				init_phy;
	uint8_t				phy_mdio_addr_fix;
	uint8_t				phy_advertise_lower;
	uint32_t			phy_poll_interval;
	uint8_t				defer_rxp_to_queue;
	uint8_t				defer_txd_to_queue;

	enum eth_xlnx_amba_dbus_width	amba_dbus_width;
	enum eth_xlnx_ahb_burst_length	ahb_burst_length;
	enum eth_xlnx_hwrx_buffer_size	hw_rx_buffer_size;
	uint8_t				hw_rx_buffer_offset;

	uint8_t				rxbd_count;
	uint8_t				txbd_count;
	uint16_t			rx_buffer_size;
	uint16_t			tx_buffer_size;

	uint8_t				ignore_igp_rxer;
	uint8_t				disable_reject_nsp;
	uint8_t				enable_igp_stretch;
	uint8_t				enable_sgmii_mode;
	uint8_t				disable_reject_fcs_crc_errors;
	uint8_t				enable_rx_halfdup_while_tx;
	uint8_t				enable_rx_chksum_offload;
	uint8_t				disable_pause_copy;
	uint8_t				discard_rx_fcs;
	uint8_t				discard_rx_length_errors;
	uint8_t				enable_pause;
	uint8_t				enable_tbi;
	uint8_t				ext_addr_match;
	uint8_t				enable_1536_frames;
	uint8_t				enable_ucast_hash;
	uint8_t				enable_mcast_hash;
	uint8_t				disable_bcast;
	uint8_t				copy_all_frames;
	uint8_t				discard_non_vlan;
	uint8_t				enable_fdx;
	uint8_t				disc_rx_ahb_unavail;
	uint8_t				enable_tx_chksum_offload;
	uint8_t				tx_buffer_size_full;
	uint8_t				enable_ahb_packet_endian_swap;
	uint8_t				enable_ahb_md_endian_swap;
};

/**
 * @brief Run-time device configuration data structure.
 *
 * This struct contains all device configuration data for a GEM
 * controller instance which is modifyable at run-time, such as
 * data relating to the attached PHY or the auxiliary thread.
 */
struct eth_xlnx_gem_dev_data {
	struct net_if			*iface;
	uint8_t				mac_addr[6];
	enum eth_xlnx_link_speed	eff_link_speed;

	struct k_work			tx_done_work;
	struct k_work			rx_pend_work;
	struct k_sem			tx_done_sem;

	uint8_t				phy_addr;
	uint32_t			phy_id;
	struct k_delayed_work		phy_poll_delayed_work;
	struct phy_xlnx_gem_api		*phy_access_api;

	uint8_t				*first_rx_buffer;
	uint8_t				*first_tx_buffer;

	struct eth_xlnx_gem_bdring	rxbd_ring;
	struct eth_xlnx_gem_bdring	txbd_ring;

#ifdef CONFIG_NET_STATISTICS_ETHERNET
	struct net_stats_eth		stats;
#endif

	uint8_t				started;
};

#endif /* _ZEPHYR_DRIVERS_ETHERNET_ETH_XLNX_GEM_PRIV_H_ */

/* EOF */