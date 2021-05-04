/*
 * Xilinx Processor System CAN controller driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 *
 * Copyright (c) 2021, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_XLNX_CAN_H_
#define ZEPHYR_DRIVERS_CAN_XLNX_CAN_H_

#include <drivers/can.h>

/* Register offsets within the respective CAN controller's address space */
/* Comp. Zynq-7000 Technical Reference Manual (ug585), Appendix B.5 */
#define CAN_XLNX_SRR_OFFSET		0x00000000 /* can.SRR          Software Reset                                    register */
#define CAN_XLNX_MSR_OFFSET		0x00000004 /* can.MSR          Mode Select                                       register */
#define CAN_XLNX_BRPR_OFFSET		0x00000008 /* can.BRPR         Baud Rate Prescaler                               register */
#define CAN_XLNX_BTR_OFFSET		0x0000000C /* can.BTR          Bit Timing                                        register */
#define CAN_XLNX_ECR_OFFSET		0x00000010 /* can.ECR          Error Counter                                     register */
#define CAN_XLNX_ESR_OFFSET		0x00000014 /* can.ESR          Error Status                                      register */
#define CAN_XLNX_SR_OFFSET		0x00000018 /* can.SR           Status                                            register */
#define CAN_XLNX_ISR_OFFSET		0x0000001C /* can.ISR          Interrupt Status                                  register */
#define CAN_XLNX_IER_OFFSET		0x00000020 /* can.IER          Interrupt Enable                                  register */
#define CAN_XLNX_ICR_OFFSET		0x00000024 /* can.ICR          Interrupt Clear                                   register */
#define CAN_XLNX_TCR_OFFSET		0x00000028 /* can.TCR          Timestamp Control                                 register */
#define CAN_XLNX_WIR_OFFSET		0x0000002C /* can.WIR          Watermark Interrupt                               register */
#define CAN_XLNX_TXFIFO_ID_OFFSET	0x00000030 /* can.TXFIFO_ID    Transmit Message FIFO Message Identifier          register */
#define CAN_XLNX_TXFIFO_DLC_OFFSET	0x00000034 /* can.TXFIFO_DLC   Transmit Message FIFO Data Length Code            register */
#define CAN_XLNX_TXFIFO_DATA1_OFFSET	0x00000038 /* can.TXFIFO_DATA1 Transmit Message FIFO Data Word 1                 register */
#define CAN_XLNX_TXFIFO_DATA2_OFFSET	0x0000003C /* can.TXFIFO_DATA2 Transmit Message FIFO Data Word 2                 register */
#define CAN_XLNX_TXHPB_ID_OFFSET	0x00000040 /* can.TXHPB_ID     Transmit Message High Priority Message Identifier register */
#define CAN_XLNX_TXHPB_OFFSET		0x00000044 /* can.TXHPB_DLC    Transmit Message High Priority Data Length Code   register */
#define CAN_XLNX_TXHPB_DATA1_OFFSET	0x00000048 /* can.TXHPB_DATA1  Transmit Message High Priority Data Word 1        register */
#define CAN_XLNX_TXPHB_DATA2_OFFSET	0x0000004C /* can.TXHPB_DATA2  Transmit Message High Priority Data Word 2        register */
#define CAN_XLNX_RXFIFO_ID_OFFSET	0x00000050 /* can.RXFIFO_ID    Receive Message FIFO Message Identifier           register */
#define CAN_XLNX_RXFIFO_DLC_OFFSET	0x00000054 /* can.RXFIFO_DLC   Receive Message FIFO Data Length Code             register */
#define CAN_XLNX_RXFIFO_DATA1_OFFSET	0x00000058 /* can.RXFIFO_DATA1 Receive Message FIFO Data Word 1                  register */
#define CAN_XLNX_RXFIFO_DATA2_OFFSET	0x0000005C /* can.RXFIFO_DATA2 Receive Message FIFO Data Word 2                  register */
#define CAN_XLNX_AFR_OFFSET		0x00000060 /* can.AFR          Acceptance File                                   register */
#define CAN_XLNX_AFMR1_OFFSET		0x00000064 /* can.AFMR1        Acceptance Filter Mask 1                          register */
#define CAN_XLNX_AFIR1_OFFSET		0x00000068 /* can.AFIR1        Acceptance Filter ID 1                            register */
#define CAN_XLNX_AFMR2_OFFSET		0x0000006C /* can.AFMR2        Acceptance Filter Mask 2                          register */
#define CAN_XLNX_AFIR2_OFFSET		0x00000070 /* can.AFIR2        Acceptance Filter ID 2                            register */
#define CAN_XLNX_AFMR3_OFFSET		0x00000074 /* can.AFMR3        Acceptance Filter Mask 3                          register */
#define CAN_XLNX_AFIR3_OFFSET		0x00000078 /* can.AFIR3        Acceptance Filter ID 3                            register */
#define CAN_XLNX_AFMR4_OFFSET		0x0000007C /* can.AFMR4        Acceptance Filter Mask 4                          register */
#define CAN_XLNX_AFIR4_OFFSET		0x00000080 /* can.AFIR4        Acceptance Filter ID 4                            register */

/* (Shift) masks for individual registers' fields */
#define CAN_XLNX_SRR_CEN_BIT		0x00000002 /* can.SRR:         CAN enable bit */
#define CAN_XLNX_SRR_SRST_BIT		0x00000001 /* can.SRR:         Controller reset bit */

#define CAN_XLNX_MSR_SNOOP_BIT		0x00000004 /* can.MSR:         Snoop Mode select bit */
#define CAN_XLNX_MSR_LBACK_BIT		0x00000002 /* can.MSR:         Loopback Mode select bit */
#define CAN_XLNX_MSR_SLEEP_BIT		0x00000001 /* can.MSR:         Sleep Mode select bit */

#define CAN_XLNX_BRPR_PRESCALER_MASK	0x000000FF /* can.BRPR:        Baud Rate Prescaler value mask */

#define CAN_XLNX_BTR_SJW_MASK		0x00000003 /* can.BTR:         Mask for Synchronization Jump Width value */
#define CAN_XLNX_BTR_SJW_SHIFT		7          /* can.BTR:         Shift count for Synchronization Jump Width value */
#define CAN_XLNX_BTR_TS2_MASK		0x00000007 /* can.BTR:         Mask for Time Segement 2 value */
#define CAN_XLNX_BTR_TS2_SHIFT		4          /* can.BTR:         Shift count for Time Segement 2 value */
#define CAN_XLNX_BTR_TS1_MASK		0x0000000F /* can.BTR:         Mask for Time Segement 1 value */
#define CAN_XLNX_BTR_TS1_SHIFT		0          /* can.BTR:         Shift count for Time Segement 1 value */

#define CAN_XLNX_ECR_REC_MASK		0x000000FF /* can.REC:         Mask for Receive Error Counter value */
#define CAN_XLNX_ECR_REC_SHIFT		8          /* can.REC:         Shift count for Receive Error Counter value */
#define CAN_XLNX_ECR_TEC_MASK		0x000000FF /* can.REC:         Mask for Transmit Error Counter value */
#define CAN_XLNX_ECR_TEC_SHIFT		0          /* can.REC:         Shift count for Transmit Error Counter value */

#define CAN_XLNX_ESR_ACKER_BIT		0x00000010 /* can.ESR:         ACK Error bit */
#define CAN_XLNX_ESR_BERR_BIT		0x00000008 /* can.ESR:         Bit Error bit */
#define CAN_XLNX_ESR_STER_BIT		0x00000004 /* can.ESR:         Stuff Error bit */
#define CAN_XLNX_ESR_FMER_BIT		0x00000002 /* can.ESR:         Form Error bit */
#define CAN_XLNX_ESR_CRCER_BIT		0x00000001 /* can.ESR:         CRC Error bit */

#define CAN_XLNX_SR_CONFIG_BIT		0x00000001 /* can.SR:          Configuration Mode Indicator bit */
#define CAN_XLNX_SR_LBACK_BIT		0x00000002 /* can.SR:          Loopback Mode bit */
#define CAN_XLNX_SR_SLEEP_BIT		0x00000004 /* can.SR:          Sleep Mode bit */
#define CAN_XLNX_SR_NORMAL_BIT		0x00000008 /* can.SR:          Normal Mode bit */
#define CAN_XLNX_SR_BIDLE_BIT		0x00000010 /* can.SR:          Bus Idle bit */
#define CAN_XLNX_SR_BBSY_BIT		0x00000020 /* can.SR:          Bus Busy bit */
#define CAN_XLNX_SR_ERRWRN_BIT		0x00000040 /* can.SR:          Error Warning bit */
#define CAN_XLNX_SR_ESTAT_MASK		0x00000003 /* can.SR:          Mask for Error Status value */
#define CAN_XLNX_SR_ESTAT_SHIFT		7          /* can.SR:          Shift count for Error Status value */
#define CAN_XLNX_SR_TXBFLL_BIT		0x00000200 /* can.SR:          High Priority Transmit Buffer Full bit */
#define CAN_XLNX_SR_TXFLL_BIT		0x00000400 /* can.SR:          Transmit FIFO Full bit */
#define CAN_XLNX_SR_ACFBSY_BIT		0x00000800 /* can.SR:          Acceptance Filter Busy Indicator bit */
#define CAN_XLNX_SR_SNOOP_BIT		0x00001000 /* can.SR:          Snoop Mode bit */

#define CAN_XLNX_IXR_TXFEMP_BIT		0x00004000 /* can.IxR:         Transmit FIFO Empty Interrupt bit */
#define CAN_XLNX_IXR_TXFWMEMP_BIT	0x00002000 /* can.IxR:         Transmit FIFO Watermark Empty Interrupt bit */
#define CAN_XLNX_IXR_RXFWMFLL_BIT	0x00001000 /* can.IxR:         Receive FIFO Watermark Full Interrupt bit */
#define CAN_XLNX_IXR_WKUP_BIT		0x00000800 /* can.IxR:         Wake Up Interrupt bit */
#define CAN_XLNX_IXR_SLP_BIT		0x00000400 /* can.IxR:         Sleep Interrupt bit */
#define CAN_XLNX_IXR_BSOFF_BIT		0x00000200 /* can.IxR:         Bus Off Interrupt bit */
#define CAN_XLNX_IXR_ERROR_BIT		0x00000100 /* can.IxR:         Error Interrupt bit */
#define CAN_XLNX_IXR_RXNEMP_BIT		0x00000080 /* can.IxR:         Receive FIFO Not Empty Interrupt bit */
#define CAN_XLNX_IXR_RXOFLW_BIT		0x00000040 /* can.IxR:         Receive FIFO Overflow Interrupt bit */
#define CAN_XLNX_IXR_RXUFLW_BIT		0x00000020 /* can.IxR:         Receive FIFO Underflow Interrupt bit */
#define CAN_XLNX_IXR_RXOK_BIT		0x00000010 /* can.IxR:         New Message Received Interrupt bit */
#define CAN_XLNX_IXR_TXBFLL_BIT		0x00000008 /* can.IxR:         High Priority Transmit Buffer Full Interrupt bit */
#define CAN_XLNX_IXR_TXFLL_BIT		0x00000004 /* can.IxR:         Transmit FIFO Full Interrupt bit */
#define CAN_XLNX_IXR_TXOK_BIT		0x00000002 /* can.IxR:         Transmission Successful Interrupt bit */
#define CAN_XLNX_IXR_ARBLST_BIT		0x00000001 /* can.IxR:         Arbitration Lost Interrupt bit */

#define CAN_XLNX_TCR_CTS_BIT		0x00000001 /* can.TCR:         Clear Timestamp bit */

#define CAN_XLNX_WIR_EW_MASK		0x000000FF /* can.REC:         Mask for Transmit FIFO Empty Watermark value */
#define CAN_XLNX_WIR_EW_SHIFT		8          /* can.REC:         Shift count for Transmit FIFO Empty Watermark value */
#define CAN_XLNX_WIR_FW_MASK		0x000000FF /* can.REC:         Mask for Receive FIFO Empty Watermark value */
#define CAN_XLNX_WIR_FW_SHIFT		0          /* can.REC:         Shift count for Receive FIFO Empty Watermark value */

#define CAN_XLNX_XXFIFO_ID_IDH_MASK	0x000007FF /* can.xxFIFO_ID:   Mask for Standard Message ID value */
#define CAN_XLNX_XXFIFO_ID_IDH_SHIFT	21         /* can.xxFIFO_ID:   Shift count for Standard Message ID value */
#define CAN_XLNX_XXFIFO_SRRRTR_BIT	0x00100000 /* can.xxFIFO_ID:   Substitute Remote Transmission Request bit */
#define CAN_XLNX_XXFIFO_IDE_BIT		0x00080000 /* can.xxFIFO_ID:   Identifier Extension bit */
#define CAN_XLNX_XXFIFO_IDL_MASK	0x0002FFFF /* can.xxFIFO_ID:   Mask for Extended Message ID value */
#define CAN_XLNX_XXFIFO_IDL_SHIFT	1          /* can.xxFIFO_ID:   Shift count for Extended Message ID value */
#define CAN_XLNX_XXFIFO_RTR_BIT		0x00000001 /* can.xxFIFO_ID:   Remote Transmission Request bit */

#define CAN_XLNX_XXFIFO_DLC_MASK	0x0000000F /* can.xxFIFO_DLC:  Mask for Data Length Code value */
#define CAN_XLNX_XXFIFO_DLC_SHIFT	28         /* can.xxFIFO_DLC:  Shift count for Data Length Code value */

#define CAN_XLNX_TXHPB_ID_IDH_MASK	0x000007FF /* can.TXHPB_ID:    Mask for Standard Message ID value */
#define CAN_XLNX_TXHPB_ID_IDH_SHIFT	21         /* can.TXHPB_ID:    Shift count for Standard Message ID value */
#define CAN_XLNX_TXHPB_SRRRTR_BIT	0x00100000 /* can.TXHPB_ID:    Substitute Remote Transmission Request bit */
#define CAN_XLNX_TXHPB_IDE_BIT		0x00080000 /* can.TXHPB_ID:    Identifier Extension bit */
#define CAN_XLNX_TXHPB_IDL_MASK		0x0002FFFF /* can.TXHPB_ID:    Mask for Extended Message ID value */
#define CAN_XLNX_TXHPB_IDL_SHIFT	1          /* can.TXHPB_ID:    Shift count for Extended Message ID value */
#define CAN_XLNX_TXHPB_RTR_BIT		0x00000001 /* can.TXHPB_ID:    Remote Transmission Request bit */

#define CAN_XLNX_TXHPB_DLC_MASK		0x0000000F /* can.TXHPB_DLC:   Mask for Data Length Code value */
#define CAN_XLNX_TXHPB_DLC_SHIFT	28         /* can.TXHPB_DLC:   Shift count for Data Length Code value */

#define CAN_XLNX_AFR_UAF4_BIT		0x00000008 /* can.AFR:         Use Acceptance Filter #4 bit */
#define CAN_XLNX_AFR_UAF3_BIT		0x00000004 /* can.AFR:         Use Acceptance Filter #3 bit */
#define CAN_XLNX_AFR_UAF2_BIT		0x00000002 /* can.AFR:         Use Acceptance Filter #2 bit */
#define CAN_XLNX_AFR_UAF1_BIT		0x00000001 /* can.AFR:         Use Acceptance Filter #1 bit */

#define CAN_XLNX_AFMRX_AMIDH_MASK	0x000007FF /* can.AFMRx:       Mask for Standard Message ID Filter Mask value */
#define CAN_XLNX_AFMRX_AMIDH_SHIFT	21         /* can.AMFRx:       Shift count for Standard Message ID Filter Mask value */
#define CAN_XLNX_AFMRX_AMSRR_BIT	0x00100000 /* can.AMFRx:       Substitute Remote Transmission Request Mask bit */
#define CAN_XLNX_AFMRX_AMIDE_BIT	0x00080000 /* can.AMFRx:       Identifier Extension Mask bit */
#define CAN_XLNX_AFMRX_AMIDL_MASK	0x0002FFFF /* can.AMFRx:       Mask for Extended Message ID Filter Mask value */
#define CAN_XLNX_AFMRX_AMIDL_SHIFT	1          /* can.AMFRx:       Shift count for Extended Message ID Filter Mask value */
#define CAN_XLNX_AFMRX_AMRTR_BIT	0x00000001 /* can.AMFRx:       Remote Transmission Request Mask bit */

#define CAN_XLNX_AFIRX_AIIDH_MASK	0x000007FF /* can.AFIRx:       Mask for Standard Message ID value */
#define CAN_XLNX_AFIRX_AIIDH_SHIFT	21         /* can.AFIRx:       Shift count for Standard Message ID value */
#define CAN_XLNX_AFIRX_AISRR_BIT	0x00100000 /* can.AFIRx:       Substitute Remote Transmission Request bit */
#define CAN_XLNX_AFIRX_AIIDE_BIT	0x00080000 /* can.AFIRx:       Identifier Extension bit */
#define CAN_XLNX_AFIRX_AIIDL_MASK	0x0002FFFF /* can.AFIRx:       Mask for Extended Message ID Mask Extended value */
#define CAN_XLNX_AFIRX_AIIDL_SHIFT	1          /* can.AFIRx:       Shift count for Extended Message ID Mask Extended value */
#define CAN_XLNX_AFIRX_AIRTR_BIT	0x00000001 /* can.AFIRx:       Remote Transmission Request Mask bit */

/* Device configuration / run-time data resolver macros */
#define DEV_CFG(dev) \
	((struct can_xlnx_dev_cfg *)(dev)->config_info)
#define DEV_DATA(dev) \
	((struct can_xlnx_dev_data *)(dev)->driver_data)

/**
 * @brief Constant device configuration data structure.
 *
 * This struct contains all device configuration data for a CAN
 * controller instance which is constant. The data herein is
 * either acquired from the generated header file based on the
 * data from Kconfig, or from header file based on the device tree
 * data. Some of the data contained, in particular data relating
 * to clock sources, is specific to either the Zynq-7000 or the
 * UltraScale SoCs, which both contain the CAN controller.
 */
struct can_xlnx_dev_cfg {
	uint32_t	base_addr;
	uint32_t	pll_clock_frequency;

	uint8_t		sjw;
	uint8_t		prop_seg;
	uint8_t		phase_seg1;
	uint8_t		phase_seg2;

	uint32_t	bus_speed;
};

/**
 * @brief Run-time device configuration data structure.
 *
 * This struct contains all device configuration data for a CAN
 * controller instance which is modifyable at run-time.
 */
struct can_xlnx_dev_data {
	can_state_change_isr_t state_change_isr;
	enum can_mode mode;
	uint32_t bus_speed;
};

#endif /* ZEPHYR_DRIVERS_CAN_XLNX_CAN_H_ */