/*
 * Xilinx Processor System CAN controller driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 *
 * Copyright (c) 2021, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include "can_xlnx.h"

#define DT_DRV_COMPAT xlnx_can

#define LOG_MODULE_NAME xlnx_can
#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int can_xlnx_init(struct device *dev)
{
	struct can_xlnx_dev_cfg *dev_conf = DEV_CFG(dev);

	/* Timing parameter validity checks:
	 * the timing parameters checked here will be written into the can.BTR
	 * register. If a value n is written for any of those paramters, the
	 * effective value will be n+1. Therefore, none of the following para-
	 * meters may have a zero value. Also, the following range limitations
	 * apply:
	 * - SJW: 2 bits -> range [0 ..  3] -> effective [1 ..  4]
	 * - TS1: 4 bits -> range [0 .. 15] -> effective [1 .. 16]
	 * - TS2: 3 bits -> range [0 ..  7] -> effective [1 ..  8]
	 * Another restriction to consider is that the TS1 bit field in the
	 * can.BTR register is not just the value of the parameter PS1, but
	 * the sum of PS1 and the Propagation Segment. Therefore, the Propa-
	 * gation Segment and PS1 values must at least be 1, must be less than
	 * 16, and the sum must not exceed 16. */
	__ASSERT((dev_conf->sjw        >= 1) && (dev_conf->sjw        <= 4),
		"CAN timing parameter sjw must be in range [1 .. 4]");
	__ASSERT((dev_conf->phase_seg2 >= 1) && (dev_conf->phase_seg2 <= 8),
		"CAN timing parameter phase-seg2 must be in range [1 .. 8]");
	__ASSERT((dev_conf->prop_seg >= 1)   && (dev_conf->prop_seg   <= 15),
		"CAN timing parameter prop-seg must be in range [1 .. 15]");	
	__ASSERT((dev_conf->phase_seg1 >= 1) && (dev_conf->phase_seg1 <= 15),
		"CAN timing parameter phase-seg1 must be in range [1 .. 15]");
	__ASSERT((dev_conf->phase_seg1 + dev_conf->prop_seg) <= 16),
		"Sum of CAN timing parameters prop-seg and phase-seg1 must \
		be in range [1 .. 16]");

	/* Reset the controller: can.SRR[SRST] */
	sys_write32(CAN_XLNX_SRR_SRST_BIT,
		dev_conf->base_addr + CAN_XLNX_SRR_OFFSET);

	return 0;
}

static int can_xlnx_configure(struct device *dev, enum can_mode mode,
	uint32_t bitrate)
{
	struct can_xlnx_dev_cfg		*dev_conf = DEV_CFG(dev);
	struct can_xlnx_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t 			reg_val   = 0;

	/* Enter configuration mode:
	 * - Clear the can.SRR[CEN] bit
	 * - Poll the can.SR[CONFIG] bit until it indicates that configuration
	 *   mode has become active.
	 * Register details: Zynq-7000 TRM, Appendix B.5, p. 806f. */
	reg_val = sys_read32(dev_conf->base_addr + CAN_XLNX_SRR_OFFSET);
	reg_val &= ~CAN_XLNX_SRR_CEN_BIT;
	sys_write32(reg_val, dev_conf->base_addr + CAN_XLNX_SRR_OFFSET);

	do {
		reg_val = sys_read32(dev_conf->base_addr + CAN_XLNX_SR_OFFSET);
	} while ((reg_val & CAN_XLNX_SR_CONFIG_BIT) == 0);

	/* Store the configured settings */
	dev_data->mode = mode;
	if (bitrate != 0) {
		dev_data->bus_speed = bitrate;
	}

	/* Configure the bit timings in can.BTR
	 * Register details: Zynq-7000 TRM, Appendix B.5, p. 809 */
	reg_val  = ((dev_conf->sjw - 1) & CAN_XLNX_BTR_SJW_MASK)
		<< CAN_XLNX_BTR_SJW_SHIFT;
	reg_val |= ((dev_conf->phase_seg2 - 1) & CAN_XLNX_BTR_TS2_MASK)
		<< CAN_XLNX_BTR_TS2_SHIFT;
	reg_val |= (((dev_conf->phase_seg1 - 1) + (dev_conf->prop_seg1 - 1))
		& CAN_XLNX_BTR_TS1_MASK) << CAN_XLNX_BTR_TS1_SHIFT;
	sys_write32(reg_val, dev_conf->base_addr + CAN_XLNX_BTR_OFFSET);

	return 0;
}

static int can_xlnx_send(struct device *dev, const struct zcan_frame *msg,
	k_timeout_t timeout, can_tx_callback_t callback_isr,
	void *callback_arg)
{
	return 0;
}

static int can_xlnx_attach_isr(struct device *dev, can_rx_callback_t isr,
	void *callback_arg,
	const struct zcan_filter *filter)
{
	return 0;
}

static void can_xlnx_detach(struct device *dev, int filter_nr)
{

}

static enum can_state can_xlnx_get_state(struct device *dev,
	struct can_bus_err_cnt *err_cnt)
{
	return CAN_BUS_OFF;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
int can_xlnx_recover(struct device *dev, k_timeout_t timeout)
{
	return 0;
}
#endif

static void can_xlnx_register_state_change_isr(struct device *dev,
	can_state_change_isr_t isr)
{
	struct can_xlnx_dev_data *dev_data = DEV_DATA(dev);

	dev_data->state_change_isr = isr;
}

/*
 * CAN Driver API declaration, required by the upcoming instances of
 * the DEVICE_AND_API_INIT macro for each activated device instance
 */
static const struct can_driver_api can_xlnx_api_funcs = {
	.configure			= can_xlnx_configure,
	.send				= can_xlnx_send,
	.attach_isr			= can_xlnx_attach_isr,
	.detach				= can_xlnx_detach,
	.get_state			= can_xlnx_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover			= can_xlnx_recover,
#endif
	.register_state_change_isr	= can_xlnx_register_state_change_isr
};

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)

static struct can_xlnx_dev_cfg can_xlnx_can0_dev_cfg = {
	.base_addr = DT_INST_REG_ADDR(0),
	.pll_clock_frequency = DT_INST_PROP(0, clock_frequency),
	.sjw = DT_INST_PROP(0, sjw),
	.prop_seg = DT_INST_PROP(0, prop_seg),
	.phase_seg1 = DT_INST_PROP(0, phase_seg1),
	.phase_seg2 = DT_INST_PROP(0, phase_seg2),
	.bus_speed = DT_INST_PROP(0, bus_speed),
}

static struct can_xlnx_dev_data can_xlnx_can0_dev_data = {
	.state_change_isr = NULL,
	.bus_speed = DT_INST_PROP(0, bus_speed),
};

DEVICE_AND_API_INIT(can_xlnx_can0, DT_INST_LABEL(0), &can_xlnx_init,
	&can_xlnx_can0_dev_data, &can_xlnx_can0_dev_cfg, POST_KERNEL,
	CONFIG_CAN_INIT_PRIORITY, &can_xlnx_api_funcs);

#endif /* CAN0 activated in device tree */

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)

static struct can_xlnx_dev_cfg can_xlnx_can1_dev_cfg = {
	.base_addr = DT_INST_REG_ADDR(1),
	.pll_clock_frequency = DT_INST_PROP(1, clock_frequency),
	.sjw = DT_INST_PROP(1, sjw),
	.prop_seg = DT_INST_PROP(1, prop_seg),
	.phase_seg1 = DT_INST_PROP(1, phase_seg1),
	.phase_seg2 = DT_INST_PROP(1, phase_seg2),
	.bus_speed = DT_INST_PROP(1, bus_speed),
}

static struct can_xlnx_dev_data can_xlnx_can1_dev_data = {
	.state_change_isr = NULL,
	.bus_speed = DT_INST_PROP(1, bus_speed),
};

DEVICE_AND_API_INIT(can_xlnx_can1, DT_INST_LABEL(1), &can_xlnx_init,
	&can_xlnx_can1_dev_data, &can_xlnx_can1_dev_cfg, POST_KERNEL,
	CONFIG_CAN_INIT_PRIORITY, &can_xlnx_api_funcs);

#endif /* CAN1 activated in device tree */

/* EOF */