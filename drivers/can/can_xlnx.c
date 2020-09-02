/*
 * Xilinx Processor System CAN controller driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 *
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include "can_xlnx.h"

#define DT_DRV_COMPAT xlnx_can

#define LOG_MODULE_NAME can_xlnx
#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int can_xlnx_init(struct device *dev)
{
	return 0;
}

static int can_xlnx_configure(struct device *dev, enum can_mode mode,
	uint32_t bitrate)
{
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

int can_xlnx_recover(struct device *dev, k_timeout_t timeout)
{
	return 0;
}

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
	.tq_sjw = DT_INST_PROP(0, sjw),
	.tq_prop = DT_INST_PROP(0, prop_seg),
	.tq_bs1 = DT_INST_PROP(0, phase_seg1),
	.tq_bs2 = DT_INST_PROP(0, phase_seg2),
	.bus_speed = DT_INST_PROP(0, bus_speed),
#ifdef CONFIG_CAN_XLNX_PORT_0_LOOPBACK
	.loopback  = 1,
#else
	.loopback  = 0,
#endif
}

static struct can_xlnx_dev_data can_xlnx_can0_dev_data = {
	.state_change_isr = NULL,
};

DEVICE_AND_API_INIT(can_xlnx_can0, DT_INST_LABEL(0), &can_xlnx_init,
	&can_xlnx_can0_dev_data, &can_xlnx_can0_dev_cfg, POST_KERNEL,
	CONFIG_CAN_INIT_PRIORITY, &can_xlnx_api_funcs);

#endif /* CAN0 activated in device tree */

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)

static struct can_xlnx_dev_cfg can_xlnx_can1_dev_cfg = {
	.base_addr = DT_INST_REG_ADDR(1),
	.tq_sjw = DT_INST_PROP(1, sjw),
	.tq_prop = DT_INST_PROP(1, prop_seg),
	.tq_bs1 = DT_INST_PROP(1, phase_seg1),
	.tq_bs2 = DT_INST_PROP(1, phase_seg2),
	.bus_speed = DT_INST_PROP(1, bus_speed),
#ifdef CONFIG_CAN_XLNX_PORT_1_LOOPBACK
	.loopback  = 1,
#else
	.loopback  = 0,
#endif
}

static struct can_xlnx_dev_data can_xlnx_can1_dev_data = {
	.state_change_isr = NULL,
};

DEVICE_AND_API_INIT(can_xlnx_can1, DT_INST_LABEL(1), &can_xlnx_init,
	&can_xlnx_can1_dev_data, &can_xlnx_can1_dev_cfg, POST_KERNEL,
	CONFIG_CAN_INIT_PRIORITY, &can_xlnx_api_funcs);

#endif /* CAN1 activated in device tree */

/* EOF */