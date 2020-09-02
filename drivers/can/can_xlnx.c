/*
 * Xilinx Processor System CAN controller driver
 * for Zynq-7000 and ZynqMP (UltraScale) SoCs
 *
 * Copyright (c) 2020, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

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
