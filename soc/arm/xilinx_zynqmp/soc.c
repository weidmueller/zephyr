/*
 * Copyright (c) 2019 Lexmark International, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/arm/aarch32/cortex_a_r/cmsis.h>

#include "soc.h"

/**
 * @brief Basic hardware initialization of the ZynqMP SoC
 *
 * Performs the basic initialization of the ZynqMP SoC.
 *
 * @return 0
 */
static int soc_xlnx_zynqmp_init(struct device *arg)
{
	ARG_UNUSED(arg);
	NMI_INIT();

	return 0;
}

SYS_INIT(soc_xlnx_zynqmp_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

void z_platform_init(void)
{
	/*
	 * Use normal exception vectors address range (0x0-0x1C).
	 */
	unsigned int sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_V_Msk;
	__set_SCTLR(sctlr);
}
