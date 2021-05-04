/*
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <init.h>

#include "soc.h"

/**
 * @brief Basic hardware initialization of the Zynq-7000 SoC
 *
 * Performs the basic initialization of the Zynq-7000 SoC.
 *
 * @return 0
 */
static int soc_xlnx_zynq7000_init(struct device *arg)
{
	ARG_UNUSED(arg);
	NMI_INIT();

	return 0;
}

SYS_INIT(soc_xlnx_zynq7000_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/* EOF */