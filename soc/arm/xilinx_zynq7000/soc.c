/*
 * Copyright (c) 2021 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <init.h>
#include <sys/util.h>

#include <arch/arm/aarch32/mmu/arm_mmu.h>
#include "soc.h"

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("Vectors",
			      0x00000000,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
	MMU_REGION_FLAT_ENTRY("SLCR",
			      0xF8000000,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("MPCore",
			      0xF8F00000,
			      0x2000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	/* ARM Arch timer, GIC are covered by the MPCore mapping */
/* UARTs */
#if DT_NODE_HAS_STATUS(DT_INST(0, xlnx_xuartps), okay)
	MMU_REGION_FLAT_ENTRY("UART0",
			      DT_REG_ADDR(DT_INST(0, xlnx_xuartps)),
			      DT_REG_SIZE(DT_INST(0, xlnx_xuartps)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif
#if DT_NODE_HAS_STATUS(DT_INST(1, xlnx_xuartps), okay)
	MMU_REGION_FLAT_ENTRY("UART1",
			      DT_REG_ADDR(DT_INST(1, xlnx_xuartps)),
			      DT_REG_SIZE(DT_INST(1, xlnx_xuartps)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif

/* GEMs */
#if DT_NODE_HAS_STATUS(DT_INST(0, xlnx_gem), okay)
	MMU_REGION_FLAT_ENTRY("GEM0",
			      DT_REG_ADDR(DT_INST(0, xlnx_gem)),
			      DT_REG_SIZE(DT_INST(0, xlnx_gem)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif
#if DT_NODE_HAS_STATUS(DT_INST(1, xlnx_gem), okay)
	MMU_REGION_FLAT_ENTRY("GEM1",
			      DT_REG_ADDR(DT_INST(1, xlnx_gem)),
			      DT_REG_SIZE(DT_INST(1, xlnx_gem)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};

/**
 * @brief Basic hardware initialization of the Zynq-7000 SoC
 *
 * Performs the basic initialization of the Zynq-7000 SoC.
 *
 * @return 0
 */
static int soc_xlnx_zynq7000_init(const struct device *arg)
{
	uint32_t reg_val;

	ARG_UNUSED(arg);
	NMI_INIT();

	return 0;
}

SYS_INIT(soc_xlnx_zynq7000_init, PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/* EOF */