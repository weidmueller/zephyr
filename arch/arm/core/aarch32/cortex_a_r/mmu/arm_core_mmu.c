/* 
 * Simple ARMv7 MMU setup, 4 GB identity mapping
 * 
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_ARMV7_A_MMU

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

#define ARM_MMU_PAGE_PERM_READ	(1 << 0)
#define ARM_MMU_PAGE_PERM_WRITE	(1 << 1)
#define ARM_MMU_PAGE_CACHEABLE	(1 << 2)
#define ARM_MMU_PAGE_BUFFERABLE	(1 << 3)
#define ARM_MMU_PAGE_SHARED	(1 << 4)

#define ARM_MMU_FIRST_LEVEL_SECTION_ID 0x2

union arm_mmu_first_level_pagetable_entry {
	struct {
		uint32_t id		: 2;
		uint32_t bufferable	: 1;
		uint32_t cacheable	: 1;
		uint32_t exe_never	: 1;
		uint32_t domain		: 4;
		uint32_t impl_def	: 1;
		uint32_t acc_perms10	: 2;
		uint32_t tex		: 3;
		uint32_t acc_perms2	: 1;
		uint32_t shared		: 1;
		uint32_t not_global	: 1;
		uint32_t zero		: 1;
		uint32_t non_sec	: 1;
		uint32_t base_address	: 12;
	} data;
	uint32_t word;
};

static uint32_t identity_pagetable[4096] __aligned(16384);

static uint32_t arm_mmu_gen_page_entry(uint32_t base, uint8_t flags) {
	union arm_mmu_first_level_pagetable_entry entry;

	entry.data.id		= ARM_MMU_FIRST_LEVEL_SECTION_ID;
	entry.data.bufferable 	= (flags & ARM_MMU_PAGE_BUFFERABLE) ? 1 : 0;
	entry.data.cacheable	= (flags & ARM_MMU_PAGE_CACHEABLE)  ? 1 : 0;
	entry.data.exe_never	= 0;
	entry.data.domain	= 0;
	entry.data.impl_def	= 0;
	entry.data.acc_perms10	= (flags & ARM_MMU_PAGE_PERM_WRITE) ? 3 : 0;
	entry.data.tex		= 0;
	entry.data.acc_perms2	= 0;
	entry.data.shared	= (flags & ARM_MMU_PAGE_SHARED) ? (1 << 4) : 0;
	entry.data.not_global	= 0;
	entry.data.zero		= 0;
	entry.data.non_sec	= 0;
	entry.data.base_address	= (base >> 20);

	return (uint32_t)entry.word;
}

static int arm_mmu_init(struct device *arg) {
	uint32_t pagetable_base	= (uint32_t)identity_pagetable;
	uint32_t reg_val	= 0;
	uint32_t iter		= 0;

	/* Obtain the base address and the size of the memory area used as RAM
	 * from the device tree data */

	uint32_t ram_base = DT_REG_ADDR(DT_CHOSEN(zephyr_sram));
	uint32_t ram_top  = ram_base + DT_REG_SIZE(DT_CHOSEN(zephyr_sram));

	/* Configure TTB Registers */

	__asm__ __volatile__ ("mcr p15,0,%0,c2,c0,0" : : "r"(pagetable_base));
	__asm__ __volatile__ ("mcr p15,0,%0,c2,c0,1" : : "r"(pagetable_base));
	__asm__ __volatile__ ("mcr p15,0,%0,c2,c0,2" : : "r"(reg_val));

	/* Configure Domain Access Control -> set to manager mode */

	reg_val = -1;
	__asm__ __volatile__ ("mcr p15,0,%0,c3,c0,0" : : "r"(reg_val));

	/* Set up a simple 4GB address space virt = phys page table */

	for (iter = 0; iter < 4096; iter++)
	{
		if (((iter * 0x00100000) >= ram_base)
			&& ((iter * 0x00100000)  < ram_top)) {
			/* Memory used as RAM */
			identity_pagetable[iter] = arm_mmu_gen_page_entry(
				(iter * 0x00100000),
				(   ARM_MMU_PAGE_PERM_READ 
				  | ARM_MMU_PAGE_PERM_WRITE 
				  | ARM_MMU_PAGE_CACHEABLE 
				  | ARM_MMU_PAGE_BUFFERABLE));
		} else {
			/* Strongly ordered */
			identity_pagetable[iter] = arm_mmu_gen_page_entry(
				(iter * 0x00100000),
				(   ARM_MMU_PAGE_PERM_READ 
				  | ARM_MMU_PAGE_PERM_WRITE));
		}
	}

	/* Enable the MMU */

	__asm__ __volatile__ ("mrc p15,0,%0,c1,c0,0" : "=r"(reg_val));
	reg_val |= 1;
	__asm__ __volatile__ ("mcr p15,0,%0,c1,c0,0" : : "r"(reg_val));

	return 0;
}

SYS_INIT(arm_mmu_init, PRE_KERNEL_1,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /* CONFIG_ARMV7_A_MMU */
