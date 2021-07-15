/*
 * ARMv7 MMU support
 *
 * This implementation supports the Short-descriptor translation
 * table format. The standard page size is 4 kB, 1 MB sections
 * are only used for mapping the code and data of the Zephyr image.
 * Secure mode and PL1 is always assumed. LPAE and PXN extensions
 * as well as TEX remapping are not supported. The AP[2:1] plus
 * Access flag permissions model is used, as the AP[2:0] model is
 * deprecated. As the AP[2:1] model can only disable write access,
 * the read permission flag is always implied.
 *
 * Reference documentation:
 * ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition,
 * ARM document ID DDI0406C Rev. d, March 2018
 *
 * Copyright (c) 2021 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <linker/linker-defs.h>

#include <logging/log.h>

#include <sys/__assert.h>
#include <sys/util.h>
#include <sys/mem_manage.h>

#include <arch/arm/aarch32/mmu/arm_mmu.h>
#include "arm_mmu_priv.h"

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

static struct arm_mmu_l1_page_table
	l1_page_table __aligned(KB(16));
static struct arm_mmu_l2_page_table
	l2_page_tables[CONFIG_ARM_MMU_NUM_L2_TABLES] __aligned(KB(1));
static struct arm_mmu_l2_page_table_status
	l2_page_tables_status[CONFIG_ARM_MMU_NUM_L2_TABLES] = {{0}};
static uint32_t arm_mmu_l2_tables_free = CONFIG_ARM_MMU_NUM_L2_TABLES;
static uint32_t arm_mmu_l2_next_free_table = 0;

/*
 * Static definition of all code & data memory regions of the
 * current Zephyr image. This information must be available &
 * processed upon MMU initialization.
 */
static const struct arm_mmu_flat_range mmu_zephyr_ranges[] = {
	/* 
	 * Mark the zephyr execution regions (data, bss, noinit, etc.)
	 * cacheable, read / write and non-executable
	 */
	{ .name  = "zephyr_data",
	  .start = _image_ram_start,
	  .end   = _image_ram_end,
	  .attrs = MT_NORMAL | MATTR_SHARED |
		   MPERM_R | MPERM_W |
		   MATTR_CACHE_OUTER_WB_WA | MATTR_CACHE_INNER_WB_WA},

	/* Mark text segment cacheable, read only and executable */
	{ .name  = "zephyr_code",
	  .start = _image_text_start,
	  .end   = _image_text_end,
	  .attrs = MT_NORMAL | MATTR_SHARED |
		   MPERM_R | MPERM_X |
		   MATTR_CACHE_OUTER_WB_nWA | MATTR_CACHE_INNER_WB_nWA},

	/* Mark rodata segment cacheable, read only and non-executable */
	{ .name  = "zephyr_rodata",
	  .start = _image_rodata_start,
	  .end   = _image_rodata_end,
	  .attrs = MT_NORMAL | MATTR_SHARED |
		   MPERM_R |
		   MATTR_CACHE_OUTER_WB_nWA | MATTR_CACHE_INNER_WB_nWA},
#ifdef CONFIG_NOCACHE_MEMORY
	/* Mark nocache segment read / write and non-executable */
	{ .name  = "nocache",
	  .start = _nocache_ram_start,
	  .end   = _nocache_ram_end,
	  .attrs = MT_STRONGLY_ORDERED |
		   MPERM_R | MPERM_W},
#endif
};

static struct arm_mmu_l2_page_table* arm_mmu_assign_l2_table(uint32_t va)
{
	struct arm_mmu_l2_page_table* l2_page_table = NULL;

	__ASSERT(arm_mmu_l2_tables_free > 0,
		 "Cannot set up L2 page table for VA 0x%08X: "
		 "no more free L2 page tables available\n",
		 va);
	__ASSERT(l2_page_tables_status[arm_mmu_l2_next_free_table].entries == 0,
		 "Cannot set up L2 page table for VA 0x%08X: "
		 "expected empty L2 table at index [%u], but the "
		 "entries value is %u\n",
		 va, arm_mmu_l2_next_free_table,
		 l2_page_tables_status[arm_mmu_l2_next_free_table].entries);

	l2_page_tables_status[arm_mmu_l2_next_free_table].l1_index =
		((va >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) & ARM_MMU_PTE_L1_INDEX_MASK);
	l2_page_tables_status[arm_mmu_l2_next_free_table].entries = 0;
	l2_page_table = &l2_page_tables[arm_mmu_l2_next_free_table];

	if (--arm_mmu_l2_tables_free > 0)
	{
		do {
			arm_mmu_l2_next_free_table = (arm_mmu_l2_next_free_table + 1) %
						      CONFIG_ARM_MMU_NUM_L2_TABLES;
		} while (l2_page_tables_status[arm_mmu_l2_next_free_table].entries != 0);
	}

	return l2_page_table;
}

static void arm_mmu_release_l2_table(struct arm_mmu_l2_page_table* l2_page_table) {
	uint32_t l2_page_table_index = 
		((uint32_t)l2_page_table - (uint32_t)l2_page_tables) /
		sizeof(struct arm_mmu_l2_page_table);

	l2_page_tables_status[l2_page_table_index].l1_index = 0;
	if (arm_mmu_l2_tables_free++ == 0)
	{
		arm_mmu_l2_next_free_table = l2_page_table_index;
	}
}

static void arm_mmu_inc_l2_table_entries(struct arm_mmu_l2_page_table* l2_page_table) {
	uint32_t l2_page_table_index = 
		((uint32_t)l2_page_table - (uint32_t)l2_page_tables) /
		sizeof(struct arm_mmu_l2_page_table);

	__ASSERT(l2_page_tables_status[l2_page_table_index].entries < ARM_MMU_PT_L2_NUM_ENTRIES,
		 "Cannot increment entry count of the L2 page table at index "
		 "[%u] / addr %p / ref L1[%u]: maximum entry count already reached",
		 l2_page_table_index, l2_page_table,
		 l2_page_tables_status[l2_page_table_index].l1_index);
	
	++l2_page_tables_status[l2_page_table_index].entries;
}

static void arm_mmu_dec_l2_table_entries(struct arm_mmu_l2_page_table* l2_page_table) {
	uint32_t l2_page_table_index = 
		((uint32_t)l2_page_table - (uint32_t)l2_page_tables) /
		sizeof(struct arm_mmu_l2_page_table);

	__ASSERT(l2_page_tables_status[l2_page_table_index].entries > 0,
		 "Cannot decrement entry count of the L2 page table at index "
		 "[%u] / addr %p / ref L1[%u]: entry count is already zero",
		 l2_page_table_index, l2_page_table,
		 l2_page_tables_status[l2_page_table_index].l1_index);
	
	if (--l2_page_tables_status[l2_page_table_index].entries == 0) {
		arm_mmu_release_l2_table(l2_page_table);
	}
}

static struct arm_mmu_perms_attrs arm_mmu_convert_attr_flags(uint32_t attrs)
{
	struct arm_mmu_perms_attrs perms_attrs = {0};

	__ASSERT(((attrs & MT_MASK) > 0),
		 "Cannot convert attrs word to PTE control bits: no "
		 "memory type specified");
	__ASSERT(!((attrs & MPERM_W) && !(attrs & MPERM_R)),
		 "attrs must not define write permission without read "
		 "permission");
	__ASSERT(!((attrs & MPERM_W) && (attrs & MPERM_X)),
		 "attrs must not define executable memory with write "
		 "permission");

	/*
	 * The translation of the memory type / permissions / attributes
	 * flags in the attrs word to the TEX, C, B, S and AP bits of the
	 * target PTE is based on the reference manual:
	 * TEX, C, B, S: Table B3-10, chap. B3.8.2, p. B3-1363f.
	 * AP          : Table B3-6,  chap. B3.7.1, p. B3-1353.
	 */

	if (attrs & MT_STRONGLY_ORDERED) {
		/* Strongly ordered is always shareable, S bit is ignored */
		perms_attrs.tex = 0;
		perms_attrs.c   = 0;
		perms_attrs.b   = 0;
		perms_attrs.s   = 0;
	} else if (attrs & MT_DEVICE) {
		/* 
		 * Shareability of device memory is determined by TEX, C, B.
		 * The S bit is ignored. C is always 0 for device memory.
		 */
		perms_attrs.s = 0;
		perms_attrs.c = 0;
		if (attrs & MATTR_SHARED) {
			perms_attrs.tex = 0;
			perms_attrs.b   = 1;
		} else {
			perms_attrs.tex = 2;
			perms_attrs.b   = 0;
		}
	} else if (attrs & MT_NORMAL) {
		/* For normal memory, shareability depends on the S bit */
		if (attrs & MATTR_SHARED) {
			perms_attrs.s = 1;
		}
		/*
		 * TEX[2] is always 1. TEX[1:0] contain the outer cache attri-
		 * butes encoding, C and B contain the inner cache attributes
		 * encoding.
		 */
		perms_attrs.tex |= ARM_MMU_TEX2_CACHEABLE_MEMORY;

		if (attrs & MATTR_CACHE_OUTER_WB_WA) {
			perms_attrs.tex |= ARM_MMU_TEX_CACHE_ATTRS_WB_WA;
		} else if (attrs & MATTR_CACHE_OUTER_WT_nWA) {
			perms_attrs.tex |= ARM_MMU_TEX_CACHE_ATTRS_WT_nWA;
		} else if (attrs & MATTR_CACHE_OUTER_WB_nWA) {
			perms_attrs.tex |= ARM_MMU_TEX_CACHE_ATTRS_WB_nWA;
		}

		if (attrs & MATTR_CACHE_INNER_WB_WA) {
			perms_attrs.c = ARM_MMU_C_CACHE_ATTRS_WB_WA;
			perms_attrs.b = ARM_MMU_B_CACHE_ATTRS_WB_WA;
		} else if (attrs & MATTR_CACHE_INNER_WT_nWA) {
			perms_attrs.c = ARM_MMU_C_CACHE_ATTRS_WT_nWA;
			perms_attrs.b = ARM_MMU_B_CACHE_ATTRS_WT_nWA;
		} else if (attrs & MATTR_CACHE_INNER_WB_nWA) {
			perms_attrs.c = ARM_MMU_C_CACHE_ATTRS_WB_nWA;
			perms_attrs.b = ARM_MMU_B_CACHE_ATTRS_WB_nWA;
		}
	}

	if (attrs & MATTR_NON_SECURE) {
		perms_attrs.ns = 1;
	}
	if (attrs & MATTR_NON_GLOBAL) {
		perms_attrs.ng = 1;
	}

	if (!((attrs & MPERM_R) || (attrs & MPERM_W))) {
		perms_attrs.id_mask = 0x0;
	} else {
		perms_attrs.id_mask = 0x3;
	}
	if (!(attrs & MPERM_W)) {
		perms_attrs.ap |= ARM_MMU_PERMS_AP2_DISABLE_WR;
	}
	if (attrs & MPERM_UNPRIVILEGED) {
		perms_attrs.ap |= ARM_MMU_PERMS_AP1_ENABLE_PL0;
	}
	if (!(attrs & MPERM_X)) {
		perms_attrs.xn = 1;
	}

	return perms_attrs;
}

static void arm_mmu_l1_map_section(uint32_t va, uint32_t pa,
				   struct arm_mmu_perms_attrs perms_attrs) 
{
	uint32_t l1_index = (va >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L1_INDEX_MASK;

	__ASSERT(l1_page_table.entries[l1_index].undefined.id == ARM_MMU_PTE_ID_INVALID,
		 "Unexpected non-zero L1 PTE ID %u for VA 0x%08X / PA 0x%08X",
		 l1_page_table.entries[l1_index].undefined.id,
		 va, pa);

	l1_page_table.entries[l1_index].l1_section_1m.id =
		(ARM_MMU_PTE_ID_SECTION & perms_attrs.id_mask);
	l1_page_table.entries[l1_index].l1_section_1m.bufferable = perms_attrs.b;
	l1_page_table.entries[l1_index].l1_section_1m.cacheable = perms_attrs.c;
	l1_page_table.entries[l1_index].l1_section_1m.exe_never = perms_attrs.xn;
	l1_page_table.entries[l1_index].l1_section_1m.domain = 0;
	l1_page_table.entries[l1_index].l1_section_1m.impl_def = 0;
	l1_page_table.entries[l1_index].l1_section_1m.acc_perms10 =
		((perms_attrs.ap & 0x1) << 1) | 0x1;
	l1_page_table.entries[l1_index].l1_section_1m.tex = perms_attrs.tex;
	l1_page_table.entries[l1_index].l1_section_1m.acc_perms2 =
		(perms_attrs.ap >> 1) & 0x1;
	l1_page_table.entries[l1_index].l1_section_1m.shared = perms_attrs.s;
	l1_page_table.entries[l1_index].l1_section_1m.not_global = perms_attrs.ng;
	l1_page_table.entries[l1_index].l1_section_1m.zero = 0;
	l1_page_table.entries[l1_index].l1_section_1m.non_sec = perms_attrs.ns;
	l1_page_table.entries[l1_index].l1_section_1m.base_address =
		(pa >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT);
}

static void arm_mmu_remap_l1_section_to_l2_table(uint32_t va,
						 struct arm_mmu_l2_page_table *l2_page_table)
{
	struct arm_mmu_perms_attrs perms_attrs = {0};
	uint32_t l1_index = (va >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L1_INDEX_MASK;
	uint32_t rem_size = MB(1);

	/*
	 * Extract the permissions and attributes from the current
	 * 1 MB section entry. This data will be carried over to the
	 * resulting L2 page table.
	 */

	perms_attrs.ap = (l1_page_table.entries[l1_index].l1_section_1m.acc_perms2 << 1) |
			 ((l1_page_table.entries[l1_index].l1_section_1m.acc_perms10 >> 1) & 0x1);
	perms_attrs.b = l1_page_table.entries[l1_index].l1_section_1m.bufferable;
	perms_attrs.c = l1_page_table.entries[l1_index].l1_section_1m.cacheable;
	perms_attrs.id_mask = (l1_page_table.entries[l1_index].l1_section_1m.id == 0) ? 0x0 : 0x3;
	perms_attrs.ng = l1_page_table.entries[l1_index].l1_section_1m.not_global;
	perms_attrs.ns = l1_page_table.entries[l1_index].l1_section_1m.non_sec;
	perms_attrs.s = l1_page_table.entries[l1_index].l1_section_1m.shared;
	perms_attrs.tex = l1_page_table.entries[l1_index].l1_section_1m.tex;
	perms_attrs.xn = l1_page_table.entries[l1_index].l1_section_1m.exe_never;

	/* Clear the L1 PTE & re-configure it as a L2 PT reference */
	l1_page_table.entries[l1_index].word = 0;

	l1_page_table.entries[l1_index].l2_page_table_ref.id = ARM_MMU_PTE_ID_L2_PT;
	l1_page_table.entries[l1_index].l2_page_table_ref.zero0 = 0;
	l1_page_table.entries[l1_index].l2_page_table_ref.zero1 = 0;
	l1_page_table.entries[l1_index].l2_page_table_ref.impl_def = 0;
	l1_page_table.entries[l1_index].l2_page_table_ref.domain = 0; // FIXME
	l1_page_table.entries[l1_index].l2_page_table_ref.non_sec =
		perms_attrs.ns;
	l1_page_table.entries[l1_index].l2_page_table_ref.l2_page_table_address =
		(((uint32_t)l2_page_table >> ARM_MMU_PT_L2_ADDR_SHIFT) &
		ARM_MMU_PT_L2_ADDR_MASK);

	/* Align the target VA to the base address of the section we're converting */
	va &= ~(MB(1) - 1);
	while (rem_size > 0) {
		arm_mmu_l2_map_page(va, va, perms_attrs);
		rem_size -= KB(4);
		va += KB(4);
	}
}

static void arm_mmu_l2_map_page(uint32_t va, uint32_t pa,
				struct arm_mmu_perms_attrs perms_attrs)
{
	struct arm_mmu_l2_page_table *l2_page_table = NULL;
	uint32_t l1_index = (va >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L1_INDEX_MASK;
	uint32_t l2_index = (va >> ARM_MMU_PTE_L2_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L2_INDEX_MASK;

	/*
	 * Use the calculated L1 index in order to determine if a L2 page
	 * table is required in order to complete the current mapping.
	 * -> See below for an explanation of the possible scenarios.
	 */

	if (l1_page_table.entries[l1_index].undefined.id == ARM_MMU_PTE_ID_INVALID ||
	    (l1_page_table.entries[l1_index].undefined.id & ARM_MMU_PTE_ID_SECTION) != 0) {
		l2_page_table = arm_mmu_assign_l2_table(pa);
		__ASSERT(l2_page_table != NULL,
			 "Unexpected L2 page table NULL pointer for VA 0x%08X",
			 va);
	}

	/*
	 * Check what is currently present at the corresponding L1 table entry.
	 * The following scenarios are possible:
	 * 1) The L1 PTE's ID bits are zero, as is the rest of the entry.
	 *    In this case, the L1 PTE is currently unused. A new L2 PT to
	 *    refer to in this entry has already been allocated above.
	 * 2) The L1 PTE's ID bits indicate a L2 PT reference entry (01).
	 *    The corresponding L2 PT's address will be resolved using this
	 *    entry.
	 * 3) The L1 PTE's ID bits may or may not be zero, and the rest of
	 *    the descriptor contains some non-zero data. This always indicates
	 *    an existing 1 MB section entry in this place. Checking only the
	 *    ID bits wouldn't be enough, as the only way to indicate a section
	 *    with neither R nor W permissions is to set the ID bits to 00 in
	 *    the AP[2:1] permissions model. As we're now about to map a single
	 *    page overlapping with the 1 MB section, the section has to be
	 *    converted into a L2 table. Afterwards, the current page mapping
	 *    can be added/modified.
	 */

	if (l1_page_table.entries[l1_index].word == 0) {
		/* The matching L1 PT entry is currently unused */
		l1_page_table.entries[l1_index].l2_page_table_ref.id = ARM_MMU_PTE_ID_L2_PT;
		l1_page_table.entries[l1_index].l2_page_table_ref.zero0 = 0;
		l1_page_table.entries[l1_index].l2_page_table_ref.zero1 = 0;
		l1_page_table.entries[l1_index].l2_page_table_ref.impl_def = 0;
		l1_page_table.entries[l1_index].l2_page_table_ref.domain = 0; // FIXME
		l1_page_table.entries[l1_index].l2_page_table_ref.non_sec =
			perms_attrs.ns;
		l1_page_table.entries[l1_index].l2_page_table_ref.l2_page_table_address =
			(((uint32_t)l2_page_table >> ARM_MMU_PT_L2_ADDR_SHIFT) &
			ARM_MMU_PT_L2_ADDR_MASK);
	} else if (l1_page_table.entries[l1_index].undefined.id == ARM_MMU_PTE_ID_L2_PT) {
		/* The matching L1 PT entry already points to a L2 PT */
		l2_page_table = (struct arm_mmu_l2_page_table *)
				((l1_page_table.entries[l1_index].word &
				(ARM_MMU_PT_L2_ADDR_MASK << ARM_MMU_PT_L2_ADDR_SHIFT)));
		/*
		 * The only configuration bit contained in the L2 PT entry is the
		 * NS bit. Set it according to the attributes passed to this function,
		 * warn if there is a mismatch between the current page's NS attribute
		 * value and the value currently contained in the L2 PT entry.
		 */
		if (l1_page_table.entries[l1_index].l2_page_table_ref.non_sec !=
		    perms_attrs.ns) {
			LOG_WRN("NS bit mismatch in L2 PT reference at L1 index [%u], "
				"re-configuring from %u to %u",
				l1_index,
				l1_page_table.entries[l1_index].l2_page_table_ref.non_sec,
				perms_attrs.ns);
			l1_page_table.entries[l1_index].l2_page_table_ref.non_sec =
				perms_attrs.ns;
		}
	} else if (l1_page_table.entries[l1_index].undefined.reserved != 0) {
		/*
		 * The matching L1 PT entry currently holds a 1 MB section entry
		 * in order to save a L2 table (as it's neither completely blank
		 * nor a L2 PT reference), but now we have to map an overlapping
		 * 4 kB page, so the section entry must be converted to a L2 table
		 * first before the individual L2 entry for the page to be mapped is
		 * accessed. A blank L2 PT has already been assigned above.
		 */
		arm_mmu_remap_l1_section_to_l2_table(va, l2_page_table);
	}

	/*
	 * If the matching L2 PTE is blank, increment the number of used entries
	 * in the L2 table. If the L2 PTE already contains some data, we're re-
	 * placing the entry's data instead, the used entry count remains unchanged.
	 * Once again, checking the ID bits might be misleading if the PTE declares
	 * a page which has neither R nor W permissions.
	 */
	if (l2_page_table->entries[l2_index].word == 0) {
		arm_mmu_inc_l2_table_entries(l2_page_table);
	}

	l2_page_table->entries[l2_index].l2_page_4k.id =
		(ARM_MMU_PTE_ID_SMALL_PAGE & perms_attrs.id_mask);
	l2_page_table->entries[l2_index].l2_page_4k.id |= perms_attrs.xn; /* XN in [0] */
	l2_page_table->entries[l2_index].l2_page_4k.bufferable = perms_attrs.b;
	l2_page_table->entries[l2_index].l2_page_4k.cacheable = perms_attrs.c;
	l2_page_table->entries[l2_index].l2_page_4k.acc_perms10 =
		((perms_attrs.ap & 0x1) << 1) | 0x1;
	l2_page_table->entries[l2_index].l2_page_4k.tex = perms_attrs.tex;
	l2_page_table->entries[l2_index].l2_page_4k.acc_perms2 =
		((perms_attrs.ap >> 1) & 0x1);
	l2_page_table->entries[l2_index].l2_page_4k.shared = perms_attrs.s;
	l2_page_table->entries[l2_index].l2_page_4k.not_global = perms_attrs.ng;
	l2_page_table->entries[l2_index].l2_page_4k.pa_base =
		((pa >> ARM_MMU_PTE_L2_SMALL_PAGE_ADDR_SHIFT) &
		ARM_MMU_PTE_L2_SMALL_PAGE_ADDR_MASK);
}

static void arm_mmu_l2_unmap_page(uint32_t va)
{
	struct arm_mmu_l2_page_table *l2_page_table;
	uint32_t l1_index = (va >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L1_INDEX_MASK;
	uint32_t l2_index = (va >> ARM_MMU_PTE_L2_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L2_INDEX_MASK;

	if (l1_page_table.entries[l1_index].undefined.id != ARM_MMU_PTE_ID_L2_PT) {
		LOG_ERR("Cannot unmap virtual memory at 0x%08X: invalid "
			"page table entry type in level 1 page table at "
			"index [%u]", va, l1_index);
		return;
	}

	l2_page_table = (struct arm_mmu_l2_page_table *)
			((l1_page_table.entries[l1_index].word &
			(ARM_MMU_PT_L2_ADDR_MASK << ARM_MMU_PT_L2_ADDR_SHIFT)));

	if ((l2_page_table->entries[l2_index].undefined.id & ARM_MMU_PTE_ID_SMALL_PAGE) !=
			ARM_MMU_PTE_ID_SMALL_PAGE) {
		LOG_ERR("Cannot unmap virtual memory at 0x%08X: invalid "
			"page table entry type in level 2 page table at "
			"L1 index [%u], L2 index [%u]", va, l1_index, l2_index);
		return;
	}

	l2_page_table->entries[l2_index].word = 0;
	
	arm_mmu_dec_l2_table_entries(l2_page_table);
}

int z_arm_mmu_init(void)
{
	uint32_t pt;
	uint32_t pte;
	uint32_t mem_range;
	uint32_t pa;
	uint32_t va;
	uint32_t attrs;
	uint32_t pt_attrs;
	uint32_t rem_size;
	uint32_t reg_val = 0;
	struct arm_mmu_perms_attrs perms_attrs;

	__ASSERT(CONFIG_MMU_PAGE_SIZE == KB(4),
		 "MMU_PAGE_SIZE value %u is invalid, only 4 kB pages are supported\n",
		 CONFIG_MMU_PAGE_SIZE);

	/* Clear the L1 page table */
	for (pte = 0; pte < ARM_MMU_PT_L1_NUM_ENTRIES; pte++) {
		l1_page_table.entries[pte].word = 0; 
	}

	/* Clear the L2 page tables */
	for (pt = 0; pt < CONFIG_ARM_MMU_NUM_L2_TABLES; pt++) {
		for (pte = 0; pte < ARM_MMU_PT_L2_NUM_ENTRIES; pte++) {
			l2_page_tables[pt].entries[pte].word = 0;
		}
	}

	/* Set up the memory regions pre-defined by the image */
	for (mem_range = 0; mem_range < ARRAY_SIZE(mmu_zephyr_ranges); mem_range++) {
		
		pa          = (uint32_t)(mmu_zephyr_ranges[mem_range].start);
		rem_size    = (uint32_t)(mmu_zephyr_ranges[mem_range].end) - pa;
		attrs       = mmu_zephyr_ranges[mem_range].attrs;
		perms_attrs = arm_mmu_convert_attr_flags(attrs);

		if (((uint32_t)&l1_page_table >= pa) && 
				((uint32_t)&l1_page_table < (pa + rem_size))) {
			pt_attrs = attrs;
		}

		while (rem_size > 0) {
			if (rem_size >= MB(1) && (pa & 0xFFFFF) == 0) {
				/*
				 * Remaining area size > 1 MB & matching alignment
				 * -> map a 1 MB section instead of individual 4 kB
				 * pages with identical configuration.
				 */
				arm_mmu_l1_map_section(pa, pa, perms_attrs);
				rem_size -= MB(1);
				pa += MB(1);
			} else {
				arm_mmu_l2_map_page(pa, pa, perms_attrs);
				rem_size -= (rem_size >= KB(4)) ? KB(4) : rem_size;
				pa += KB(4);
			}
		}
	}

	/* Set up the memory regions defined at the SoC level */
	for (mem_range = 0; mem_range < mmu_config.num_regions; mem_range++) {
		pa          = (uint32_t)(mmu_config.mmu_regions[mem_range].base_pa);
		va          = (uint32_t)(mmu_config.mmu_regions[mem_range].base_va);
		rem_size    = (uint32_t)(mmu_config.mmu_regions[mem_range].size);
		attrs       = mmu_config.mmu_regions[mem_range].attrs;
		perms_attrs = arm_mmu_convert_attr_flags(attrs);

		while (rem_size > 0) {
			arm_mmu_l2_map_page(va, pa, perms_attrs);
			rem_size -= (rem_size >= KB(4)) ? KB(4) : rem_size;
			va += KB(4);
			pa += KB(4);
		}
	}

	/* Clear TTBR1 */
	__asm__ __volatile__("mcr p15, 0, %0, c2, c0, 1" : : "r"(reg_val));

	/* Write TTBCR: EAE, security not yet relevant, N[2:0] = 0 */
	__asm__ __volatile__("mcr p15, 0, %0, c2, c0, 2"
			     : : "r"(reg_val));

	/* Write TTBR0 */
	reg_val = ((uint32_t)&l1_page_table.entries[0] & ~0x3FFF);

	/*
	 * Set IRGN, RGN, S in TTBR0 based on the configuration of the
	 * memory area the actual page tables are located in.
	 */
	if (pt_attrs & MATTR_SHARED) {
		reg_val |= ARM_MMU_TTBR_SHAREABLE_BIT;
	}

	if (pt_attrs & MATTR_CACHE_OUTER_WB_WA) {
		reg_val |= (ARM_MMU_TTBR_RGN_OUTER_WB_WA_CACHEABLE <<
			    ARM_MMU_TTBR_RGN_SHIFT);
	} else if (pt_attrs & MATTR_CACHE_OUTER_WT_nWA) {
		reg_val |= (ARM_MMU_TTBR_RGN_OUTER_WT_CACHEABLE <<
			    ARM_MMU_TTBR_RGN_SHIFT);
	} else if (pt_attrs & MATTR_CACHE_OUTER_WB_nWA) {
		reg_val |= (ARM_MMU_TTBR_RGN_OUTER_WB_nWA_CACHEABLE <<
			    ARM_MMU_TTBR_RGN_SHIFT);
	}

	if (pt_attrs & MATTR_CACHE_INNER_WB_WA) {
		reg_val |= ARM_MMU_TTBR_IRGN0_BIT_MP_EXT_ONLY;
	} else if (pt_attrs & MATTR_CACHE_INNER_WT_nWA) {
		reg_val |= ARM_MMU_TTBR_IRGN1_BIT_MP_EXT_ONLY;
	} else if (pt_attrs & MATTR_CACHE_INNER_WB_nWA) {
		reg_val |= ARM_MMU_TTBR_IRGN0_BIT_MP_EXT_ONLY;
		reg_val |= ARM_MMU_TTBR_IRGN1_BIT_MP_EXT_ONLY;
	}

	__asm__ __volatile__("mcr p15, 0, %0, c2, c0, 0" : : "r"(reg_val));

	/* Write DACR -> all domains to client = 01b. */
	reg_val = ARM_MMU_DACR_ALL_DOMAINS_CLIENT;
	__asm__ __volatile__("mcr p15, 0, %0, c3, c0, 0" : : "r"(reg_val));

	invalidate_tlb_all();

	/* Enable the MMU and Cache in SCTLR */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0" : "=r"(reg_val));
	reg_val |= ARM_MMU_SCTLR_AFE_BIT;
	reg_val |= ARM_MMU_SCTLR_ICACHE_ENABLE_BIT;
	reg_val |= ARM_MMU_SCTLR_DCACHE_ENABLE_BIT;
	reg_val |= ARM_MMU_SCTLR_MMU_ENABLE_BIT;
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0" : : "r"(reg_val));

	return 0;
}

static int __arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	uint32_t va = (uint32_t)virt;
	uint32_t pa = (uint32_t)phys;
	uint32_t rem_size = (uint32_t)size;
	uint32_t conv_flags = MPERM_R;
	struct arm_mmu_perms_attrs perms_attrs;
	int key;

	if (size == 0) {
		LOG_ERR("Cannot map physical memory at 0x%08X: invalid "
			"zero size", (uint32_t)phys);
		return -EINVAL;
	}

	if (flags == 0) {
		LOG_ERR("Cannot map physical memory at 0x%08X: no memory "
			"configuration flags provided",	(uint32_t)phys);
		return -EINVAL;		
	}

	switch (flags & K_MEM_CACHE_MASK) {

	case K_MEM_CACHE_NONE:
	default:
		conv_flags |= MT_DEVICE;
		break;
	case K_MEM_CACHE_WB:
		conv_flags |= MT_NORMAL;
		conv_flags |= MATTR_SHARED;
		if (flags & K_MEM_PERM_RW) {
			conv_flags |= MATTR_CACHE_OUTER_WB_WA;
			conv_flags |= MATTR_CACHE_INNER_WB_WA;
		} else {
			conv_flags |= MATTR_CACHE_OUTER_WB_nWA;
			conv_flags |= MATTR_CACHE_INNER_WB_nWA;
		}
		break;
	case K_MEM_CACHE_WT:
		conv_flags |= MT_NORMAL;
		conv_flags |= MATTR_SHARED;
		conv_flags |= MATTR_CACHE_OUTER_WT_nWA;
		conv_flags |= MATTR_CACHE_INNER_WT_nWA;
		break;

	}

	if (flags & K_MEM_PERM_RW) {
		conv_flags |= MPERM_W;
	}
	
	perms_attrs = arm_mmu_convert_attr_flags(conv_flags);

	key = arch_irq_lock();

	while (rem_size > 0) {
		arm_mmu_l2_map_page(va, pa, perms_attrs);
		rem_size -= (rem_size >= KB(4)) ? KB(4) : rem_size;
		va += KB(4);
	}

	arch_irq_unlock(key);

	return 0;
}

void arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	int ret = __arch_mem_map(virt, phys, size, flags);

	if (ret) {
		LOG_ERR("__arch_mem_map() returned %d", ret);
		k_panic();
	} else {
		invalidate_tlb_all();
	}
}

static int __arch_mem_unmap(void *addr, size_t size)
{
	uint32_t va = (uint32_t)addr;
	uint32_t rem_size = (uint32_t)size;
	int key;

	if (addr == NULL) {
		LOG_ERR("Cannot unmap virtual memory: invalid NULL pointer");
		return -EINVAL;
	}

	if (size == 0) {
		LOG_ERR("Cannot unmap virtual memory at 0x%08X: invalid "
			"zero size", (uint32_t)addr);
		return -EINVAL;
	}

	key = arch_irq_lock();

	while (rem_size > 0) {
		arm_mmu_l2_unmap_page(va);
		rem_size -= (rem_size >= KB(4)) ? KB(4) : rem_size;
		va += KB(4);
	}

	arch_irq_unlock(key);

	return 0;
}

void arch_mem_unmap(void *addr, size_t size)
{
	int ret = __arch_mem_unmap(addr, size);

	if (ret) {
		LOG_ERR("__arch_mem_unmap() returned %d", ret);
	} else {
		invalidate_tlb_all();
	}
}

int arch_page_phys_get(void *virt, uintptr_t *phys)
{
	uint32_t l1_index = ((uint32_t)virt >> ARM_MMU_PTE_L1_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L1_INDEX_MASK;
	uint32_t l2_index = ((uint32_t)virt >> ARM_MMU_PTE_L2_INDEX_PA_SHIFT) &
			    ARM_MMU_PTE_L2_INDEX_MASK;
	struct arm_mmu_l2_page_table *l2_page_table;

	uint32_t pa_resolved = 0;
	uint32_t l2_pt_resolved;

	int rc = 0;
	int key;

	key = arch_irq_lock();

	if (l1_page_table.entries[l1_index].undefined.id == ARM_MMU_PTE_ID_SECTION) {
		/*
		 * If the virtual address points to a level 1 PTE whose ID bits
		 * identify it as a 1 MB section entry rather than a level 2 PT
		 * entry, the given VA belongs to a memory region used by the
		 * Zephyr image itself - it is only for those static regions that
		 * L1 Section entries are used to save L2 tables if a sufficient-
		 * ly large block of memory is specified. The memory regions be-
		 * longing to the Zephyr image are identity mapped -> just return
		 * the value of the VA as the value of the PA.
		 */
		pa_resolved = (uint32_t)virt;
	} else if (l1_page_table.entries[l1_index].undefined.id == ARM_MMU_PTE_ID_L2_PT) {
		/*
		 * The VA points to a level 1 PTE which re-directs to a level 2
		 * PT. -> Assemble the level 2 PT pointer and resolve the PA for
		 * the specified VA from there.
		 */
		l2_pt_resolved =
			l1_page_table.entries[l1_index].l2_page_table_ref.l2_page_table_address;
		l2_pt_resolved <<= ARM_MMU_PT_L2_ADDR_SHIFT;
		l2_page_table = (struct arm_mmu_l2_page_table *)l2_pt_resolved;

		pa_resolved = l2_page_table->entries[l2_index].l2_page_4k.pa_base;
		pa_resolved <<= ARM_MMU_PTE_L2_SMALL_PAGE_ADDR_SHIFT;
		pa_resolved |= ((uint32_t)virt & ARM_MMU_ADDR_BELOW_PAGE_GRAN_MASK);
	} else {
		/* The level 1 PTE is invalid -> the specified VA is not mapped */
		rc = -EFAULT;
	}

	arch_irq_unlock(key);

	if (phys) {
		*phys = (uintptr_t)pa_resolved;
	}
	return rc;
}

static void invalidate_tlb_all(void)
{
	uint32_t reg_val = 0; /* 0 = opc2 = invalidate entire TLB */
	__asm__ __volatile__("mcr p15, 0, %0, c8, c7, 0" : : "r"(reg_val));
	__asm__ __volatile__("dsb sy");
	__asm__ __volatile__("isb");
}
