/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Full C support initialization
 *
 *
 * Initialization of full C support: zero the .bss, copy the .data if XIP,
 * call z_cstart().
 *
 * Stack is available in this module, but not the global data/bss until their
 * initialization is performed.
 */

#include <kernel.h>
#include <kernel_internal.h>
#include <linker/linker-defs.h>

#if defined(CONFIG_ARMV7_R) || defined(CONFIG_ARMV7_A)
#include <aarch32/cortex_a_r/stack.h>
#endif

#if defined(__GNUC__)
/*
 * GCC can detect if memcpy is passed a NULL argument, however one of
 * the cases of relocate_vector_table() it is valid to pass NULL, so we
 * suppress the warning for this case.  We need to do this before
 * string.h is included to get the declaration of memcpy.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif

#include <string.h>

#if defined(CONFIG_SW_VECTOR_RELAY) || defined(CONFIG_SW_VECTOR_RELAY_CLIENT)
Z_GENERIC_SECTION(.vt_pointer_section) __attribute__((used))
void *_vector_table_pointer;
#endif

#ifdef CONFIG_CPU_CORTEX_M_HAS_VTOR

#ifdef CONFIG_XIP
#define VECTOR_ADDRESS ((uintptr_t)_vector_start)
#else
#define VECTOR_ADDRESS CONFIG_SRAM_BASE_ADDRESS
#endif
static inline void relocate_vector_table(void)
{
	SCB->VTOR = VECTOR_ADDRESS & SCB_VTOR_TBLOFF_Msk;
	__DSB();
	__ISB();
}

#else

#define VECTOR_ADDRESS 0

void __weak relocate_vector_table(void)
{
#if defined(CONFIG_XIP) && (CONFIG_FLASH_BASE_ADDRESS != 0) || \
    !defined(CONFIG_XIP) && (CONFIG_SRAM_BASE_ADDRESS != 0)
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
	(void)memcpy(VECTOR_ADDRESS, _vector_start, vector_size);
#elif defined(CONFIG_SW_VECTOR_RELAY) || defined(CONFIG_SW_VECTOR_RELAY_CLIENT)
	_vector_table_pointer = _vector_start;
#endif
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif /* CONFIG_CPU_CORTEX_M_HAS_VTOR */

#if defined(CONFIG_CPU_HAS_FPU) && !defined(CONFIG_ARMV7_A_FP_VFPV3_D16)
static inline void z_arm_floating_point_init(void)
{
	/*
	 * Upon reset, the Co-Processor Access Control Register is, normally,
	 * 0x00000000. However, it might be left un-cleared by firmware running
	 * before Zephyr boot.
	 */
	SCB->CPACR &= (~(CPACR_CP10_Msk | CPACR_CP11_Msk));

#if defined(CONFIG_FPU)
	/*
	 * Enable CP10 and CP11 Co-Processors to enable access to floating
	 * point registers.
	 */
#if defined(CONFIG_USERSPACE)
	/* Full access */
	SCB->CPACR |= CPACR_CP10_FULL_ACCESS | CPACR_CP11_FULL_ACCESS;
#else
	/* Privileged access only */
	SCB->CPACR |= CPACR_CP10_PRIV_ACCESS | CPACR_CP11_PRIV_ACCESS;
#endif /* CONFIG_USERSPACE */
	/*
	 * Upon reset, the FPU Context Control Register is 0xC0000000
	 * (both Automatic and Lazy state preservation is enabled).
	 */
#if !defined(CONFIG_FPU_SHARING)
	/* Default mode is Unshared FP registers mode. We disable the
	 * automatic stacking of FP registers (automatic setting of
	 * FPCA bit in the CONTROL register), upon exception entries,
	 * as the FP registers are to be used by a single context (and
	 * the use of FP registers in ISRs is not supported). This
	 * configuration improves interrupt latency and decreases the
	 * stack memory requirement for the (single) thread that makes
	 * use of the FP co-processor.
	 */
	FPU->FPCCR &= (~(FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk));
#else
	/*
	 * Enable both automatic and lazy state preservation of the FP context.
	 * The FPCA bit of the CONTROL register will be automatically set, if
	 * the thread uses the floating point registers. Because of lazy state
	 * preservation the volatile FP registers will not be stacked upon
	 * exception entry, however, the required area in the stack frame will
	 * be reserved for them. This configuration improves interrupt latency.
	 * The registers will eventually be stacked when the thread is swapped
	 * out during context-switch.
	 */
	FPU->FPCCR = FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;
#endif /* CONFIG_FPU_SHARING */

	/* Make the side-effects of modifying the FPCCR be realized
	 * immediately.
	 */
	__DSB();
	__ISB();

	/* Initialize the Floating Point Status and Control Register. */
	__set_FPSCR(0);

	/*
	 * Note:
	 * The use of the FP register bank is enabled, however the FP context
	 * will be activated (FPCA bit on the CONTROL register) in the presence
	 * of floating point instructions.
	 */

#endif /* CONFIG_FPU */

	/*
	 * Upon reset, the CONTROL.FPCA bit is, normally, cleared. However,
	 * it might be left un-cleared by firmware running before Zephyr boot.
	 * We must clear this bit to prevent errors in exception unstacking.
	 *
	 * Note:
	 * In Sharing FP Registers mode CONTROL.FPCA is cleared before switching
	 * to main, so it may be skipped here (saving few boot cycles).
	 */
#if !defined(CONFIG_FPU) || !defined(CONFIG_FPU_SHARING)
	__set_CONTROL(__get_CONTROL() & (~(CONTROL_FPCA_Msk)));
#endif
}
#elif defined(CONFIG_CPU_HAS_FPU) && defined(CONFIG_ARMV7_A_FP_VFPV3_D16)
static inline void z_arm_floating_point_init(void)
{
	#ifdef CONFIG_FPU

	uint32_t reg_val = 0;

	/*
	 * TODO: use CMSIS for control register access as seen above for 
	 * Cortex-M. 
	 * TODO: configure NSACR once non-secure mode is supported.
	 */

	/*
	 * CPACR : Coprocessor Access Control Register -> CP15 1/0/2
	 * comp. ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition, 
	 * chap. B4.1.40.
	 *
	 * Must be accessed at >= PL1!
	 * [23..22] = CP11 access control bits,
	 * [21..20] = CP10 access control bits.
	 * 11b = Full access as defined for the respective CP,
	 * 10b = UNDEFINED,
	 * 01b = Access at PL1 only,
	 * 00b = No access.
	 */

	__asm__ __volatile__ ("mrc p15,0,%0,c1,c0,2" : "=r"(reg_val));
	reg_val |= ((3 << 22) | (3 << 20)); /* Enable full access to CP10, CP11 */
	__asm__ __volatile__ ("mcr p15,0,%0,c1,c0,2" : : "r"(reg_val));
	__asm__ __volatile__ ("isb");

	/*
	 * FPEXC: Floating-Point Exception Control register
	 * comp. ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition, 
	 * chap. B6.1.38 or Cortex-A9 Floating-Point Unit Technical Reference 
	 * Manual, chap. 2.5.3.
	 *
	 * Must be accessed at >= PL1!
	 * [31] EX bit = determines which registers comprise the current state
	 *               of the FPU. The effects of setting this bit to 1 are
	 *               subarchitecture defined. If EX=0, the following registers
	 *               contain the complete current state information of the FPU
	 *               and must therefore be saved during a context switch:
	 *               * D0-D15
	 *               * D16-D31 if implemented
	 *               * FPSCR
	 *               * FPEXC.
	 * [30] EN bit = Advanced SIMD/Floating Point Extensions enable bit.
	 * [29..00]    = Subarchitecture defined -> not relevant here.
	 */

	__asm__ __volatile__ ("vmrs %0, fpexc" : "=r"(reg_val));
	reg_val |= (1 << 30); /* Set the EN bit */
	__asm__ __volatile__ ("vmsr fpexc,%0" : : "r"(reg_val));
	__asm__ __volatile__ ("isb");

	/*
	 * The contents of the FPSCR register are written:
	 * - within arch_switch_to_main_thread() if register sharing is disabled,
	 * - within each call of arch_new_thread() if register sharing is enabled.
	 */

	#endif /* CONFIG_FPU */
}
#endif /* CONFIG_CPU_HAS_FPU && !CONFIG_ARMV7_A_FP_VFPV3_D16 */

extern FUNC_NORETURN void z_cstart(void);
/**
 *
 * @brief Prepare to and run C code
 *
 * This routine prepares for the execution of and runs C code.
 *
 * @return N/A
 */
void z_arm_prep_c(void)
{
	relocate_vector_table();
#if defined(CONFIG_CPU_HAS_FPU)
	z_arm_floating_point_init();
#endif
	z_bss_zero();
	z_data_copy();
#if (defined(CONFIG_ARMV7_R) && defined(CONFIG_INIT_STACKS)) \
	|| (defined(CONFIG_ARMV7_A) && defined(CONFIG_INIT_STACKS))
	z_arm_init_stacks();
#endif
	z_arm_interrupt_init();
	z_cstart();
	CODE_UNREACHABLE;
}
