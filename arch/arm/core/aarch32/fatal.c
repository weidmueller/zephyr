/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Kernel fatal error handler for ARM Cortex-M and Cortex-R
 *
 * This module provides the z_arm_fatal_error() routine for ARM Cortex-M
 * and Cortex-R CPUs.
 */

#include <kernel.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(os);

static void esf_dump(const z_arch_esf_t *esf)
{
	LOG_ERR("r0/a1:  0x%08x  r1/a2:  0x%08x  r2/a3:  0x%08x",
		esf->basic.a1, esf->basic.a2, esf->basic.a3);
	LOG_ERR("r3/a4:  0x%08x r12/ip:  0x%08x r14/lr:  0x%08x",
		esf->basic.a4, esf->basic.ip, esf->basic.lr);
	LOG_ERR(" xpsr:  0x%08x", esf->basic.xpsr);

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
#if defined(CONFIG_ARMV7_A_FP) || defined(CONFIG_ARMV7_R_FP)

	/* Registers of the VFPv3(-D32/-D16) FPUs, Cortex-A / Cortex-R */
	uint32_t *d_regs_as_s = (uint32_t*)&(esf->d[0]);
	for (int i = 0; i < 16; i++) {
		LOG_ERR("d[%2d]: 0x%08x%08x -> s[%2d]: 0x%08x s[%2d]: 0x%08x", 
		i, 
		d_regs_as_s[(i * 2)], 
		d_regs_as_s[(i * 2) + 1],
		(i * 2), d_regs_as_s[(i * 2)],
		((i * 2) + 1), d_regs_as_s[(i * 2) + 1]);
	}
#if defined(CONFIG_ARMV7_A_FP)
	/* D16..D31 are not available in the VFPv3-D16 implementation which
	 * is optional in the Cortex-R CPUs */
	for (int i = 16; i < 32; i++) {
		LOG_ERR("d[%2d]: 0x%08x%08x -> s[%2d]: 0x%08x s[%2d]: 0x%08x", 
		i, 
		d_regs_as_s[(i * 2)], 
		d_regs_as_s[(i * 2) + 1],
		(i * 2), d_regs_as_s[(i * 2)],
		((i * 2) + 1), d_regs_as_s[(i * 2) + 1]);
	}
#endif /* CONFIG_ARMV7_A_FP */
	LOG_ERR("fpexc:  0x%08x", esf->fpexc);
	LOG_ERR("fpscr:  0x%08x", esf->fpscr);

#else /* !(CONFIG_ARMV7_A_FP || CONFIG_ARMV7_R_FP) */

	/* ARMv7 Cortex-M FPU registers (VFPv4-SP) */
	for (int i = 0; i < 16; i += 4) {
		LOG_ERR("s[%2d]:  0x%08x  s[%2d]:  0x%08x"
			"  s[%2d]:  0x%08x  s[%2d]:  0x%08x",
			i, (uint32_t)esf->s[i],
			i + 1, (uint32_t)esf->s[i + 1],
			i + 2, (uint32_t)esf->s[i + 2],
			i + 3, (uint32_t)esf->s[i + 3]);
	}
	LOG_ERR("fpscr:  0x%08x", esf->fpscr);

#endif /* CONFIG_ARMV7_A_FP || CONFIG_ARMV7_R_FP */
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

	LOG_ERR("Faulting instruction address (r15/pc): 0x%08x",
		esf->basic.pc);
}

void z_arm_fatal_error(unsigned int reason, const z_arch_esf_t *esf)
{

	if (esf != NULL) {
		esf_dump(esf);
	}
	z_fatal_error(reason, esf);
}

/**
 * @brief Handle a software-generated fatal exception
 * (e.g. kernel oops, panic, etc.).
 *
 * Notes:
 * - the function is invoked in SVC Handler
 * - if triggered from nPRIV mode, only oops and stack fail error reasons
 *   may be propagated to the fault handling process.
 * - We expect the supplied exception stack frame to always be a valid
 *   frame. That is because, if the ESF cannot be stacked during an SVC,
 *   a processor fault (e.g. stacking error) will be generated, and the
 *   fault handler will executed insted of the SVC.
 *
 * @param esf exception frame
 */
void z_do_kernel_oops(const z_arch_esf_t *esf)
{
	/* Stacked R0 holds the exception reason. */
	unsigned int reason = esf->basic.r0;

#if defined(CONFIG_USERSPACE)
	if ((__get_CONTROL() & CONTROL_nPRIV_Msk) == CONTROL_nPRIV_Msk) {
		/*
		 * Exception triggered from nPRIV mode.
		 *
		 * User mode is only allowed to induce oopses and stack check
		 * failures via software-triggered system fatal exceptions.
		 */
		if (!((esf->basic.r0 == K_ERR_KERNEL_OOPS) ||
			(esf->basic.r0 == K_ERR_STACK_CHK_FAIL))) {

			reason = K_ERR_KERNEL_OOPS;
		}
	}

#endif /* CONFIG_USERSPACE */
	z_arm_fatal_error(reason, esf);
}

FUNC_NORETURN void arch_syscall_oops(void *ssf_ptr)
{
	uint32_t *ssf_contents = ssf_ptr;
	z_arch_esf_t oops_esf = { 0 };

	/* TODO: Copy the rest of the register set out of ssf_ptr */
	oops_esf.basic.pc = ssf_contents[3];

	z_arm_fatal_error(K_ERR_KERNEL_OOPS, &oops_esf);
	CODE_UNREACHABLE;
}
