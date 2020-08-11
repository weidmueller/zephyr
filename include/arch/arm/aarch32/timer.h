/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_TIMER_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_TIMER_H_

#ifdef CONFIG_ARM_ARCH_TIMER

#ifndef _ASMLANGUAGE

#include <drivers/timer/arm_arch_timer.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_ARCH_TIMER_IRQ		ARM_TIMER_VIRTUAL_IRQ
#define ARM_ARCH_TIMER_PRIO		ARM_TIMER_VIRTUAL_PRIO
#define ARM_ARCH_TIMER_FLAGS		ARM_TIMER_VIRTUAL_FLAGS

#define CNTV_CTL_ENABLE			(1 << 0)
#define COMP_CTL_ENABLE			(1 << 1)
#define IRQ_CTL_ENABLE			(1 << 2)

#define COUNTVAL_LOW_REG_OFFSET		0x00
#define COUNTVAL_HIGH_REG_OFFSET	0x04
#define CONTROL_REG_OFFSET		0x08
#define INT_STATUS_REG_OFFSET		0x0C
#define COMPVAL_LOW_REG_OFFSET		0x10
#define COMPVAL_HIGH_REG_OFFSET		0x14
#define AUTO_INCR_REG_OFFSET		0x18

static ALWAYS_INLINE void arm_arch_timer_set_compare(uint64_t val)
{
	uint32_t high     = (uint32_t)(val >> 32);
	uint32_t low      = (uint32_t)val;
	uint32_t cntv_ctl = 0;

	/* Compare register update procedure as described in the Zynq-7000
	 * TRM, Appendix B, p. 1452 :
	 * 1. Clear the Comp. Enable bit in the Timer Control Register.
	 * 2. Write the lower 32-bit Comparator Value Register.
	 * 3. Write the upper 32-bit Comparator Value Register.
	 * 4. Set the Comparator Enable bit and the IRQ enable bit. */

	cntv_ctl = sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ CONTROL_REG_OFFSET);
	sys_write32((cntv_ctl & ~COMP_CTL_ENABLE), 
		DT_REG_ADDR(DT_INST(0, arm_arm_timer)) + CONTROL_REG_OFFSET);

	sys_write32(low,  DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ COMPVAL_LOW_REG_OFFSET);
	sys_write32(high, DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ COMPVAL_HIGH_REG_OFFSET);

	sys_write32((cntv_ctl | COMP_CTL_ENABLE),
		DT_REG_ADDR(DT_INST(0, arm_arm_timer)) + CONTROL_REG_OFFSET);
}

#if defined(CONFIG_ARM_ARCH_TIMER_ERRATUM_740657) || defined(CONFIG_QEMU_TARGET)

/* 
 * R/W access to the event flag register is required for the timer errata
 * 740657 workaround -> comp. ISR implementation in arm_arch_timer.c.
 * This functionality is not present in the aarch64 implementation of the
 * ARM global timer access functions. 
 * 
 * comp. ARM Cortex-A9 processors Software Developers Errata Notice,
 * ARM document ID032315.
 */

static ALWAYS_INLINE uint8_t arm_arch_timer_get_int_status(void)
{
	return (uint8_t)(sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ INT_STATUS_REG_OFFSET) & 0x1);
}

static ALWAYS_INLINE void arm_arch_timer_clear_int_status(void)
{
	sys_write32(0x1, DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ INT_STATUS_REG_OFFSET);
}

#endif /* CONFIG_ARM_ARCH_TIMER_ERRATUM_740657 || CONFIG_QEMU_TARGET */

static ALWAYS_INLINE void arm_arch_timer_enable(unsigned char enable)
{
	uint32_t cntv_ctl = sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ CONTROL_REG_OFFSET);

	if (enable) {
		cntv_ctl |=  (CNTV_CTL_ENABLE | IRQ_CTL_ENABLE);
	} else {
		cntv_ctl &= (~CNTV_CTL_ENABLE | IRQ_CTL_ENABLE);
	}

	sys_write32(cntv_ctl, DT_REG_ADDR(DT_INST(0, arm_arm_timer))
		+ CONTROL_REG_OFFSET);
}

static ALWAYS_INLINE uint64_t arm_arch_timer_count(void)
{
	uint32_t high_first  = 0;
	uint32_t high_second = 1;
	uint32_t low;

	/* Counter register read procedure as described in the Zynq-7000
	 * TRM, Appendix B, p. 1449 :
	 * 1. Read the upper 32-bit timer counter register
	 * 2. Read the lower 32-bit timer counter register
	 * 3. Read the upper 32-bit timer counter register again. 
	 * If the value is different to the 32-bit upper value read previously, 
	 * go back to step 2. Otherwise the 64-bit timer counter value is correct. */

	while (high_first != high_second) {
		high_first  = sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
			+ COUNTVAL_HIGH_REG_OFFSET);
		low         = sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
			+ COUNTVAL_LOW_REG_OFFSET);
		high_second = sys_read32(DT_REG_ADDR(DT_INST(0, arm_arm_timer))
			+ COUNTVAL_HIGH_REG_OFFSET);
	}

	return (((uint64_t)high_first << 32) | (uint64_t)low);
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* CONFIG_ARM_ARCH_TIMER */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_TIMER_H_ */
