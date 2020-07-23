/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/timer/arm_arch_timer.h>
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <arch/cpu.h>

#define CYC_PER_TICK	((uint64_t)sys_clock_hw_cycles_per_sec() \
			/ (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS	INT32_MAX
#define MIN_DELAY	(1000)

static struct k_spinlock lock;
static volatile uint64_t last_cycle;

static void arm_arch_timer_compare_isr(void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);

	#ifdef CONFIG_ARM_ARCH_TIMER_ERRATA_740657

	/* Workaround required for Cortex-A9 MPCore errata 740657
	 * comp. ARM Cortex-A9 processors Software Developers Errata Notice,
	 * ARM document ID032315. */

	if (!arm_arch_timer_get_int_status()) {
		/* If the event flag is not set, this is a spurious interrupt.
		 * DO NOT modify the compare register's value, DO NOT announce
		 * elapsed ticks! */
		k_spin_unlock(&lock, key);
		return;
	}

	#endif /* CONFIG_ARM_ARCH_TIMER_ERRATA_740657 */

	uint64_t curr_cycle = arm_arch_timer_count();
	uint32_t delta_ticks = (uint32_t)((curr_cycle - last_cycle) / CYC_PER_TICK);

	last_cycle += delta_ticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		uint64_t next_cycle = last_cycle + CYC_PER_TICK;

		if ((uint64_t)(next_cycle - curr_cycle) < MIN_DELAY) {
			next_cycle += CYC_PER_TICK;
		}
		arm_arch_timer_set_compare(next_cycle);
	}
	#ifdef CONFIG_ARM_ARCH_TIMER_ERRATA_740657
	else {
		/* In tickless mode, the compare register is normally not updated from
		* within the ISR. Yet, to work around the timer's errata, a new value
		* *must* be written while the interrupt is being processed, the event
		* flag *must* be cleared after the compare register is updated. */
		arm_arch_timer_set_compare(0xFFFFFFFFFFFFFFFFLLU);
	}
	
	arm_arch_timer_clear_int_status();

	#endif /* CONFIG_ARM_ARCH_TIMER_ERRATA_740657 */

	k_spin_unlock(&lock, key);

	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? delta_ticks : 1);
}

int z_clock_driver_init(struct device *device)
{
	ARG_UNUSED(device);

	IRQ_CONNECT(ARM_ARCH_TIMER_IRQ, ARM_ARCH_TIMER_PRIO,
		    arm_arch_timer_compare_isr, NULL, ARM_ARCH_TIMER_FLAGS);
	arm_arch_timer_set_compare(arm_arch_timer_count() + CYC_PER_TICK);
	arm_arch_timer_enable(true);
	irq_enable(ARM_ARCH_TIMER_IRQ);

	return 0;
}

void z_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)

	if (idle) {
		return;
	}

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : \
		MIN(MAX_TICKS,  MAX(ticks - 1,  0));

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t curr_cycle = arm_arch_timer_count();
	uint64_t req_cycle = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary */
	req_cycle += (curr_cycle - last_cycle) + (CYC_PER_TICK - 1);

	req_cycle = (req_cycle / CYC_PER_TICK) * CYC_PER_TICK;

	if ((req_cycle + last_cycle - curr_cycle) < MIN_DELAY) {
		req_cycle += CYC_PER_TICK;
	}

	arm_arch_timer_set_compare(req_cycle + last_cycle);
	k_spin_unlock(&lock, key);

#endif
}

uint32_t z_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = (uint32_t)((arm_arch_timer_count() - last_cycle)
		    / CYC_PER_TICK);

	k_spin_unlock(&lock, key);
	return ret;
}

uint32_t z_timer_cycle_get_32(void)
{
	return (uint32_t)arm_arch_timer_count();
}
