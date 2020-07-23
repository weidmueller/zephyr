/*
 * Copyright (c) 2013-2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions (ARM)
 *
 * This file contains private kernel structures definitions and various
 * other definitions for the ARM Cortex-A, Cortex-M and Cortex-R processor
 * architecture families.
 *
 * This file is also included by assembly language files which must #define
 * _ASMLANGUAGE before including this header file.  Note that kernel
 * assembly source files obtains structure offset values via "absolute symbols"
 * in the offsets.o module.
 */

#ifndef ZEPHYR_ARCH_ARM_INCLUDE_KERNEL_ARCH_DATA_H_
#define ZEPHYR_ARCH_ARM_INCLUDE_KERNEL_ARCH_DATA_H_

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>

#if defined(CONFIG_CPU_CORTEX_M)
#include <aarch32/cortex_m/stack.h>
#include <aarch32/cortex_m/exc.h>
#elif defined(CONFIG_CPU_CORTEX_R) \
	|| (defined(CONFIG_CPU_CORTEX_A) && !defined(CONFIG_ARM64))
#include <aarch32/cortex_a_r/stack.h>
#include <aarch32/cortex_a_r/exc.h>
#elif defined(CONFIG_CPU_CORTEX_A) && defined(CONFIG_ARM64)
#include <aarch64/exc.h>
#endif

#ifndef _ASMLANGUAGE
#include <kernel.h>
#include <zephyr/types.h>
#include <sys/dlist.h>
#include <sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __esf _esf_t;
typedef struct __basic_sf _basic_sf_t;

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_ARM_INCLUDE_KERNEL_ARCH_DATA_H_ */
