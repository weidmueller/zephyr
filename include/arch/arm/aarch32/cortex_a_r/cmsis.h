/*
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief CMSIS interface file
 *
 * This header contains the interface to the ARM CMSIS Core headers.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_CORTEX_A_R_CMSIS_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_CORTEX_A_R_CMSIS_H_

#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CP10 Access Bits */
#define CPACR_CP10_Pos          20U
#define CPACR_CP10_Msk          (3UL << CPACR_CP10_Pos)
#define CPACR_CP10_NO_ACCESS    (0UL << CPACR_CP10_Pos)
#define CPACR_CP10_PRIV_ACCESS  (1UL << CPACR_CP10_Pos)
#define CPACR_CP10_RESERVED     (2UL << CPACR_CP10_Pos)
#define CPACR_CP10_FULL_ACCESS  (3UL << CPACR_CP10_Pos)

/* CP11 Access Bits */
#define CPACR_CP11_Pos          22U
#define CPACR_CP11_Msk          (3UL << CPACR_CP11_Pos)
#define CPACR_CP11_NO_ACCESS    (0UL << CPACR_CP11_Pos)
#define CPACR_CP11_PRIV_ACCESS  (1UL << CPACR_CP11_Pos)
#define CPACR_CP11_RESERVED     (2UL << CPACR_CP11_Pos)
#define CPACR_CP11_FULL_ACCESS  (3UL << CPACR_CP11_Pos)

/* FPEXC Bits - derived from CMSIS-Core (Cortex-A) FPEXC documentation */
#define FPEXC_EX_Pos            31U
#define FPEXC_EX_Msk            (1UL << FPEXC_EX_Pos)
#define FPEXC_EN_Pos            30U
#define FPEXC_EN_Msk            (1UL << FPEXC_EN_Pos)

/* FPSCR Bits - derived from CMSIS-Core (Cortex-A) FPSCR documentation */
#define FPSCR_N_Pos             31U
#define FPSCR_N_Msk             (1UL << FPSCR_N_Pos)
#define FPSCR_Z_Pos             30U
#define FPSCR_Z_Msk             (1UL << FPSCR_Z_Pos)
#define FPSCR_C_Pos             29U
#define FPSCR_C_Msk             (1UL << FPSCR_C_Pos)
#define FPSCR_V_Pos             28U
#define FPSCR_V_Msk             (1UL << FPSCR_V_Pos)
#define FPSCR_QC_Pos            27U
#define FPSCR_QC_Msk            (1UL << FPSCR_QC_Pos)
#define FPSCR_AHP_Pos           26U
#define FPSCR_AHP_Msk           (1UL << FPSCR_AHP_Pos)
#define FPSCR_DN_Pos            25U
#define FPSCR_DN_Msk            (1UL << FPSCR_DN_Pos)
#define FPSCR_FZ_Pos            24U
#define FPSCR_FZ_Msk            (1UL << FPSCR_FZ_Pos)
#define FPSCR_Mode_Pos          22U
#define FPSCR_Mode_Msk          (3UL << FPSCR_Mode_Pos)
#define FPSCR_Mode_NEAREST      (0UL << FPSCR_Mode_Pos)
#define FPSCR_Mode_PLUSINF      (1UL << FPSCR_Mode_Pos)
#define FPSCR_Mode_MINUSINF     (2UL << FPSCR_Mode_Pos)
#define FPSCR_Mode_ZERO         (3UL << FPSCR_Mode_Pos)
#define FPSCR_Stride_Pos        20U
#define FPSCR_Stride_Msk        (3UL << FPSCR_Stride_Pos)
#define FPSCR_Len_Pos           16U
#define FPSCR_Len_Msk           (7UL << FPSCR_Len_Pos)
#define FPSCR_IDE_Pos           15U
#define FPSCR_IDE_Msk           (1UL << FPSCR_IDE_Pos)
#define FPSCR_IXE_Pos           12U
#define FPSCR_IXE_Msk           (1UL << FPSCR_IXE_Pos)
#define FPSCR_UFE_Pos           11U
#define FPSCR_UFE_Msk           (1UL << FPSCR_UFE_Pos)
#define FPSCR_OFE_Pos           10U
#define FPSCR_OFE_Msk           (1UL << FPSCR_OFE_Pos)
#define FPSCR_DZE_Pos           9U
#define FPSCR_DZE_Msk           (1UL << FPSCR_DZE_Pos)
#define FPSCR_IOE_Pos           8U
#define FPSCR_IOE_Msk           (1UL << FPSCR_IOE_Pos)
#define FPSCR_IDC_Pos           7U
#define FPSCR_IDC_Msk           (1UL << FPSCR_IDC_Pos)
#define FPSCR_IXC_Pos           4U
#define FPSCR_IXC_Msk           (1UL << FPSCR_IXC_Pos)
#define FPSCR_UFC_Pos           3U
#define FPSCR_UFC_Msk           (1UL << FPSCR_UFC_Pos)
#define FPSCR_OFC_Pos           2U
#define FPSCR_OFC_Msk           (1UL << FPSCR_OFC_Pos)
#define FPSCR_DZC_Pos           1U
#define FPSCR_DZC_Msk           (1UL << FPSCR_DZC_Pos)
#define FPSCR_IOC_Pos           0U
#define FPSCR_IOC_Msk           (1UL << FPSCR_IOC_Pos)

#ifndef __CR_REV
#define __CR_REV                0U
#endif

#ifndef __CA_REV
#define __CA_REV                0U
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT           CONFIG_CPU_HAS_FPU
#endif

#ifdef __cplusplus
}
#endif

#if defined(CONFIG_CPU_CORTEX_R4)
#include <core_cr4.h>
#elif defined(CONFIG_CPU_CORTEX_R5)
#include <core_cr5.h>
#elif defined(CONFIG_CPU_CORTEX_A9)
#include <core_ca9.h>
#else
#error "Unknown device"
#endif

#include <arch/arm/aarch32/cortex_a_r/cmsis_ext.h>

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_CORTEX_A_R_CMSIS_H_ */
