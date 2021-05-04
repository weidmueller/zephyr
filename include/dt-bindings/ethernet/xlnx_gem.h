/*
 * Copyright (c) 2021, Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ETHERNET_XLNX_GEM_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ETHERNET_XLNX_GEM_H_

/* MDC divider values */
/* The following values are supported by both the Zynq-7000 and the ZynqMP */
#define XLNX_GEM_MDC_DIVIDER_8   0 /* cpu_1x or LPD_LSBUS_CLK     <  20 MHz */
#define XLNX_GEM_MDC_DIVIDER_16  1 /* cpu_1x or LPD_LSBUS_CLK  20 -  40 MHz */
#define XLNX_GEM_MDC_DIVIDER_32  2 /* cpu_1x or LPD_LSBUS_CLK  40 -  80 MHz */
/* According to the ZynqMP's gem.network_config register documentation,
 * divider /32 is to be used for a 100 MHz LPD LSBUS clock. */
/* The following values are supported by the Zynq-7000 only */
#define XLNX_GEM_MDC_DIVIDER_48  3 /* cpu_1x                   80 - 120 MHz */
#define XLNX_GEM_MDC_DIVIDER_64  4 /* cpu_1x                  120 - 160 MHz */
#define XLNX_GEM_MDC_DIVIDER_96  5 /* cpu_1x                  160 - 240 MHz */
#define XLNX_GEM_MDC_DIVIDER_128 6 /* cpu_1x                  240 - 320 MHz */
#define XLNX_GEM_MDC_DIVIDER_224 7 /* cpu_1x                  320 - 540 MHz */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ETHERNET_XLNX_GEM_H_ */