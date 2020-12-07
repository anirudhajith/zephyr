/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the Shakti processors
 */

#ifndef __RISCV_SHAKTI_SOC_H_
#define __RISCV_SHAKTI_SOC_H_

#include <soc_common.h>
#include <devicetree.h>

/* Timer configuration */
#define RISCV_MTIME_BASE             0x0200BFF8
#define RISCV_MTIMECMP_BASE          0x02004000


/* PINMUX Configuration */
#define SHAKTI_PINMUX_0_BASE_ADDR     (DT_REG_ADDR(DT_INST(0, shakti_gpio0)) + 0x38)

/* PINMUX IO Hardware Functions */
#define SHAKTI_PINMUX_IOF0            0x00
#define SHAKTI_PINMUX_IOF1            0x01

/* PINMUX MAX PINS */
#define SHAKTI_PINMUX_PINS            32

#if 0
/* Clock controller. */
#define PRCI_BASE_ADDR               0x10008000


/* Always ON Domain */
#define SIFIVE_PMUIE		     0x10000140
#define SIFIVE_PMUCAUSE		     0x10000144
#define SIFIVE_PMUSLEEP		     0x10000148
#define SIFIVE_PMUKEY		     0x1000014C
#define SIFIVE_SLEEP_KEY_VAL	     0x0051F15E

#define SIFIVE_BACKUP_REG_BASE	     0x10000080
#endif

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(CONFIG_SRAM_SIZE)

#endif /* __RISCV_SHAKTI_SOC_H_ */