/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <assert.h>

#include "riscv_internal.h"

#include "sbi_internal.h"

/****************************************************************************
 * Preprocecssor definitions
 ****************************************************************************/

#define NAPOT_RWX   (PMPCFG_A_NAPOT | PMPCFG_RWX_MASK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void __trap_vec_tmp(void) noreturn_function;
static void __trap_vec_tmp(void)
{
  for (; ; )
    {
      asm("WFI");
    }
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern void __start_s(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sbi_start(void)
{
  uintptr_t reg;
  uintptr_t hartid;

  /* Read hart ID */

  hartid = READ_CSR(CSR_MHARTID);

  /* Set mscratch, mtimer */

  sbi_mscratch_assign(hartid);
  sbi_init_mtimer(MTIMER_TIME_BASE, MTIMER_CMP_BASE);

  /* Setup system to enter S-mode */

  reg  =  READ_CSR(CSR_MSTATUS);
  reg &= ~MSTATUS_MPPM; /* Clear MPP */
  reg &= ~MSTATUS_MPIE; /* Clear MPIE */
  reg &= ~MSTATUS_TW;   /* Do not trap WFI */
  reg &= ~MSTATUS_TSR;  /* Do not trap sret */
  reg &= ~MSTATUS_TVM;  /* Allow satp writes from S-mode */
  reg |=  MSTATUS_SUM;  /* Allow supervisor to access user memory */
  reg |=  MSTATUS_MPPS; /* Set next context = supervisor */

  /* Setup next context */

  WRITE_CSR(CSR_MSTATUS, reg);

  /* Setup a temporary S-mode interrupt vector */

  WRITE_CSR(CSR_STVEC, __trap_vec_tmp);

  /* Delegate interrupts */

  reg = (MIP_SSIP | MIP_STIP | MIP_SEIP);
  WRITE_CSR(CSR_MIDELEG, reg);

  /* Delegate exceptions (all of them) */

  reg = ((1 << RISCV_IRQ_IAMISALIGNED) |
         (1 << RISCV_IRQ_INSTRUCTIONPF) |
         (1 << RISCV_IRQ_LOADPF) |
         (1 << RISCV_IRQ_STOREPF) |
         (1 << RISCV_IRQ_ECALLU));
  WRITE_CSR(CSR_MEDELEG, reg);

  /* Enable access to all counters for S- and U-mode */

  WRITE_CSR(CSR_MCOUNTEREN, UINT32_C(~0));
  WRITE_CSR(CSR_SCOUNTEREN, UINT32_C(~0));

  /* Set program counter to __start_s */

  WRITE_CSR(CSR_MEPC, __start_s);

#ifdef CONFIG_NUTTSBI_LATE_INIT
  /* Do device specific handling */

  sbi_late_initialize();
#else
  /* Open everything for PMP, may fail if no empty entry left */

  DEBUGASSERT(riscv_append_pmp_region(NAPOT_RWX, 0, 0) == 0);
#endif

  /* Then jump to the S-mode start function */

  register long r0 asm("a0") = (long) hartid;
  __asm__ __volatile__("mret" : : "r"(r0));

  PANIC();
}
