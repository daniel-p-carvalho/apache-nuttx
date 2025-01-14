/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_uart.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_UART_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_UART_CLK 40000000

#define BL808_UART_BASE(n) ((n == 0) ? BL808_UART0_BASE \
                            : (n == 1) ? BL808_UART1_BASE \
                            : (n == 2) ? BL808_UART2_BASE \
                            : BL808_UART3_BASE)

/* Register offsets *********************************************************/

#define BL808_UART_UTX_CONFIG_OFFSET          0x000000  /* utx_config */
#define BL808_UART_URX_CONFIG_OFFSET          0x000004  /* urx_config */
#define BL808_UART_BIT_PRD_OFFSET             0x000008  /* uart_bit_prd */
#define BL808_UART_DATA_CONFIG_OFFSET         0x00000c  /* data_config */
#define BL808_UART_UTX_IR_POSITION_OFFSET     0x000010  /* utx_ir_position */
#define BL808_UART_URX_IR_POSITION_OFFSET     0x000014  /* urx_ir_position */
#define BL808_UART_URX_RTO_TIMER_OFFSET       0x000018  /* urx_rto_timer */
#define BL808_UART_INT_STS_OFFSET             0x000020  /* UART interrupt status */
#define BL808_UART_INT_MASK_OFFSET            0x000024  /* UART interrupt mask */
#define BL808_UART_INT_CLEAR_OFFSET           0x000028  /* UART interrupt clear */
#define BL808_UART_INT_EN_OFFSET              0x00002c  /* UART interrupt enable */
#define BL808_UART_STATUS_OFFSET              0x000030  /* uart_status */
#define BL808_UART_STS_URX_ABR_PRD_OFFSET     0x000034  /* sts_urx_abr_prd */
#define BL808_UART_FIFO_CONFIG_0_OFFSET       0x000080  /* uart_fifo_config_0 */
#define BL808_UART_FIFO_CONFIG_1_OFFSET       0x000084  /* uart_fifo_config_1 */
#define BL808_UART_FIFO_WDATA_OFFSET          0x000088  /* uart_fifo_wdata */
#define BL808_UART_FIFO_RDATA_OFFSET          0x00008c  /* uart_fifo_rdata */

/* Register definitions *****************************************************/

#define BL808_UART_UTX_CONFIG(n)          (BL808_UART_BASE(n) + BL808_UART_UTX_CONFIG_OFFSET)
#define BL808_UART_URX_CONFIG(n)          (BL808_UART_BASE(n) + BL808_UART_URX_CONFIG_OFFSET)
#define BL808_UART_BIT_PRD(n)             (BL808_UART_BASE(n) + BL808_UART_BIT_PRD_OFFSET)
#define BL808_UART_DATA_CONFIG(n)         (BL808_UART_BASE(n) + BL808_UART_DATA_CONFIG_OFFSET)
#define BL808_UART_UTX_IR_POSITION(n)     (BL808_UART_BASE(n) + BL808_UART_UTX_IR_POSITION_OFFSET)
#define BL808_UART_URX_IR_POSITION(n)     (BL808_UART_BASE(n) + BL808_UART_URX_IR_POSITION_OFFSET)
#define BL808_UART_URX_RTO_TIMER(n)       (BL808_UART_BASE(n) + BL808_UART_URX_RTO_TIMER_OFFSET)
#define BL808_UART_INT_STS(n)             (BL808_UART_BASE(n) + BL808_UART_INT_STS_OFFSET)
#define BL808_UART_INT_MASK(n)            (BL808_UART_BASE(n) + BL808_UART_INT_MASK_OFFSET)
#define BL808_UART_INT_CLEAR(n)           (BL808_UART_BASE(n) + BL808_UART_INT_CLEAR_OFFSET)
#define BL808_UART_INT_EN(n)              (BL808_UART_BASE(n) + BL808_UART_INT_EN_OFFSET)
#define BL808_UART_STATUS(n)              (BL808_UART_BASE(n) + BL808_UART_STATUS_OFFSET)
#define BL808_UART_STS_URX_ABR_PRD(n)     (BL808_UART_BASE(n) + BL808_UART_STS_URX_ABR_PRD_OFFSET)
#define BL808_UART_FIFO_CONFIG_0(n)       (BL808_UART_BASE(n) + BL808_UART_FIFO_CONFIG_0_OFFSET)
#define BL808_UART_FIFO_CONFIG_1(n)       (BL808_UART_BASE(n) + BL808_UART_FIFO_CONFIG_1_OFFSET)
#define BL808_UART_FIFO_WDATA(n)          (BL808_UART_BASE(n) + BL808_UART_FIFO_WDATA_OFFSET)
#define BL808_UART_FIFO_RDATA(n)          (BL808_UART_BASE(n) + BL808_UART_FIFO_RDATA_OFFSET)

/* Register bit definitions *************************************************/

#define UART_UTX_CONFIG_CR_LEN_SHIFT                 (16)
#define UART_UTX_CONFIG_CR_LEN_MASK                  (0xffff << UART_UTX_CONFIG_CR_LEN_SHIFT)
#define UART_UTX_CONFIG_CR_BIT_CNT_P_SHIFT           (11)
#define UART_UTX_CONFIG_CR_BIT_CNT_P_MASK            (0x03 << UART_UTX_CONFIG_CR_BIT_CNT_P_SHIFT)
#define UART_UTX_CONFIG_CR_BIT_CNT_D_SHIFT           (8)
#define UART_UTX_CONFIG_CR_BIT_CNT_D_MASK            (0x07 << UART_UTX_CONFIG_CR_BIT_CNT_D_SHIFT)
#define UART_UTX_CONFIG_CR_IR_INV                    (1 << 7)
#define UART_UTX_CONFIG_CR_IR_EN                     (1 << 6)
#define UART_UTX_CONFIG_CR_PRT_SEL                   (1 << 5)
#define UART_UTX_CONFIG_CR_PRT_EN                    (1 << 4)
#define UART_UTX_CONFIG_CR_FRM_EN                    (1 << 2)
#define UART_UTX_CONFIG_CR_CTS_EN                    (1 << 1)
#define UART_UTX_CONFIG_CR_EN                        (1 << 0)

#define UART_URX_CONFIG_CR_LEN_SHIFT                 (16)
#define UART_URX_CONFIG_CR_LEN_MASK                  (0xffff << UART_URX_CONFIG_CR_LEN_SHIFT)
#define UART_URX_CONFIG_CR_DEG_CNT_SHIFT             (12)
#define UART_URX_CONFIG_CR_DEG_CNT_MASK              (0x0f << UART_URX_CONFIG_CR_DEG_CNT_SHIFT)
#define UART_URX_CONFIG_CR_DEG_EN                    (1 << 11)
#define UART_URX_CONFIG_CR_BIT_CNT_D_SHIFT           (8)
#define UART_URX_CONFIG_CR_BIT_CNT_D_MASK            (0x07 << UART_URX_CONFIG_CR_BIT_CNT_D_SHIFT)
#define UART_URX_CONFIG_CR_IR_INV                    (1 << 7)
#define UART_URX_CONFIG_CR_IR_EN                     (1 << 6)
#define UART_URX_CONFIG_CR_PRT_SEL                   (1 << 5)
#define UART_URX_CONFIG_CR_PRT_EN                    (1 << 4)
#define UART_URX_CONFIG_CR_ABR_EN                    (1 << 3)
#define UART_URX_CONFIG_CR_RTS_SW_VAL                (1 << 2)
#define UART_URX_CONFIG_CR_RTS_SW_MODE               (1 << 1)
#define UART_URX_CONFIG_CR_EN                        (1 << 0)

#define UART_BIT_PRD_CR_URX_BIT_SHIFT                (16)
#define UART_BIT_PRD_CR_URX_BIT_MASK                 (0xffff << UART_BIT_PRD_CR_URX_BIT_SHIFT)
#define UART_BIT_PRD_CR_UTX_BIT_MASK                 (0xffff)

#define UART_DATA_CONFIG_CR_UART_BIT_INV             (1 << 0)

#define UART_UTX_IR_POSITION_CR_IR_POS_P_SHIFT       (16)
#define UART_UTX_IR_POSITION_CR_IR_POS_P_MASK        (0xffff << UART_UTX_IR_POSITION_CR_IR_POS_P_SHIFT)
#define UART_UTX_IR_POSITION_CR_IR_POS_S_MASK        (0xffff)

#define UART_URX_IR_POSITION_CR_IR_POS_S_MASK        (0xffff)

#define UART_URX_RTO_TIMER_CR_RTO_VALUE_MASK         (0xff)

#define UART_INT_STS_URX_FER_INT                    (1 << 7)
#define UART_INT_STS_UTX_FER_INT                    (1 << 6)
#define UART_INT_STS_URX_PCE_INT                    (1 << 5)
#define UART_INT_STS_URX_RTO_INT                    (1 << 4)
#define UART_INT_STS_URX_FIFO_INT                   (1 << 3)
#define UART_INT_STS_UTX_FIFO_INT                   (1 << 2)
#define UART_INT_STS_URX_END_INT                    (1 << 1)
#define UART_INT_STS_UTX_END_INT                    (1 << 0)

#define UART_INT_MASK_CR_URX_FER_MASK               (1 << 7)
#define UART_INT_MASK_CR_UTX_FER_MASK               (1 << 6)
#define UART_INT_MASK_CR_URX_PCE_MASK               (1 << 5)
#define UART_INT_MASK_CR_URX_RTO_MASK               (1 << 4)
#define UART_INT_MASK_CR_URX_FIFO_MASK              (1 << 3)
#define UART_INT_MASK_CR_UTX_FIFO_MASK              (1 << 2)
#define UART_INT_MASK_CR_URX_END_MASK               (1 << 1)
#define UART_INT_MASK_CR_UTX_END_MASK               (1 << 0)

#define UART_INT_CLEAR_RSVD_7                       (1 << 7)
#define UART_INT_CLEAR_RSVD_6                       (1 << 6)
#define UART_INT_CLEAR_CR_URX_PCE_CLR               (1 << 5)
#define UART_INT_CLEAR_CR_URX_RTO_CLR               (1 << 4)
#define UART_INT_CLEAR_RSVD_3                       (1 << 3)
#define UART_INT_CLEAR_RSVD_2                       (1 << 2)
#define UART_INT_CLEAR_CR_URX_END_CLR               (1 << 1)
#define UART_INT_CLEAR_CR_UTX_END_CLR               (1 << 0)

#define UART_INT_EN_CR_URX_FER_EN                   (1 << 7)
#define UART_INT_EN_CR_UTX_FER_EN                   (1 << 6)
#define UART_INT_EN_CR_URX_PCE_EN                   (1 << 5)
#define UART_INT_EN_CR_URX_RTO_EN                   (1 << 4)
#define UART_INT_EN_CR_URX_FIFO_EN                  (1 << 3)
#define UART_INT_EN_CR_UTX_FIFO_EN                  (1 << 2)
#define UART_INT_EN_CR_URX_END_EN                   (1 << 1)
#define UART_INT_EN_CR_UTX_END_EN                   (1 << 0)

#define UART_STATUS_STS_URX_BUS_BUSY                (1 << 1)
#define UART_STATUS_STS_UTX_BUS_BUSY                (1 << 0)

#define UART_STS_URX_ABR_PRD_0X55_SHIFT             (16)
#define UART_STS_URX_ABR_PRD_0X55_MASK              (0xffff << UART_STS_URX_ABR_PRD_0X55_SHIFT)
#define UART_STS_URX_ABR_PRD_START_MASK             (0xffff)

#define UART_FIFO_CONFIG_0_RX_UNDERFLOW             (1 << 7)
#define UART_FIFO_CONFIG_0_RX_OVERFLOW              (1 << 6)
#define UART_FIFO_CONFIG_0_TX_UNDERFLOW             (1 << 5)
#define UART_FIFO_CONFIG_0_TX_OVERFLOW              (1 << 4)
#define UART_FIFO_CONFIG_0_RX_CLR                   (1 << 3)
#define UART_FIFO_CONFIG_0_TX_CLR                   (1 << 2)
#define UART_FIFO_CONFIG_0_DMA_RX_EN                (1 << 1)
#define UART_FIFO_CONFIG_0_DMA_TX_EN                (1 << 0)

#define UART_FIFO_CONFIG_1_RX_TH_SHIFT              (24)
#define UART_FIFO_CONFIG_1_RX_TH_MASK               (0x1f << UART_FIFO_CONFIG_1_RX_TH_SHIFT)
#define UART_FIFO_CONFIG_1_TX_TH_SHIFT              (16)
#define UART_FIFO_CONFIG_1_TX_TH_MASK               (0x1f << UART_FIFO_CONFIG_1_TX_TH_SHIFT)
#define UART_FIFO_CONFIG_1_RX_CNT_SHIFT             (8)
#define UART_FIFO_CONFIG_1_RX_CNT_MASK              (0x3f << UART_FIFO_CONFIG_1_RX_CNT_SHIFT)
#define UART_FIFO_CONFIG_1_TX_CNT_MASK              (0x3f)

#define UART_FIFO_WDATA_MASK                        (0xff)

#define UART_FIFO_RDATA_MASK                        (0xff)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_UART_H */
