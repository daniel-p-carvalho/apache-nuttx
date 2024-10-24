/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_1ph_inverter.c
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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ram_vectors.h"

#include "stm32l4_pwm.h"
#include "stm32l4_tim.h"

#ifdef CONFIG_NUCLEOL432KC_1PH_INVERTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Assertions ***************************************************************/

#ifndef CONFIG_ARCH_CHIP_STM32L432KC
#  warning "This only have been verified with CONFIG_ARCH_CHIP_STM32L432KC"
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error "CONFIG_ARCH_HIPRI_INTERRUPT is required"
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error "CONFIG_ARCH_RAMVECTORS is required"
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error "CONFIG_ARCH_IRQPRIO is required"
#endif

#ifndef CONFIG_ARCH_FPU
#  warning "Set CONFIG_ARCH_FPU for hardware FPU support"
#endif

/* Check the configuration for TIM1 */

#ifdef CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1

/* Leg 1 upper switch is TIM1 CH1 */

#  ifndef CONFIG_STM32L4_TIM1_CH1OUT
#    error
#  endif
#  ifndef CONFIG_STM32L4_TIM6
#    error
#  endif

/* Leg 1 lower switch is TIM1 CH2 */

#  ifndef CONFIG_STM32L4_TIM1_CH2OUT
#    error
#  endif

// /* Leg 2 upper switch is TIM1 CH3 */

// #  ifndef CONFIG_STM32L4_TIM1_CH3OUT
// #    error
// #  endif

// /* Leg 2 lower switch is TIM1 CH4 */

// #  ifndef CONFIG_STM32L4_TIM1_CH4OUT
// #    error
// #  endif

#endif /* CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1 */

/* Configuration ************************************************************/

#define SAMPLES_NUM CONFIG_NUCLEOL432KC_1PH_INVERTER_SAMPLES
#define PHASES_NUM  CONFIG_NUCLEOL432KC_1PH_INVERTER_PHASE_NUM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Inverter private data */

struct inverter_s
{
  struct stm32l4_pwm_dev_s *pwm;
#ifdef CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1
  struct stm32l4_tim_dev_s *tim;
#endif
  float waveform[SAMPLES_NUM];               /* Waveform samples */
  float phase_step;                          /* Waveform phase step */
  float waveform_freq;                       /* Waveform frequency */
  uint16_t cmp[SAMPLES_NUM];                 /* PWM TIM compare table */
  uint16_t per;                              /* PWM TIM period */
  uint16_t samples;                          /* Modulation waveform samples num */
  volatile uint16_t leg1_idx;                /* Current sample number for phase */
  volatile uint16_t leg2_idx;                /* Current sample number for phase */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct inverter_s g_inverter =
{
  .waveform_freq = ((float)CONFIG_NUCLEOL432KC_1PH_INVERTER_FREQ),
  .samples       = SAMPLES_NUM,
  .leg1_idx      = 0,
  .leg2_idx      = SAMPLES_NUM / 2,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static float waveform_func(float x);
static int waveform_init(struct inverter_s *inverter, float (*f)(float));
static int inverter_start(struct inverter_s *inverter);
static int inverter_start(struct inverter_s *inverter);
static int inverter_stop(struct inverter_s *inverter);
#ifdef CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1
static int inverter_tim1_setup(struct inverter_s *inverter);
static int inverter_tim6_setup(struct inverter_s *inverter);
static int inverter_tim1_start(struct inverter_s *inverter);
static int inverter_tim6_start(struct inverter_s *inverter);
static int inverter_tim1_stop(struct inverter_s *inverter);
static int inverter_tim6_stop(struct inverter_s *inverter);
#endif /* CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1 */
static int inverter_setup(struct inverter_s *inverter);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: waveform_func
 *
 * Description:
 *   Modulation function. This function must return values from <0.0, 1.0>!
 *
 ****************************************************************************/

static float waveform_func(float x)
{
  DEBUGASSERT(x >= 0 && x <= 2 * M_PI);

  /* Sine modulation */

  return (sinf(x) * 0.4) + 0.5;
}

/****************************************************************************
 * Name: waveform_init
 *
 * Description:
 *   Initialize modulation waveform
 *
 ****************************************************************************/

static int waveform_init(struct inverter_s *inverter, float (*f)(float))
{
  uint16_t i = 0;
  int ret = 0;

  printf("Initialize waveform\n");

  /* Get phase step to achieve one sine waveform period */

  inverter->phase_step = (float)(2 * M_PI / inverter->samples);

  /* Initialize sine and PWM compare tables */

  for (i = 0; i < inverter->samples; i += 1)
    {
      /* We need sine modulation in range from 0 to 1.0 */

      inverter->waveform[i] = f(inverter->phase_step * i);
      inverter->cmp[i] = (uint16_t)(inverter->waveform[i] * inverter->per);
    }

  printf("\tsamples = %d\n", inverter->samples);
  printf("\tper     = %d\n", inverter->per);

  return ret;
}

/****************************************************************************
 * Name: inverter_start
 *
 * Description:
 *   Start Inverter
 *
 ****************************************************************************/

static int inverter_start(struct inverter_s *inverter)
{
  /* Start TIM1 */

  inverter_tim1_start(inverter);

  /* Start TIM6 */

  inverter_tim6_start(inverter);

  return OK;
}

/****************************************************************************
 * Name: inverter_stop
 *
 * Description:
 *   Stop Inverter
 *
 ****************************************************************************/

static int inverter_stop(struct inverter_s *inverter)
{
  /* Stop TIM1 */

  inverter_tim1_stop(inverter);

  /* Stop TIM6 */

  inverter_tim6_stop(inverter);

  return OK;
}

#ifdef CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1

/****************************************************************************
 * Name: tim6_handler
 ****************************************************************************/

static void tim6_handler(void)
{
  struct inverter_s *inverter = &g_inverter;
  struct stm32l4_pwm_dev_s *pwm = inverter->pwm;
  struct stm32l4_tim_dev_s *tim = inverter->tim;

  /* Set new CMP for timers */

  PWM_CCR_UPDATE(pwm, 1, inverter->cmp[inverter->leg1_idx]);
  PWM_CCR_UPDATE(pwm, 2, inverter->cmp[inverter->leg2_idx]);

  /* Increase sample pointer */

  inverter->leg1_idx += 1;
  if (inverter->leg1_idx == inverter->samples)
    {
      inverter->leg1_idx = 0;
    }
  
  inverter->leg2_idx += 1;
  if (inverter->leg2_idx == inverter->samples)
    {
      inverter->leg2_idx = 0;
    }

  /* TODO: Software update */

  STM32L4_TIM_ACKINT(tim, ATIM_SR_UIF);
}

/****************************************************************************
 * Name: inverter_tim6_setup
 ****************************************************************************/

static int inverter_tim6_setup(struct inverter_s *inverter)
{
  struct stm32l4_tim_dev_s *tim = NULL;
  uint64_t freq = 0;
  int ret = OK;

  /* Get TIM6 interface */

  tim = stm32l4_tim_init(6);
  if (tim == NULL)
    {
      printf("ERROR: Failed to get TIM6 interface\n");
      ret = -1;
      goto errout;
    }

  inverter->tim = tim;

  /* Frequency with which we will change samples.
   *
   * tim6_freq = samples_num * waveform_freq.
   */

  freq = inverter->samples * inverter->waveform_freq;

  STM32L4_TIM_SETFREQ(tim, freq);
  STM32L4_TIM_ENABLE(tim);

  /* Attach TIM6 ram vector */

  ret = arm_ramvec_attach(STM32L4_IRQ_TIM6, tim6_handler);
  if (ret < 0)
    {
      printf("ERROR: arm_ramvec_attach failed: %d\n", ret);
      ret = -1;
      goto errout;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(STM32L4_IRQ_TIM6, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      printf("ERROR: up_prioritize_irq failed: %d\n", ret);
      ret = -1;
      goto errout;
    }

  inverter_tim6_stop(inverter);

errout:
  return ret;
}

/****************************************************************************
 * Name: inverter_tim6_start
 ****************************************************************************/

static int inverter_tim6_start(struct inverter_s *inverter)
{
  struct stm32l4_tim_dev_s *tim = inverter->tim;

  /* Enable the timer interrupt at the NVIC and at TIM6 */

  up_enable_irq(STM32L4_IRQ_TIM6);
  STM32L4_TIM_ENABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: inverter_tim6_stop
 ****************************************************************************/

static int inverter_tim6_stop(struct inverter_s *inverter)
{
  struct stm32l4_tim_dev_s *tim = inverter->tim;

  /* Disable the timer interrupt at the NVIC and at TIM6 */

  up_disable_irq(STM32L4_IRQ_TIM6);
  STM32L4_TIM_DISABLEINT(tim, BTIM_DIER_UIE);

  return OK;
}

/****************************************************************************
 * Name: inverter_tim1_setup
 ****************************************************************************/

static int inverter_tim1_setup(struct inverter_s *inverter)
{
  struct stm32l4_pwm_dev_s *pwm = NULL;
  int ret = OK;

  /* Get TIM1 PWM interface */

  pwm = (struct stm32l4_pwm_dev_s *)stm32l4_pwminitialize(1);
  if (pwm == NULL)
    {
      printf("ERROR: Failed to get TIM1 PWM interface\n");
      ret = -1;
      goto errout;
    }

  inverter->pwm = pwm;

  /* Initial PWM1 setup */

  ret = PWM_SETUP(pwm);
  if (ret < 0)
    {
      printf("ERROR: Failed to get setup TIM1 PWM\n");
      ret = -1;
      goto errout;
    }

  /* Configure TIM1 PWM frequency */

  ret = PWM_FREQ_UPDATE(pwm, CONFIG_NUCLEOL432KC_1PH_INVERTER_PWM_FREQ);
  if (ret < 0)
    {
      printf("ERROR: Failed to set TIM1 PWM frequency\n");
      ret = -1;
      goto errout;
    }

  /* Get TIM1 period (ARR) */

  inverter->per = PWM_ARR_GET(pwm);

  inverter_tim1_stop(inverter);

errout:
  return ret;
}

/****************************************************************************
 * Name: inverter_tim1_start
 ****************************************************************************/

static int inverter_tim1_start(struct inverter_s *inverter)
{
  struct stm32l4_pwm_dev_s *pwm = inverter->pwm;

  /* Enable PWM outputs */

  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT1, true);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT1N, true);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT2, true);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT2N, true);
  // PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT3, true);
  // PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT4, true);

  /* Enable TIM1 */

  PWM_TIM_ENABLE(pwm, true);

  return OK;
}

/****************************************************************************
 * Name: inverter_tim1_stop
 ****************************************************************************/

static int inverter_tim1_stop(struct inverter_s *inverter)
{
  struct stm32l4_pwm_dev_s *pwm = inverter->pwm;

  /* Disable PWM outputs */

  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT1, false);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT1N, false);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT2, false);
  PWM_OUTPUTS_ENABLE(pwm, STM32L4_PWM_OUT2N, false);

  /* Disable TIM1 */

  PWM_TIM_ENABLE(pwm, false);

  return OK;
}

#endif /* CONFIG_NUCLEOL432KC_1PH_INVERTER_USE_TIM1 */

/****************************************************************************
 * Name: inverter_setup
 ****************************************************************************/

static int inverter_setup(struct inverter_s *inverter)
{
  int ret = OK;

  /* TIM1 setup - PWM */

  printf("Setup TIM1 and TIM6\n");
  ret = inverter_tim1_setup(inverter);
  if (ret < 0)
    {
      goto errout;
    }

  /* TIM6 setup - IRQ */

  ret = inverter_tim6_setup(inverter);
  if (ret < 0)
    {
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inverter_main
 *
 * Description:
 *   Entrypoint for single phase example.
 *
 ****************************************************************************/

int inverter_main(int argc, char *argv[])
{
  struct inverter_s *inverter = NULL;
  int ret = OK;
  int i = 0;

  inverter = &g_inverter;

  printf("\ninverter_main: Started\n");

  /* Setup single phase example */

  ret = inverter_setup(inverter);
  if (ret < 0)
    {
      printf("ERROR: failed to setup 1PH_INVERTER %d!\n", ret);
      goto errout;
    }

  /* Initialize modulation waveform */

  ret = waveform_init(inverter, waveform_func);
  if (ret < 0)
    {
      printf("ERROR: failed initialize modulation wavefrom %d!\n", ret);
      goto errout;
    }

  /* Start Inverter */

  ret = inverter_start(inverter);
  if (ret < 0)
    {
      printf("ERROR: failed start inverter %d!\n", ret);
      goto errout;
    }

  /* Main loop */

  while (1)
    {
      /* Print counter */

      printf("%d\n", i);

      /* Increase counter */

      i += 1;

      /* Sleep */

      nxsig_sleep(1);
    }

errout:
  inverter_stop(inverter);

  return 0;
}

#endif /* CONFIG_NUCLEOL432KC_1PH_INVERTER */
