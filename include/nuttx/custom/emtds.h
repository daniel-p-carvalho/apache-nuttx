/****************************************************************************
 * include/nuttx/drivers/emtds.h
 * NuttX EMTDS Interfaces
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

#ifndef __INCLUDE_NUTTX_DRIVERS_EMTDS_H
#define __INCLUDE_NUTTX_DRIVERS_EMTDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>

#include <stdbool.h>

#ifdef CONFIG_DRIVER_EMTDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_DRIVER_EMTDS - Upper half EMTDS driver support
 *
 * Specific, lower-half drivers will have other configuration requirements
 * such as:
 */

/* IOCTL Commands ***********************************************************/

/* The upper-half EMTDS driver provides a character driver "wrapper" around
 * the lower-half EMTDS driver that does all of the real work.
 * 
 * Since there is no real data transfer to/or from the EMTDS, all of the
 * driver interaction is through IOCTL commands.  The IOCTL commands
 * supported by the upper-half driver simply provide calls into the
 * lower half as summarized below:
 * 
 * EMTDSIOC_VOLTAGE - Define the eletronic meter test voltage.
 *   Input value:  An int defining the voltage value.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* EMTDS VGEN taps */

enum emtds_vgen_tap_e
{
  VGEN_TAP_LOW = 0,
  VGEN_TAP_HIGH,
  VGEN_TAP_AUTO = (1 << 7)
};

/* EMTDS VGEN current gain */

enum emtds_vgen_i_gain_e
{
  VGEN_I_GAIN_25 = 0,
  VGEN_I_GAIN_50,
  VGEN_I_GAIN_100,
  VGEN_I_GAIN_200
};

/* EMTDS VGEN output channel */

enum emtds_out_chn_e
{
  OUT_CHN_AN = 0,
  OUT_CHN_BN,
  OUT_CHN_CN,
  OUT_CHN_AB,
  OUT_CHN_AC,
  OUT_CHN_BC,
  OUT_CHN_OFF
};

enum emtds_circuit_e
{
  AN = 0,
  BN = 1,
  CN = 2,
  AB = 3,
  AC = 4,
  BC = 5,
  OFF
};

/* EMTDS Digital DNA */

struct emtds_dig_dna_s
{
  uint16_t voltage;        /* Test voltage (not the setpoint) */
  uint16_t current;        /* Test current */
  uint16_t int_losses;     /* Internal losses */
  uint16_t thd;            /* Total Harmonic Distortion */
  uint16_t i_spectrum[16]; /* Current Spectrum */
};

struct emtds_dig_dna_tolerances
{
  uint16_t voltage_err_max;
  uint16_t current_err_max;
  uint16_t int_losses_err_max;
  uint16_t thd_err_max;
};

struct emtds_meter_dig_dna_s
{
  uint8_t                         model_id;
  struct emtds_dig_dna_tolerances tolerances;
  struct emtds_dig_dna_s          dig_dna[6];
};

struct emtds_waveform_s
{
  uint16_t  offset;
  uint16_t  length;
  uint8_t   *data;
};

/* EMTDS parameters */

struct emtds_params_s
{
  int32_t vgen_va_setpoint;      /* Voltage generator setpoint */
  int32_t vgen_va_pi_sat_en;     /* Va pid output saturation enabled */
  int32_t vgen_va_pi_ireset_en;  /* Va pid integrator reset enabled */
  int32_t vgen_va_l_kp;          /* Va channel kp pid constant for tap low */
  int32_t vgen_va_l_ki;          /* Va channel ki pid constant for tap low */
  int32_t vgen_va_h_kp;          /* Va channel kp pid constant for tap high */
  int32_t vgen_va_h_ki;          /* Va channel ki pid constant for tap high */
  int32_t vgen_va_sat_min;       /* Va pid output min */
  int32_t vgen_va_sat_max;       /* Va pid output min */
  // int32_t vgen_out_chn;

  int32_t meas_va_gain;          /* Va channel voltage gain */
  int32_t meas_ia_gain;          /* Ia channel current gain */
  int32_t meas_a_phase;          /* Channel A phase */
};

/* EMTDS private data structure  */

struct emtds_s
{
  struct emtds_params_s      param;   /* EMTDS settings */
  FAR void                   *priv;   /* Private data */
};

/* This structure defines the lower half EMTDS interface */

struct emtds_dev_s;
struct emtds_ops_s
{
  /* Start EMTDS action */

  CODE int (*start)(FAR struct emtds_dev_s *dev);

  /* Stop EMTDS action */

  CODE int (*stop)(FAR struct emtds_dev_s *dev);

  /* Set EMTDS parameters */

  CODE int (*params_set)(FAR struct emtds_dev_s *dev,
                         FAR struct emtds_params_s *param);

  /* Configure EMTDS */

  CODE int (*setup)(FAR struct emtds_dev_s *dev);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct emtds_dev_s *dev, int cmd, unsigned long arg);

  /* Disable EMTDS */

  CODE int (*shutdown)(FAR struct emtds_dev_s *dev);
};

/* EMTDS device structure used by the driver. The caller of emtds_register
 * must allocate and initialize this structure. The calling logic need
 * provide 'ops', 'priv' and 'lower' elements.
 */

struct emtds_dev_s
{
  /* Fields managed by common upper half EMTDS logic */

  uint8_t                     ocount;   /* The number of times the device
                                         * has been opened
                                         */
  sem_t                       closesem; /* Locks out new opens while close
                                         * is in progress
                                         */

  /* Fields provided by lower half EMTDS logic */

  FAR const struct emtds_ops_s *ops;     /* Arch-specific operations */
  FAR void                     *priv;    /* Reference to EMTDS private data */
  FAR void                     *lower;   /* Reference to lower level drivers */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: emtds_register
 ****************************************************************************/

int emtds_register(FAR const char *path, FAR struct emtds_dev_s *dev,
                   FAR void *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVER_EMTDS */
#endif /* __INCLUDE_NUTTX_DRIVERS_EMTDS_H */