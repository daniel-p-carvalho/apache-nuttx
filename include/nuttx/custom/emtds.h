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

/* This structure defines the lower half EMTDS interface */

struct emtds_dev_s;
struct emtds_ops_s
{
  /* Configure EMTDS */

  CODE int (*setup)(FAR struct emtds_dev_s *dev);

  /* Disable EMTDS */

  CODE int (*shutdown)(FAR struct emtds_dev_s *dev);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct emtds_dev_s *dev, int cmd, unsigned long arg);
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