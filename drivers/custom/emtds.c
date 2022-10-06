/****************************************************************************
 * emtds/boards/common/src/emtds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>

#include "nuttx/custom/emtds.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     emtds_open(FAR struct file *filep);
static int     emtds_close(FAR struct file *filep);
static ssize_t emtds_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t emtds_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     emtds_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations emtds_fops =
{
  emtds_open,                   /* open */
  emtds_close,                  /* close */
  emtds_read,                   /* read */
  emtds_write,                  /* write */
  NULL,                         /* seek */
  emtds_ioctl,                  /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emtds_open
 *
 * Description:
 *   This function is called whenever the EMTDS device is opened.
 *
 ****************************************************************************/

static int emtds_open(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct emtds_dev_s *dev   = inode->i_private;
  uint8_t                tmp;
  int                    ret;

  /* If the port is the middle of closing, wait until the close is finished */

  ret = nxsem_wait(&dev->closesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then
   * initialize the device.
   */

  tmp = dev->ocount + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
    }
  else
    {
      /* Check if this is the first time that the driver has been
       * opened.
       */

      if (tmp == 1)
        {
          /* Yes.. perform one time hardware initialization. */

          irqstate_t flags = enter_critical_section();
          ret = dev->ops->setup(dev);
          if (ret == OK)
            {
              /* Save the new open count on success */

              dev->ocount = tmp;
            }

          leave_critical_section(flags);
        }
    }

  nxsem_post(&dev->closesem);

  return OK;
}

/****************************************************************************
 * Name: emtds_close
 *
 * Description:
 *   This function is called when the EMTDS device is closed.
 *
 ****************************************************************************/

static int emtds_close(FAR struct file *filep)
{
  FAR struct inode       *inode  = filep->f_inode;
  FAR struct emtds_dev_s *dev   = inode->i_private;
  irqstate_t             flags;
  int                    ret;

  ret = nxsem_wait(&dev->closesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the references to the driver. If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (dev->ocount > 1)
    {
      dev->ocount--;
      nxsem_post(&dev->closesem);
    }
  else
    {
      /* There are no more references to the port */

      dev->ocount = 0;

      /* Free the IRQ and disable the EMTDS device */

      flags = enter_critical_section();      /* Disable interrupts */
      dev->ops->shutdown(dev);               /* Disable the EMTDS */
      leave_critical_section(flags);

      nxsem_post(&dev->closesem);
    }

  return ret;
}

/****************************************************************************
 * Name: emtds_read
 ****************************************************************************/

static ssize_t emtds_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: emtds_write
 ****************************************************************************/

static ssize_t emtds_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 1;
}

/****************************************************************************
 * Name: emtds_ioctl
 ****************************************************************************/

static int emtds_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode     = filep->f_inode;
  FAR struct emtds_dev_s *dev = inode->i_private;
  FAR struct emtds_s *emtds   = (FAR struct emtds_s *)dev->priv;
  int ret;

  switch (cmd)
    {
      default:
        {
          _err("ERROR: Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emtds_register
 ****************************************************************************/

int emtds_register(FAR const char *path, FAR struct emtds_dev_s *dev,
                   FAR void *lower)
{
  int ret;

  DEBUGASSERT(path != NULL && dev != NULL && lower != NULL);
  DEBUGASSERT(dev->ops != NULL);

  /* For safety reason, when some necessary low-level logic is not provided,
   * system should fail before low-level hardware initialization, so:
   *   - all ops are checked here, before character driver registration
   *   - all ops must be provided, even if not used
   */

  DEBUGASSERT(dev->ops->setup != NULL);
  DEBUGASSERT(dev->ops->ioctl != NULL);

  /* Initialize the EMTDS device structure */

  dev->ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->closesem, 0, 1);

  /* Connect EMTDS driver with lower level interface */

  dev->lower = lower;

  /* Register the EMTDS character driver */

  ret = register_driver(path, &emtds_fops, 0666, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->closesem);
    }

  return ret;
}
