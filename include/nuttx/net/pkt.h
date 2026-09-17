/****************************************************************************
 * include/nuttx/net/pkt.h
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

#ifndef __INCLUDE_NUTTX_NET_PKT_H
#define __INCLUDE_NUTTX_NET_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_input
 *
 * Description:
 *   Handle incoming packet input
 *
 *   This function provides the interface between Ethernet device drivers and
 *   packet socket logic.  All frames that are received should be provided to
 *   pkt_input() prior to other routing.
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK    The packet has been processed  and can be deleted
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Useful when a packet arrives before a recv call is in
 *         place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
int pkt_input(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_TIMESTAMP

/****************************************************************************
 * Name: pkt_tx_timestamp_complete
 *
 * Description:
 *   Deliver a TX timestamp for a completed transmission directly, without
 *   requiring the frame to loop back through the receive path first (the
 *   only delivery path SO_TIMESTAMPING has out of the box). Drivers that
 *   know when a transmission with a timestamp request completes (e.g. from
 *   a TX-complete interrupt) call this instead, passing the timestamp to
 *   record and the socket connection that requested it.
 *
 * Input Parameters:
 *   dev  - The device driver structure for the interface the frame was
 *          sent on
 *   conn - The socket connection that requested the timestamp (read back
 *          from the transmitted iob's io_conn field by the driver)
 *   ts   - The TX-complete timestamp to deliver
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called from the network driver with the network locked.
 *
 ****************************************************************************/

struct socket_conn_s;  /* Forward reference */
struct timespec;       /* Forward reference */
int pkt_tx_timestamp_complete(FAR struct net_driver_s *dev,
                              FAR struct socket_conn_s *conn,
                              FAR const struct timespec *ts);

#endif /* CONFIG_NET_TIMESTAMP */

#endif /* __INCLUDE_NUTTX_NET_PKT_H */
