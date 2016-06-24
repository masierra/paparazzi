/*
 * Copyright (C) 2016 Michael Sierra, Transition Robotics Inc.
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/libcanard/actuator/node_status.c
 * receive and send commands via libcanard
 */

#include "subsystems/libcanard/node_status.h"

#include "mcu_periph/sys_time.h"

uavcan_protocol_NodeStatus local_status;

uavcan_protocol_NodeStatus ext_status;
uint8_t ext_node_id;

uint32_t t_since_last_node_status; //ms since last update from slave

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_canard_node_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t node_status_okay = (uint8_t)canard_node_status_okay();
  pprz_msg_send_CANARD_NODE_STATUS(trans, dev, AC_ID,
    &ext_node_id, &ext_status.uptime_sec, &node_status_okay);
}
#endif

/** 
  *
  */
void canard_node_status_init(void)
{
  t_since_last_node_status = 0;

  local_status.health = HEALTH_OK;
  local_status.mode = MODE_OPERATIONAL;
  local_status.vendor_specific_status_code = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CANARD_NODE_STATUS, send_canard_node_status);
#endif
}

/** 
  *
  */
void node_status_receive_msg(void* payload, uint8_t source_node_id)
{
  ext_node_id = source_node_id;
  uavcan_protocol_NodeStatus__deserialize(&ext_status, payload);
  t_since_last_node_status = get_sys_time_msec();
}

/** 
  *
  */
void uavcan_protocol_NodeStatus__deserialize(uavcan_protocol_NodeStatus *out, uint8_t *buffer)
{
  out->uptime_sec = (buffer[0] << 0) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
  out->health = (buffer[4] >> 6) & 0x3;
  out->mode = (buffer[4] >> 3) & 0x7;
  out->sub_mode = (buffer[4] >> 0) & 0x7;
  out->vendor_specific_status_code = (buffer[5] << 0) | (buffer[6] << 8);
}

/** 
  *
  */
void uavcan_protocol_NodeStatus__serialize(uint8_t *buffer, uavcan_protocol_NodeStatus value)
{
  buffer[0] = (value.uptime_sec >> 0)  & 0xFF;
  buffer[1] = (value.uptime_sec >> 8)  & 0xFF;
  buffer[2] = (value.uptime_sec >> 16) & 0xFF;
  buffer[3] = (value.uptime_sec >> 24) & 0xFF;
  buffer[4] = (value.health << 6) | (value.mode << 3) | (value.sub_mode << 0);
  buffer[5] = (value.vendor_specific_status_code >> 0) & 0xFF;
  buffer[6] = (value.vendor_specific_status_code >> 8) & 0xFF;
}

/** 
  * 
  */
int publish_node_status(CanardInstance* ins)
{
  uint8_t payload[7];

  local_status.uptime_sec = sys_time.nb_sec;

  uavcan_protocol_NodeStatus__serialize(payload, local_status);

  static const uint16_t data_type_id = NODE_STATUS_DTID;
  static uint8_t transfer_id;
  uint64_t data_type_signature = NODE_STATUS_SIG;
  return canardBroadcast(ins, data_type_signature,
                         data_type_id, &transfer_id, PRIORITY_LOW, payload, sizeof(payload));
}

/** return FALSE if canard_actuators_watchdog has timed out*/
bool canard_node_status_okay(void)
{
  if (get_sys_time_msec() - t_since_last_node_status > NODE_STATUS_TIMEOUT_MS)
  {
    return FALSE;
  } else
  {
    return TRUE;
  }
}