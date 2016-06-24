/*
 * Copyright (C) 2016 Transition Robotics Inc.
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
 * @file subsystems/libcanard/node_status.h
 * publish and receive node status
 */
#ifndef NODE_STATUS_H
#define NODE_STATUS_H

#include "subsystems/node.h"

// data type paremeters
#define NODE_STATUS_DTID 341
#define NODE_STATUS_SIG 0xF0868D0C1A7C6F1
#define NODE_STATUS_TIMEOUT_MS 3000

// constants
#define HEALTH_OK        0
#define HEALTH_WARNING   1
#define HEALTH_ERROR     2
#define HEALTH_CRITICAL  3

#define MODE_OPERATIONAL      0
#define MODE_INITIALIZATION   1
#define MODE_MAINTENANCE      2
#define MODE_SOFTWARE_UPDATE  3
#define MODE_OFFLINE          7

typedef struct
{
    uint32_t uptime_sec;
    uint8_t health;
    uint8_t mode;
    uint8_t sub_mode;
    uint16_t vendor_specific_status_code;
} uavcan_protocol_NodeStatus;

void canard_node_status_init(void);
void node_status_receive_msg(void* payload, uint8_t source_node_id);
void uavcan_protocol_NodeStatus__deserialize(uavcan_protocol_NodeStatus *out, uint8_t *buffer);
void uavcan_protocol_NodeStatus__serialize(uint8_t *buffer, uavcan_protocol_NodeStatus value);
int publish_node_status(CanardInstance* ins);
bool canard_node_status_okay(void);

#endif /* NODE_STATUS_H */