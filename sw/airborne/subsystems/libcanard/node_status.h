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
#include "subsystems/node.h"

#include "mcu_periph/sys_time.h"

// data type paremeters
#define NODE_STATUS_DTID 341
#define NODE_STATUS_SIG 0xF0868D0C1A7C6F1

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

uavcan_protocol_NodeStatus node_status;

void uavcan_protocol_NodeStatus__deserialize(uavcan_protocol_NodeStatus *out, uint8_t *buffer);
void uavcan_protocol_NodeStatus__serialize(uint8_t *buffer, uavcan_protocol_NodeStatus value);
int publish_node_status(CanardInstance* ins);

void uavcan_protocol_NodeStatus__deserialize(uavcan_protocol_NodeStatus *out, uint8_t *buffer)
{
    out->uptime_sec = (buffer[0] << 0) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    out->health = (buffer[4] >> 6) & 0x3;
    out->mode = (buffer[4] >> 3) & 0x7;
    out->sub_mode = (buffer[4] >> 0) & 0x7;
    out->vendor_specific_status_code = (buffer[5] << 0) | (buffer[6] << 8);
}

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

int publish_node_status(CanardInstance* ins)
{
    uint8_t payload[7];

    uavcan_protocol_NodeStatus__serialize(payload, node_status);

    static const uint16_t data_type_id = NODE_STATUS_DTID;
    static uint8_t transfer_id;
    uint64_t data_type_signature = NODE_STATUS_SIG;
    return canardBroadcast(ins, data_type_signature,
                           data_type_id, &transfer_id, PRIORITY_LOW, payload, sizeof(payload));
}
