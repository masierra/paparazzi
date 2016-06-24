// @file: uavcan_node.h
#ifndef NODE_H
#define NODE_H

#include <canard.h>
#include "subsystems/libcanard/actuator/canard_actuators.h"
#include "subsystems/libcanard/node_status.h"

/// Arbitrary priority values
#define PRIORITY_HIGHEST 0
#define PRIORITY_HIGH    8
#define PRIORITY_MEDIUM  16
#define PRIORITY_LOW     24
#define PRIORITY_LOWEST  31

#define MSEC_TO_SEND_NODE_STATUS 1000
#define MSEC_TO_SEND_ACTUATORS 1

// int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode, uint16_t vendor_specific_status_code);
void node_handle_frame(uint32_t id, uint8_t *buf, uint8_t len);
void on_reception(CanardInstance* ins, CanardRxTransfer* transfer);
bool should_accept(const CanardInstance* ins, uint64_t* out_data_type_signature,
                   uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id);
extern void node_init(void);
extern void node_periodic(void);

#endif
