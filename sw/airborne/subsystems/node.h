// @file: uavcan_node.h
#ifndef NODE_H
#define NODE_H

#include <libcanard/src/canard.h>
#include "subsystems/libcanard/actuator/canard_actuators.h"

/// Arbitrary priority values
static const uint8_t PRIORITY_HIGHEST = 0;
static const uint8_t PRIORITY_HIGH    = 8;
static const uint8_t PRIORITY_MEDIUM  = 16;
static const uint8_t PRIORITY_LOW     = 24;
static const uint8_t PRIORITY_LOWEST  = 31;

 // implementation details on the sbp side can be found in ext/libsbp/c
/** SBP callback node.
 * Forms a linked list of callbacks.
 * \note Must be statically allocated for use with canard_register_callback().
 */
/*typedef struct canard_callbacks_node {
  uint16_t data_type_id;
  canard_msg_callback_t cb;
  struct canard_callbacks_node *next;
} canard_callbacks_node_t;
*/

/// Defined for the standard data type uavcan.protocol.NodeStatus
enum node_health
{
    HEALTH_OK       = 0,
    HEALTH_WARNING  = 1,
    HEALTH_ERROR    = 2,
    HEALTH_CRITICAL = 3
};

/// Defined for the standard data type uavcan.protocol.NodeStatus
enum node_mode
{
    MODE_OPERATIONAL     = 0,
    MODE_INITIALIZATION  = 1,
    MODE_MAINTENANCE     = 2,
    MODE_SOFTWARE_UPDATE = 3,
    MODE_OFFLINE         = 7
};
int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode, uint16_t vendor_specific_status_code);
void node_handle_frame(uint32_t id, uint8_t *buf, uint8_t len);
void on_reception(CanardInstance* ins, CanardRxTransfer* transfer);
extern void node_init(void);
extern void node_periodic(void);

#endif
// // @file: uavcan_node.h
// #ifndef NODE_H
// #define NODE_H

// #include "subsystems/libcanard/canard.h"
// #include "subsystems/libcanard/actuator/canard_actuators.h"

// /// Arbitrary priority values
// static const uint8_t PRIORITY_HIGHEST = 0;
// static const uint8_t PRIORITY_HIGH    = 8;
// static const uint8_t PRIORITY_MEDIUM  = 16;
// static const uint8_t PRIORITY_LOW     = 24;
// static const uint8_t PRIORITY_LOWEST  = 31;

//  // implementation details on the sbp side can be found in ext/libsbp/c
// /** SBP callback node.
//  * Forms a linked list of callbacks.
//  * \note Must be statically allocated for use with canard_register_callback().
//  */
// /*typedef struct canard_callbacks_node {
//   uint16_t data_type_id;
//   canard_msg_callback_t cb;
//   struct canard_callbacks_node *next;
// } canard_callbacks_node_t;
// */

// /// Defined for the standard data type uavcan.protocol.NodeStatus
// enum node_health
// {
//     HEALTH_OK       = 0,
//     HEALTH_WARNING  = 1,
//     HEALTH_ERROR    = 2,
//     HEALTH_CRITICAL = 3
// };

// /// Defined for the standard data type uavcan.protocol.NodeStatus
// enum node_mode
// {
//     MODE_OPERATIONAL     = 0,
//     MODE_INITIALIZATION  = 1,
//     MODE_MAINTENANCE     = 2,
//     MODE_SOFTWARE_UPDATE = 3,
//     MODE_OFFLINE         = 7
// };

// static void sendFrame(const CanardCANFrame* frame);
// int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode, uint16_t vendor_specific_status_code);
// void node_handle_frame(uint32_t id, uint8_t *buf, uint8_t len);
// void on_reception(CanardInstance* ins, CanardRxTransfer* transfer);
// extern void node_init(void);
// extern void node_periodic(void);

// #endif