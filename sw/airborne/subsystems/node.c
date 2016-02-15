// @file: uavcan_node.c
#include "std.h"
#include "node.h"
#include <stdio.h>
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time.h"

#define PPZ_NODE_ID 1

static CanardInstance canard_instance;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_canard_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CANARD_DEBUG(trans, dev, AC_ID, 
                              &canard_instance.allocator_space,
                              &canard_instance.tx_queue_count);
}
#endif


/*
 * function to handle recieption of can frame. (isr)
 */
void node_handle_frame(uint32_t id, uint8_t *buf, uint8_t len) {
  CanardCANFrame canard_frame;
  canard_frame.id = id;
  memcpy(&canard_frame.data, buf, len);
  canard_frame.data_len = len;
  //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("len"), buf);
  canardHandleRxFrame(&canard_instance, &canard_frame, get_sys_time_usec());

}

/*
 * this function is called when a transfer has completed
 */
void on_reception(CanardInstance* ins, CanardRxTransfer* transfer) {
    uint8_t payload[transfer->payload_len];
    if (transfer->payload_len > 7)
    {
    CanardBufferBlock* block = transfer->payload_middle;
    uint8_t i, index = 0;

    if(CANARD_RX_PAYLOAD_HEAD_SIZE > 0)
    {
        for (i=0; i < CANARD_RX_PAYLOAD_HEAD_SIZE; i++)
        {
            payload[i] = transfer->payload_head[i];
        }
        index = i;
    }

    for (i=0; index < (CANARD_RX_PAYLOAD_HEAD_SIZE + transfer->middle_len); i++, index++)
    {
        payload[index] = block->data[i];
        if(i==CANARD_BUFFER_BLOCK_DATA_SIZE-1)
        {
            i = -1;
            block = block->next;
        }
    }
    int tail_len = transfer->payload_len - (CANARD_RX_PAYLOAD_HEAD_SIZE + transfer->middle_len);
    for (i=0; i<(tail_len);i++, index++)
    {
            payload[index] = transfer->payload_tail[i];
    }
    
    }
    else
    {
        uint8_t i;
        for (i = 0; i<transfer->payload_len; i++)
        {
            payload[i] = transfer->payload_head[i];
        }
    }
    //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, transfer->payload_len, payload);

    if (transfer->data_type_id == 1010 && transfer->payload_len == sizeof(CommandArray)){
        canard_actuators_recieve_msg(payload);
    }
    canardReleaseRxTransferPayload(ins, transfer);

}


void node_init(void) {
  //initialize can hardware/callback
  ppz_can_init(node_handle_frame);
  //DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen("init"), "init");
  //initialize canard instance
  canard_actuators_init();
  canardInit(&canard_instance, (canardOnTransferReception)on_reception);
  canardSetLocalNodeID(&canard_instance, PPZ_NODE_ID);
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "CANARD_DEBUG", send_canard_debug);
#endif
}

void node_periodic(void) {
  canardCleanupStaleTransfers(&canard_instance, (uint64_t)get_sys_time_usec());
  // //char *str = "string poop";
  static uint32_t sendstring;
  const CanardCANFrame* transmit_frame = canardPeekTxQueue(&canard_instance);
  if (transmit_frame != NULL) {
    int ret;
    ret = ppz_can_transmit(transmit_frame->id, transmit_frame->data, transmit_frame->data_len);
    if (ret != -1) {
      canardPopTxQueue(&canard_instance);
    }
    return;
  }
  // RunOnceEvery(9,canard_publish_actuators(&canard_instance));
  if((get_sys_time_usec()-sendstring)>2500){
    canard_publish_actuators(&canard_instance);
    sendstring = get_sys_time_usec();
  }

  return;
}

// /// Standard data type: uavcan.protocol.NodeStatus
// int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode, uint16_t vendor_specific_status_code)
// {
//     // static uint64_t startup_timestamp_usec;
//     // if (startup_timestamp_usec == 0)
//     // {
//     //     startup_timestamp_usec = (uint64_t)get_sys_time_usec();
//     // }
//     uint8_t i;
//     uint8_t payload[7] = {0};
//     for (i = 0;i<sizeof(payload);i++){
//       payload[i] = i;
//     }

//     // // Uptime in seconds
//     // const uint32_t uptime_sec = ((uint64_t)get_sys_time_usec() - startup_timestamp_usec) / 1000000ULL;
//     // payload[0] = (uptime_sec >> 0)  & 0xFF;
//     // payload[1] = (uptime_sec >> 8)  & 0xFF;
//     // payload[2] = (uptime_sec >> 16) & 0xFF;
//     // payload[3] = (uptime_sec >> 24) & 0xFF;

//     // payload[0] = 0XFF;
//     // payload[1] = 0XFF;
//     // payload[2] = 0XFF;
//     // // Health and mode
//     // payload[4] = ((uint8_t)health << 6) | ((uint8_t)mode << 3);

//     // // Vendor-specific status code
//     // payload[5] = (vendor_specific_status_code >> 0) & 0xFF;
//     // payload[6] = (vendor_specific_status_code >> 8) & 0xFF;

//     static const uint16_t data_type_id = 341;
//     static uint8_t transfer_id;
//     return canardBroadcast(ins, data_type_id, &transfer_id, PRIORITY_LOW, payload, sizeof(payload));
// }

// // /// Standard data type: uavcan.protocol.NodeStatus
// // int publish_node_status(CanardInstance* ins, enum node_health health, enum node_mode mode, uint16_t vendor_specific_status_code)
// // {
// //     // static uint64_t startup_timestamp_usec;
// //     // if (startup_timestamp_usec == 0)
// //     // {
// //     //     startup_timestamp_usec = (uint64_t)get_sys_time_usec();
// //     // }
// //     uint8_t i;
// //     uint8_t payload[7] = {0};
// //     for (i = 0;i<sizeof(payload);i++){
// //       payload[i] = i;
// //     }

// //     // // Uptime in seconds
// //     // const uint32_t uptime_sec = ((uint64_t)get_sys_time_usec() - startup_timestamp_usec) / 1000000ULL;
// //     // payload[0] = (uptime_sec >> 0)  & 0xFF;
// //     // payload[1] = (uptime_sec >> 8)  & 0xFF;
// //     // payload[2] = (uptime_sec >> 16) & 0xFF;
// //     // payload[3] = (uptime_sec >> 24) & 0xFF;

// //     // payload[0] = 0XFF;
// //     // payload[1] = 0XFF;
// //     // payload[2] = 0XFF;
// //     // // Health and mode
// //     // payload[4] = ((uint8_t)health << 6) | ((uint8_t)mode << 3);

// //     // // Vendor-specific status code
// //     // payload[5] = (vendor_specific_status_code >> 0) & 0xFF;
// //     // payload[6] = (vendor_specific_status_code >> 8) & 0xFF;

// //     static const uint16_t data_type_id = 341;
// //     static uint8_t transfer_id;
// //     return canardBroadcast(ins, data_type_id, &transfer_id, PRIORITY_LOW, payload, sizeof(payload));
// // }