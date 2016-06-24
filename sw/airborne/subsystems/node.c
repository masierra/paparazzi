// @file: uavcan_node.c
#include "std.h"
#include "node.h"
#include <string.h>
#include <stdio.h>
#include "mcu_periph/can.h"
#include "mcu_periph/sys_time.h"

#ifdef CANARD_SENDER
#define PPZ_NODE_ID 1
#else
#define PPZ_NODE_ID 2
#endif

static CanardInstance canard_instance;
static CanardPoolAllocatorBlock canard_buf[32];           // pool blocks

// #if PERIODIC_TELEMETRY
// #include "subsystems/datalink/telemetry.h"

// static void send_canard_debug(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_CANARD_DEBUG(trans, dev, AC_ID, 
//                               &canard_instance.allocator_space,
//                               &canard_instance.tx_queue_count);
// }
// #endif


/*
 * function to handle recieption of can frame. (isr)
 */
void node_handle_frame(uint32_t id, uint8_t *buf, uint8_t len) {
  CanardCANFrame canard_frame;
  canard_frame.id = id;

  memcpy(&canard_frame.data, buf, len);
  canard_frame.data_len = len;

  // queue frame
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

      if (CANARD_RX_PAYLOAD_HEAD_SIZE > 0)
      {
        for (i = 0; i < CANARD_RX_PAYLOAD_HEAD_SIZE; i++, index++)
        {
          payload[i] = transfer->payload_head[i];
        }
      }

      for (i=0; index < (CANARD_RX_PAYLOAD_HEAD_SIZE + transfer->middle_len); i++, index++)
      {
        payload[index] = block->data[i];
        if (i==CANARD_BUFFER_BLOCK_DATA_SIZE - 1)
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
    } else
    {
      uint8_t i;
      for (i = 0; i<transfer->payload_len; i++)
      {
        payload[i] = transfer->payload_head[i];
      }
    }

    switch (transfer->data_type_id)
    {
      case(COMMAND_ARRAY_DTID):
        canard_actuators_recieve_msg(payload);
      break;

      case(NODE_STATUS_DTID):
        node_status_receive_msg(payload, transfer->source_node_id);
      break;

      default:
      break;
    }

    canardReleaseRxTransferPayload(ins, transfer);

}

bool should_accept(const CanardInstance* ins, uint64_t* out_data_type_signature,
                   uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id)
{
    *out_data_type_signature = CANARD_ACTUATORS_SIG;
    return true;
}

// void canardInit(CanardInstance* out_ins,  void* mem_arena, size_t mem_arena_size,
                // CanardOnTransferReception on_reception, CanardShouldAcceptTransfer should_accept)

void node_init(void) {
  //initialize can hardware/callback
  ppz_can_init(node_handle_frame);

  //initialize actuators submodule
  canard_actuators_init();

  //initialize node_status
  canard_node_status_init();

  //initialize canard instance
  canardInit(&canard_instance, canard_buf, sizeof(canard_buf), on_reception, should_accept);
  canardSetLocalNodeID(&canard_instance, PPZ_NODE_ID);

// #if PERIODIC_TELEMETRY
//   register_periodic_telemetry(DefaultPeriodic, "CANARD_DEBUG", send_canard_debug);
// #endif
}

void node_periodic(void) {
  canardCleanupStaleTransfers(&canard_instance, (uint64_t)get_sys_time_usec());

  static uint32_t last_actuators_ms = 0;
  static uint32_t last_node_status_ms = 0;

  // send out node status periodically
  if ((get_sys_time_msec() - last_node_status_ms) > MSEC_TO_SEND_NODE_STATUS)
  {
    publish_node_status(&canard_instance);
    last_node_status_ms = get_sys_time_msec();
  }

  // transmit anything in the queue
  const CanardCANFrame* transmit_frame = canardPeekTxQueue(&canard_instance);
  if (transmit_frame != NULL)
  {
    int ret;
    ret = ppz_can_transmit(transmit_frame->id, transmit_frame->data, transmit_frame->data_len);
    if (ret != -1)
    {
      canardPopTxQueue(&canard_instance);
    }
    return;
  }

  //  queue canard actuators
#ifdef CANARD_SENDER
  if ((get_sys_time_msec() - last_actuators_ms) > MSEC_TO_SEND_ACTUATORS)
  {
    canard_publish_actuators(&canard_instance);
    last_actuators_ms = get_sys_time_msec();
  }
#endif

  return;
}
