#include "canard_actuators.h"

#include "subsystems/datalink/telemetry.h"

static CommandArray commands;
CanardActuators canard_actuators;


// #if PERIODIC_TELEMETRY
// #include "subsystems/datalink/telemetry.h"

// static void send_canard_actuators(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_CANARD_ACTUATORS(trans, dev, AC_ID,
//     CANARD_ACTUATORS_NB, canard_actuators.actuator_ids, CANARD_ACTUATORS_NB, canard_actuators.command_values);
// }
// #endif

void canard_actuators_init(void) {
  int i;
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
      commands.command_array[i].actuator_id = i;
      commands.command_array[i].command_value = ((uint16_t)i)*111;
  }
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
    canard_actuators.actuator_ids[i] = i;
    canard_actuators.command_values[i] = 1000;
  }
// #if PERIODIC_TELEMETRY
//   register_periodic_telemetry(DefaultPeriodic, "CANARD_ACTUATORS", send_canard_actuators);
// #endif
}

int canard_publish_actuators(CanardInstance* ins) {
  static const uint16_t data_type_id = COMMAND_ARRAY_DTID;
  static uint8_t transfer_id;
  int i;

  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
    commands.command_array[i].actuator_id = i;
    commands.command_array[i].command_value = actuators[i];
  }
  uint64_t data_type_signature = CANARD_ACTUATORS_DTID;
  return canardBroadcast(ins, data_type_signature,
                           data_type_id, &transfer_id, 03, &commands, sizeof(commands));
}

void canard_actuators_recieve_msg(void* payload) {
  CommandArray *received_commands = payload;
  uint8_t i;
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
    canard_actuators.actuator_ids[i] = received_commands->command_array[i].actuator_id;
    canard_actuators.command_values[i] = received_commands->command_array[i].command_value;
  }
}