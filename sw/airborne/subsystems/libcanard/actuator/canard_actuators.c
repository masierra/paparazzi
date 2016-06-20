#include "canard_actuators.h"

static CommandArray commands;     //outgoing commands
CanardActuators canard_actuators; //received commands


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_canard_actuators(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CANARD_ACTUATORS(trans, dev, AC_ID,
    CANARD_ACTUATORS_NB, canard_actuators.actuator_ids, CANARD_ACTUATORS_NB, canard_actuators.command_values);
}
#endif

void canard_actuators_init(void) {
  int i;
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
      commands.command_array[i].actuator_id = i;
      commands.command_array[i].command_value = 0; //((uint16_t)i)*222;
  }
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
    canard_actuators.actuator_ids[i] = i;
    canard_actuators.command_values[i] = 1000;
  }
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CANARD_ACTUATORS, send_canard_actuators);
#endif
}

void canard_set_actuator(uint8_t i, uint16_t value)
{
  commands.command_array[i].actuator_id = i;
  commands.command_array[i].command_value = value;
}

int canard_publish_actuators(CanardInstance* ins) {
  static const uint16_t data_type_id = COMMAND_ARRAY_DTID;
  static uint8_t transfer_id;
  // int i;

  // for(i=0;i<CANARD_ACTUATORS_NB;i++) {
  //   commands.command_array[i].actuator_id = i;
  //   commands.command_array[i].command_value = actuators[i];
  // }
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

// void canard_set_actuators(void)
// {
//   int i;
//   for (i=0; i<CANARD_ACTUATORS_NB; i++) {
//     ActuatorPwmSet(i,canard_actuators.command_values[i]);
//   }
//   AllActuatorsCommit();
// }