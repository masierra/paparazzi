#include "canard_actuators.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

static CommandArray commands;     //outgoing commands
CanardActuators canard_actuators; //received commands

uint32_t t_since_last_msg;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_canard_actuators(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t canard_actuators_okay = (uint8_t)canard_status_okay();
  pprz_msg_send_CANARD_ACTUATORS(trans, dev, AC_ID,
    CANARD_ACTUATORS_NB, canard_actuators.actuator_ids, CANARD_ACTUATORS_NB, canard_actuators.command_values, &canard_actuators_okay);
}
#endif

/** init */
void canard_actuators_init(void)
{
  t_since_last_msg = 0;

  int i;
  for(i=0;i<CANARD_ACTUATORS_NB;i++)
  {
    commands.command_array[i].actuator_id = i;
    commands.command_array[i].command_value = 0;
  }
  for(i=0;i<CANARD_ACTUATORS_NB;i++)
  {
    canard_actuators.actuator_ids[i] = i;
    canard_actuators.command_values[i] = 1000;
  }
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CANARD_ACTUATORS, send_canard_actuators);
#endif
}

/** set individual actuator (automatically from airframe.h)*/
void canard_set_actuator(uint8_t i, uint16_t value)
{
  commands.command_array[i].actuator_id = i;
  commands.command_array[i].command_value = value;
}

/** send actuator command via libcanard*/
int canard_publish_actuators(CanardInstance* ins)
{
  static const uint16_t data_type_id = COMMAND_ARRAY_DTID;
  static uint8_t transfer_id;

  uint64_t data_type_signature = CANARD_ACTUATORS_DTID;
  return canardBroadcast(ins, data_type_signature,
                           data_type_id, &transfer_id, 03, &commands, sizeof(commands));
}

/** callback for libcanard actuators message*/
void canard_actuators_recieve_msg(void* payload)
{
  t_since_last_msg = get_sys_time_msec();

  CommandArray *received_commands = payload;
  uint8_t i;
  for(i=0;i<CANARD_ACTUATORS_NB;i++) {
    canard_actuators.actuator_ids[i] = received_commands->command_array[i].actuator_id;
    canard_actuators.command_values[i] = received_commands->command_array[i].command_value;
  }
}

/** return FALSE if canard_actuators_watchdog has timed out*/
bool canard_status_okay(void)
{
  if (get_sys_time_msec() - t_since_last_msg > CANARD_ACTUATORS_TIMEOUT_MS)
  {
    return FALSE;
  } else
  {
    return TRUE;
  }
}

/** set pwm actuators from incoming commands*/
void canard_set_actuators(void)
{
  int i;
  for (i=0; i<ACTUATORS_PWM_NB; i++) {
    if (canard_actuators.command_values[i] != 0)
    {
      ActuatorPwmSet(i,canard_actuators.command_values[i]);
    }
  }
  ActuatorsPwmCommit();
}