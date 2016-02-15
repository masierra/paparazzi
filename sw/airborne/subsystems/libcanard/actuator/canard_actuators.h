#ifndef CANARD_ACTUATORS_H
#define CANARD_ACTUATORS_H

#include "subsystems/node.h"
#include "subsystems/actuators.h"

#define CANARD_ACTUATORS_NB ACTUATORS_NB
#define COMMAND_ARRAY_DTID 1010

typedef struct Command
{
	uint8_t actuator_id;
	uint16_t command_value;
} Command;

typedef struct CommandArray
{
	Command command_array[CANARD_ACTUATORS_NB];
} CommandArray;

typedef struct CanardActuators
{
	uint8_t actuator_ids[CANARD_ACTUATORS_NB];
	uint16_t command_values[CANARD_ACTUATORS_NB];
} CanardActuators;

extern CanardActuators canard_actuators;

void canard_actuators_init(void);
int canard_publish_actuators(CanardInstance* ins);
void canard_actuators_recieve_msg(void* payload);


#endif /* CANARD_ACTUATORS_H */
