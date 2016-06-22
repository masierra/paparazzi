/*
 * Copyright (C) 2016 Michael Sierra, Transition Robotics Inc.
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
 * @file subsystems/libcanard/actuator/canard_actuators.h
 * receive and send commands via libcanard
 */

#ifndef CANARD_ACTUATORS_H
#define CANARD_ACTUATORS_H

#include "subsystems/node.h"
// #include "subsystems/actuators.h"

#define CANARD_ACTUATORS_NB 8
#define COMMAND_ARRAY_DTID 1010
#define CANARD_ACTUATORS_DTID 0x26EBF643995F91A0ULL

#define CANARD_ACTUATORS_TIMEOUT_MS 1000

/** individual command stucture*/
typedef struct Command
{
	uint8_t actuator_id;
	uint16_t command_value;
} Command;

typedef struct CommandArray
{
	Command command_array[CANARD_ACTUATORS_NB];
} CommandArray;

/** actuators internal to AP (set to actual pwm on receiving side)*/
typedef struct CanardActuators
{
	uint8_t actuator_ids[CANARD_ACTUATORS_NB];
	uint16_t command_values[CANARD_ACTUATORS_NB];
} CanardActuators;

extern CanardActuators canard_actuators;

void canard_actuators_init(void);
void canard_set_actuator(uint8_t i, uint16_t value);
int canard_publish_actuators(CanardInstance* ins);
void canard_actuators_recieve_msg(void* payload);
void canard_set_actuators(void);
bool canard_status_okay(void);

#endif /* CANARD_ACTUATORS_H */
