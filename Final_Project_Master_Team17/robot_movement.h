/*
Name: robot_movement.h
Author: Jonathan Lee
Description: header file for methods related to robot movement
*/
#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <arduino.h>
#include "datatypes.h"

// robot movement methods
void tire_motor_setup();
void robot_move(Movement direction);
void robot_complex_move(int8_t speed, int8_t turn, int8_t strafe);
void robot_rotate(Movement direction);
void robot_stop();
void set_motor_speed(Motor motor, int8_t duty);

// robot claw methods
void claw_setup();
void set_claw(ClawStatus status);

// movement template
bool move_to_template(
    uint16_t parameter_1,
    uint16_t parameter_2,
    bool (*stop_condition_reached)(uint16_t, uint16_t),
    bool (*heading_reached)(uint16_t, uint16_t),
    void (*change_heading)(uint16_t, uint16_t),
    bool (*robot_stuck)(unsigned long)
);

#endif