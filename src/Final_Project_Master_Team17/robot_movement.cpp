/*
Name: robot_movement.cpp
Author: Jonathan Lee
Description: implementation of methods related to robot movement
*/

#include "robot_movement.h"

// Tire Motors Parameters
#define FR_MOTOR_FWD 19
#define FR_MOTOR_BWD 20
#define FR_PWM_PIN   21
#define FR_PWM_CHAN  0

#define FL_MOTOR_FWD 1
#define FL_MOTOR_BWD 2
#define FL_PWM_PIN   42
#define FL_PWM_CHAN  1

#define RR_MOTOR_FWD 39
#define RR_MOTOR_BWD 40
#define RR_PWM_PIN   41
#define RR_PWM_CHAN  2

#define RL_MOTOR_FWD 45
#define RL_MOTOR_BWD 34
#define RL_PWM_PIN   33
#define RL_PWM_CHAN  3

#define FREQUENCY_MOTOR  10000
#define RESOLUTION_MOTOR 10

const uint16_t MAX_DUTY_MOTOR = ((1 << RESOLUTION_MOTOR) - 1);
const int8_t LINEAR_DUTY_MOTOR = 95;
const int8_t ROTATION_DUTY_MOTOR = 30;

// Claw Parameters
#define CLAW_PWM_PIN  8
#define CLAW_PWM_CHAN 4

#define FREQUENCY_SERVO    50
#define RESOLUTION_SERVO   14
const uint16_t DUTY_OPEN = (((1 << RESOLUTION_SERVO) - 1) * 3 / 100);
const uint16_t DUTY_CLOSED = (((1 << RESOLUTION_SERVO) - 1) * 7.5 / 100);

/*
ROBOT MOVEMENT METHODS
*/

// Hardware Setup: Tire Motors (Front Right, Front Left, Rear Right, Rear Left)
void tire_motor_setup() {
  pinMode(FR_MOTOR_FWD, OUTPUT);
  pinMode(FR_MOTOR_BWD, OUTPUT);
  ledcSetup(FR_PWM_CHAN, FREQUENCY_MOTOR, RESOLUTION_MOTOR);
  ledcAttachPin(FR_PWM_PIN, FR_PWM_CHAN);
  ledcWrite(FR_PWM_CHAN, 0);

  pinMode(FL_MOTOR_FWD, OUTPUT);
  pinMode(FL_MOTOR_BWD, OUTPUT);
  ledcSetup(FL_PWM_CHAN, FREQUENCY_MOTOR, RESOLUTION_MOTOR);
  ledcAttachPin(FL_PWM_PIN, FL_PWM_CHAN);
  ledcWrite(FL_PWM_CHAN, 0);

  pinMode(RR_MOTOR_FWD, OUTPUT);
  pinMode(RR_MOTOR_BWD, OUTPUT);
  ledcSetup(RR_PWM_CHAN, FREQUENCY_MOTOR, RESOLUTION_MOTOR);
  ledcAttachPin(RR_PWM_PIN, RR_PWM_CHAN);
  ledcWrite(RR_PWM_CHAN, 0);

  pinMode(RL_MOTOR_FWD, OUTPUT);
  pinMode(RL_MOTOR_BWD, OUTPUT);
  ledcSetup(RL_PWM_CHAN, FREQUENCY_MOTOR, RESOLUTION_MOTOR);
  ledcAttachPin(RL_PWM_PIN, RL_PWM_CHAN);
  ledcWrite(RL_PWM_CHAN, 0);
}

/*
move the robot
:param direction: which direction to move in
*/
void robot_move(Movement direction) {
  if (direction == FORWARDS) {
    set_motor_speed(FR, LINEAR_DUTY_MOTOR);
    set_motor_speed(FL, LINEAR_DUTY_MOTOR);
    set_motor_speed(RR, LINEAR_DUTY_MOTOR);
    set_motor_speed(RL, LINEAR_DUTY_MOTOR);
  } else if (direction == BACKWARDS) {
    set_motor_speed(FR, -LINEAR_DUTY_MOTOR);
    set_motor_speed(FL, -LINEAR_DUTY_MOTOR);
    set_motor_speed(RR, -LINEAR_DUTY_MOTOR);
    set_motor_speed(RL, -LINEAR_DUTY_MOTOR);
  } else if (direction == RIGHT) {
    set_motor_speed(FR, -LINEAR_DUTY_MOTOR);
    set_motor_speed(FL, LINEAR_DUTY_MOTOR);
    set_motor_speed(RR, LINEAR_DUTY_MOTOR);
    set_motor_speed(RL, -LINEAR_DUTY_MOTOR);
  } else {
    set_motor_speed(FR, LINEAR_DUTY_MOTOR);
    set_motor_speed(FL, -LINEAR_DUTY_MOTOR);
    set_motor_speed(RR, -LINEAR_DUTY_MOTOR);
    set_motor_speed(RL, LINEAR_DUTY_MOTOR);
  }
}

/*
move robot in complex pattern
:param speed:  how much to move forwards and backwards
:param turn:   how much to rotate right and left
:param strafe: how much to strafe right and left
*/
void robot_complex_move(int8_t speed, int8_t turn, int8_t strafe) {
  // duty cycles for FR, FL, RR, RL
  int16_t duties[4] = {speed - turn - strafe,
                       speed + turn + strafe,
                       speed - turn + strafe,
                       speed + turn - strafe};

  // find scaling values
  int16_t min = duties[0];
  int16_t max = duties[0];
  for (int i = 1; i < 4; i++) {
    if (duties[i] > max) {
      max = duties[i];
    }
    if (duties[i] < min) {
      min = duties[i];
    }
  }

  // scale
  int16_t bound = (abs(min) > max) ? abs(min) : abs(max);
  if (bound > 90) {
    for (int i = 0; i < 4; i++) {
      duties[i] = map(duties[i], -bound, bound, -90, 90);
    }
  }

  set_motor_speed(FR, duties[0]);
  set_motor_speed(FL, duties[1]);
  set_motor_speed(RR, duties[2]);
  set_motor_speed(RL, duties[3]);
}

/*
rotate the robot
:param direction: which direction to rotate
*/
void robot_rotate(Movement direction) {
  if (direction == RIGHT) {
    set_motor_speed(FR, -ROTATION_DUTY_MOTOR);
    set_motor_speed(FL, ROTATION_DUTY_MOTOR);
    set_motor_speed(RR, -ROTATION_DUTY_MOTOR);
    set_motor_speed(RL, ROTATION_DUTY_MOTOR);
  } else if (direction == LEFT) {
    set_motor_speed(FR, ROTATION_DUTY_MOTOR);
    set_motor_speed(FL, -ROTATION_DUTY_MOTOR);
    set_motor_speed(RR, ROTATION_DUTY_MOTOR);
    set_motor_speed(RL, -ROTATION_DUTY_MOTOR);
  }
}

// stop the robot
void robot_stop() {
  set_motor_speed(FR, 0);
  set_motor_speed(FL, 0);
  set_motor_speed(RR, 0);
  set_motor_speed(RL, 0);
}

/*
set the speed of a motor
:param motor: given motor
:param duty: duty to set
*/
void set_motor_speed(Motor motor, int8_t duty) {
  // assign forwards and packwards pins and PWM channel
  uint8_t fwd_pin, bwd_pin, pwm_chan;
  if (motor == FR) {
    fwd_pin = FR_MOTOR_FWD;
    bwd_pin = FR_MOTOR_BWD;
    pwm_chan = FR_PWM_CHAN;
  } else if (motor == FL) {
    fwd_pin = FL_MOTOR_FWD;
    bwd_pin = FL_MOTOR_BWD;
    pwm_chan = FL_PWM_CHAN;
  } else if (motor == RR) {
    fwd_pin = RR_MOTOR_FWD;
    bwd_pin = RR_MOTOR_BWD;
    pwm_chan = RR_PWM_CHAN;
  } else {
    fwd_pin = RL_MOTOR_FWD;
    bwd_pin = RL_MOTOR_BWD;
    pwm_chan = RL_PWM_CHAN;
  }

  // set speed and direction
  if (duty == 0) {
    digitalWrite(fwd_pin, LOW);
    digitalWrite(bwd_pin, LOW);
  } else if (duty > 0) {
    digitalWrite(fwd_pin, HIGH);
    digitalWrite(bwd_pin, LOW);
  } else {
    digitalWrite(fwd_pin, LOW);
    digitalWrite(bwd_pin, HIGH);
  }

  ledcWrite(pwm_chan, map(abs(duty), 0, 100, 0, MAX_DUTY_MOTOR));
}

/*
ROBOT CLAW METHODS
*/

// Hardware Setup: Claw
void claw_setup() {
  ledcSetup(CLAW_PWM_CHAN, FREQUENCY_SERVO, RESOLUTION_SERVO);
  ledcAttachPin(CLAW_PWM_PIN, CLAW_PWM_CHAN);
  set_claw(CLOSED);
}

/*
open or close the claw
:param status: status to set the claw to
*/
void set_claw(ClawStatus status) {
  if (status == OPEN) {
    ledcWrite(CLAW_PWM_CHAN, DUTY_OPEN);
  } else {
    ledcWrite(CLAW_PWM_CHAN, DUTY_CLOSED);
  }
}

/*
MOVEMENT TEMPLATE
*/

/*
TEMPLATE FOR ALL MOVEMENT METHODS
:param parameter_1:            first parameter
:param parameter_2:            second parameter
:param stop_condition_reached: function to determine when objective is reached
:param heading_reached:        function to determine when required heading is achieved
:param change_heading:         function to determine how to change heading to achieve required heading
:param robot_stuck:            function to determine if robot is stuck
:return: if routine completed successfully
*/
bool move_to_template(
    uint16_t parameter_1,
    uint16_t parameter_2,
    bool (*stop_condition_reached)(uint16_t, uint16_t),
    bool (*heading_reached)(uint16_t, uint16_t),
    void (*change_heading)(uint16_t, uint16_t),
    bool (*robot_stuck)(unsigned long)
) {
  // move while stop condition is not reached
  unsigned long global_start = millis();
  while (!stop_condition_reached(parameter_1, parameter_2) &&
         !robot_stuck(global_start)) {
    // rotate while required heading is not achieved
    while (!heading_reached(parameter_1, parameter_2) &&
           !robot_stuck(global_start)) {
      change_heading(parameter_1, parameter_2);
    }
    robot_stop();

    // move while required heading is achieved
    robot_move(FORWARDS);
    while (heading_reached(parameter_1, parameter_2) &&
           !stop_condition_reached(parameter_1, parameter_2) &&
           !robot_stuck(global_start));
    robot_stop();
  }
  robot_stop();
  return !robot_stuck(global_start);
}