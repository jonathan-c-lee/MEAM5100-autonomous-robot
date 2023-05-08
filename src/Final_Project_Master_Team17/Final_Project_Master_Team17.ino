/*
Name: Final_Project_Master_Team17.ino
Author: Jonathan Lee
Description: codebase for master ESP32-S2 for MEAM5100 Final Project
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include "vive510.h"
#include "datatypes.h"
#include "robot_movement.h"

/*
General Interrupt Parameter
*/
portMUX_TYPE local_mux = portMUX_INITIALIZER_UNLOCKED;

/*
Beacon Detection Parameters
*/
#define R_DETECTOR_PIN 5
#define L_DETECTOR_PIN 4
#define TIMES_STORED   6

volatile unsigned long r_detector_times[TIMES_STORED] = {0, 0, 0, 0, 0, 0};
volatile unsigned long l_detector_times[TIMES_STORED] = {0, 0, 0, 0, 0, 0};

// update right detection times
void IRAM_ATTR handle_right_detector() {
  portENTER_CRITICAL_ISR(&local_mux);
  for (int i = 1; i < TIMES_STORED; i++) {
    r_detector_times[i - 1] = r_detector_times[i];
  }
  r_detector_times[TIMES_STORED - 1] = micros();
  portEXIT_CRITICAL_ISR(&local_mux);
}

// update left detection times
void IRAM_ATTR handle_left_detector() {
  portENTER_CRITICAL_ISR(&local_mux);
  for (int i = 1; i < TIMES_STORED; i++) {
    l_detector_times[i - 1] = l_detector_times[i];
  }
  l_detector_times[TIMES_STORED - 1] = micros();
  portEXIT_CRITICAL_ISR(&local_mux);
}

/*
ToF Sensor Parameters
*/
#define FRONT_BEACON_SIGNAL_PIN 9
#define RIGHT_FAR_SIGNAL_PIN    11
#define RIGHT_CLOSE_SIGNAL_PIN  12

/*
Vive Parameters
*/
#define CENTER_VIVE_PIN 15
#define REAR_VIVE_PIN   17

Vive510 vive_center(CENTER_VIVE_PIN);
Vive510 vive_rear(REAR_VIVE_PIN);
hw_timer_t *vive_timer = NULL;
volatile uint16_t robot_location_center[2] = {0, 0};
volatile uint16_t robot_location_rear[2] = {0, 0};
uint16_t field_center[2] = {0, 0};
int8_t opponent_direction = 0;

// get updated robot position from vive
void IRAM_ATTR update_robot_position() {
  portENTER_CRITICAL_ISR(&local_mux);
  if (vive_center.status() == VIVE_RECEIVING) {
    robot_location_center[0] = vive_center.xCoord();
    robot_location_center[1] = vive_center.yCoord();
  }
  if (vive_rear.status() == VIVE_RECEIVING) {
    robot_location_rear[0] = vive_rear.xCoord();
    robot_location_rear[1] = vive_rear.yCoord();
  }
  portEXIT_CRITICAL_ISR(&local_mux);
}

/*
ESP-NOW Communcation Parameters
*/
#define TEAM_NUMBER 17

hw_timer_t *send_timer = NULL;
const esp_now_peer_info_t staff_comm = {
  .peer_addr = {0x84, 0xF7, 0x03, 0xA9, 0x04, 0x78},
  .channel = 1,
  .encrypt = false,
};

// send robot location to staff
void IRAM_ATTR ping_location() {
  portENTER_CRITICAL_ISR(&local_mux);
  uint8_t buffer[13];
  sprintf((char *) buffer, "%2d:%4d,%4d", TEAM_NUMBER, robot_location_center[0] % 10000,
          robot_location_center[1] % 10000);
  esp_now_send(staff_comm.peer_addr, buffer, sizeof(buffer));
  portEXIT_CRITICAL_ISR(&local_mux);
}

/*
UDP Communication Parameters
*/
#define UDP_PORT        2510
#define UDP_PACKET_SIZE 14

WiFiUDP UDP_server;
IPAddress robot_ip_address(192, 168, 1, 131);
const char *ssid = "TP-Link_05AF";
const char *password = "47543454";
uint16_t police_car[2] = {0, 0};

// try to update police car location and return if updated
bool update_police_car() {
  uint8_t buffer[UDP_PACKET_SIZE];
  int message_length = UDP_server.parsePacket();

  // message received with police car location
  if (message_length > 0) {
    buffer[UDP_PACKET_SIZE - 1] = 0;
    UDP_server.read(buffer, UDP_PACKET_SIZE);
    if (atoi((char *) buffer) == 0) {
      police_car[0] = atoi((char *) buffer + 3);
      police_car[1] = atoi((char *) buffer + 8);
      return true;
    }
  }
  return false;
}

// Thresholds
const uint8_t COORD_THRESHOLD = 200;
const float ANGLE_THRESHOLD = 0.15;
const uint8_t STUCK_THRESHOLD = 60;
const uint16_t STALL_THRESHOLD = 6000;

/*
SETUP
  1. Hardware Setup
    a. Movement
    b. Claw
    c. Beacon Detection
    d. Vive
    e. ToF Sensor
  2. Communication Setup
    a. ESP-NOW
    b. UDP
  3. Interrupts
    a. Location Transmission (Timer Interrupt)
    b. Beacon Reading (Pin Interrupt)
    c. Vive Reading (Timer Interrupt)
*/
void setup() {
  tire_motor_setup();
  claw_setup();

  // Hardware Setup: Beacon Detection
  pinMode(R_DETECTOR_PIN, INPUT_PULLUP);
  attachInterrupt(R_DETECTOR_PIN, handle_right_detector, RISING);
  pinMode(L_DETECTOR_PIN, INPUT_PULLUP);
  attachInterrupt(L_DETECTOR_PIN, handle_left_detector, RISING);

  // Hardware Setup: ToF Sensor
  pinMode(FRONT_BEACON_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(RIGHT_FAR_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(RIGHT_CLOSE_SIGNAL_PIN, INPUT_PULLUP);

  // Hardware Setup: Vive
  vive_center.begin();
  vive_rear.begin();
  while (vive_center.status() != VIVE_RECEIVING) {
    vive_center.sync(10);
  }
  if (vive_center.status() == VIVE_RECEIVING) {
    robot_location_center[0] = vive_center.xCoord();
    robot_location_center[1] = vive_center.yCoord();
  }
  while (vive_rear.status() != VIVE_RECEIVING) {
    vive_rear.sync(10);
  }
  if (vive_rear.status() == VIVE_RECEIVING) {
    robot_location_rear[0] = vive_rear.xCoord();
    robot_location_rear[1] = vive_rear.yCoord();
  }

  // Communication Setup: ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != ESP_OK) return;
  if (esp_now_add_peer(&staff_comm) != ESP_OK) return;

  // Communication Setup: UDP
  WiFi.begin(ssid, password);
  WiFi.config(robot_ip_address, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  UDP_server.begin(UDP_PORT);
  while (WiFi.status() != WL_CONNECTED);

  // Communication Setup: Timers and Interrupts for Send, Vive
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &ping_location, true);
  timerAlarmWrite(send_timer, 1000000, true);
  timerAlarmEnable(send_timer);
  vive_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(vive_timer, &update_robot_position, true);
  timerAlarmWrite(vive_timer, 20000, true);
  timerAlarmEnable(vive_timer);

  // capture field center position
  while (!update_police_car());
  field_center[0] = police_car[0];
  field_center[1] = police_car[1];

  // get opponent direction
  while (robot_location_rear[0] == 0);
  opponent_direction = (field_center[0] > robot_location_rear[0]) ? 1 : -1;
}

void loop() {
  delay(10000);
  grab_beacon(700);
  delay(30000);
  push_police_car();
  delay(30000);
  wall_follow();
  while (true);
}

/*
Higher Order Commands
*/

/*
perform wall follow
*/
void wall_follow() {
  set_claw(CLOSED);
  unsigned long start_time = millis();
  int far_signal = digitalRead(RIGHT_FAR_SIGNAL_PIN);
  int close_signal = digitalRead(RIGHT_CLOSE_SIGNAL_PIN);

  // wall follow for 45 seconds
  int8_t speed = 70;
  int8_t turn = 0;
  while ((millis() - start_time) < 45000) {
    delay(4);
    robot_complex_move(speed, turn, 0);
    far_signal = digitalRead(RIGHT_FAR_SIGNAL_PIN);
    close_signal = digitalRead(RIGHT_CLOSE_SIGNAL_PIN);

    // stop turning if no forwards obstruction
    if (!far_signal && !close_signal) {
      if (turn < 0) turn++;
      if (turn > 0) turn--;
      continue;
    }

    // if too far from wall, move towards wall
    if (far_signal) {
      if (turn < 30) turn += 4;
      continue;
    }

    // if too close to wall, move away from wall
    if (close_signal) {
      if (turn > -30) turn -= 4;
      continue;
    }
  }
  robot_stop();
}

/*
grab beacon and move to correct position
:param frequency: frequency of the beacon to grab
*/
void grab_beacon(uint16_t frequency) {
  do {
    set_claw(OPEN);
    search_for_beacon(frequency);
    move_to_beacon(frequency);
    set_claw(CLOSED);
    delay(1000);
    robot_move(BACKWARDS);
    delay(1000);
    set_claw(OPEN);
  } while (!beacon_reached(frequency, 1000000 / frequency));
  robot_stop();
}

/*
scan the field for a beacon
:param frequency: frequency to scan for
*/
void search_for_beacon(uint16_t frequency) {
  static const int16_t offsets[2] = {1000, -1000};
  uint16_t period_micros = 1000000 / frequency;
  for (int i = 0; i < 2; i++) {
    move_to_position(field_center[0], field_center[1] + offsets[i], DIRECT);
    unsigned long start_time = millis();
    while (!beacon_straight_ahead(frequency, period_micros) &&
           ((millis() - start_time) < 20000)) {
      robot_rotate(LEFT);
    }
    robot_stop();
    if (beacon_straight_ahead(frequency, period_micros)) break;    
  }  
}

// push the police car
void push_police_car() {
  move_to_position(field_center[0] - opponent_direction * 1500, field_center[1], DIRECT);
  move_to_position(field_center[0] - opponent_direction * 1000, field_center[1], DIRECT);
  robot_move(FORWARDS);
  while (police_car_moving());
  robot_stop();
}

/*
determine if police car is moving
:return: if police car is moving
*/
bool police_car_moving() {
  static uint16_t previous_location[2] = {police_car[0], police_car[1]};
  static unsigned long stall_time = millis();
  unsigned long current_time = millis();

  update_police_car();
  if (abs(police_car[0] - previous_location[0]) > STUCK_THRESHOLD ||
      abs(police_car[1] - previous_location[1]) > STUCK_THRESHOLD) {
    previous_location[0] = police_car[0];
    previous_location[1] = police_car[1];
    stall_time = current_time;
    return true;
  }
  return (current_time - stall_time) < STALL_THRESHOLD;
}

/*
General Movement and Position Methods
*/

// get robot angle
float get_robot_angle() {
  return atan2((robot_location_center[1]) - robot_location_rear[1], (robot_location_center[0] - robot_location_rear[0]));
}

/*
change heading to reach required heading
:param required_angle: desired heading
*/
void change_heading(float required_angle) {
  float current_angle = get_robot_angle();
  float absolute_difference = abs(current_angle - required_angle);
  if (current_angle > required_angle) {
    (absolute_difference < PI) ? robot_rotate(RIGHT) : robot_rotate(LEFT);
  } else {
    (absolute_difference < PI) ? robot_rotate(LEFT) : robot_rotate(RIGHT);
  }
  delay(25);
  robot_stop();
  if (vive_center.status() != VIVE_RECEIVING) {
    vive_center.sync(3);
  }
  if (vive_rear.status() != VIVE_RECEIVING) {
    vive_rear.sync(3);
  }
}

/*
determine if robot is stuck
:param global_start: time when global state was entered in ms
:return: if robot is in a stuck state
*/
bool robot_stuck(unsigned long global_start) {
  static uint16_t prev_x = robot_location_rear[0];
  static uint16_t prev_y = robot_location_rear[1];
  static unsigned long stall_time = millis();
  unsigned long current_time = millis();

  if ((current_time - global_start) > 120000) {
    return true;
  }

  if (abs(robot_location_rear[0] - prev_x) > STUCK_THRESHOLD ||
      abs(robot_location_rear[1] - prev_y) > STUCK_THRESHOLD) {
    prev_x = robot_location_rear[0];
    prev_y = robot_location_rear[1];
    stall_time = current_time;
    return false;
  }

  return (current_time - stall_time) > STALL_THRESHOLD;
}

/*
move to a given position
:param x:       x coordinate
:param y:       y coordinate
:param pattern: movement pattern to follow
:return: if routine completed successfully
*/
bool move_to_position(uint16_t x, uint16_t y, MovementPattern pattern) {
  if (pattern == XY) {
    return (move_to_coordinate(x, X) && move_to_coordinate(y, Y));
  } else if (pattern == YX) {
    return (move_to_coordinate(y, Y) && move_to_coordinate(x, X));
  } else {
    return move_to_directly(x, y);
  }
}

/*
MOVE TO AXIS
*/

/*
move to a given coordinate along an axis
:param coordinate: coordinate to move to
:param axis:       axis to move along
:return: if routine completed successfully
*/
bool move_to_coordinate(uint16_t coordinate, Axis axis) {
  return move_to_template(
    coordinate,
    axis,
    &coordinate_reached,
    &axis_heading_reached,
    &change_heading_for_coordinate,
    &robot_stuck
  );
}

/*
determine if coordinate is reached
:param coordinate: coordinate to reach
:param axis:       axis to move along
:return: if coordinate has been reached
*/
bool coordinate_reached(uint16_t coordinate, uint16_t axis) {
  if (axis == X) {
    return (abs(coordinate - robot_location_center[0]) < COORD_THRESHOLD);
  }
  return (abs(coordinate - robot_location_center[1]) < COORD_THRESHOLD);
}

/*
determine if required axis heading is reached
:param coordinate: coordinate to reach
:param axis:       axis to move along
:return: if required heading has been reached
*/
bool axis_heading_reached(uint16_t coordinate, uint16_t axis) {
  float current_angle = get_robot_angle();
  float axis_angle = (axis == X) ? PI : PI / 2;
  return (abs(current_angle - axis_angle) < ANGLE_THRESHOLD);
}

/*
change heading to reach required heading
:param coordinate: coordinate to reach
:param axis:       axis to move along
*/
void change_heading_for_coordinate(uint16_t coordinate, uint16_t axis) {
  float axis_angle = (axis == X) ? PI : PI / 2;
  change_heading(axis_angle);
}

/*
MOVE DIRECTLY TO COORDINATES
*/

/*
move to a position following direct path
:param x: x coordinate
:param y: y coordinate
:return: if routine completed successfully
*/
bool move_to_directly(uint16_t x, uint16_t y) {
  return move_to_template(
    x,
    y,
    &coordinates_reached,
    &coordinates_heading_reached,
    &change_heading_for_coordinates,
    &robot_stuck
  );
};

/*
determin if x, y coordinates are reached
:param x: x coordinate
:param y: y coordinate
:return: if coordinates have been reached
*/
bool coordinates_reached(uint16_t x, uint16_t y) {
  return ((abs(x - robot_location_center[0]) < COORD_THRESHOLD) &&
          (abs(y - robot_location_center[1]) < COORD_THRESHOLD));
}

/*
determine if required heading for coordinates is reached
:param x: x coordinate
:param y: y coordinate
:return: if required heading for coordinates has been reached
*/
bool coordinates_heading_reached(uint16_t x, uint16_t y) {
  float current_angle = get_robot_angle();
  float required_angle = atan2((y - robot_location_center[1]), (x - robot_location_center[0]));
  return (abs(current_angle - required_angle) < ANGLE_THRESHOLD);
}

/*
change heading to reach required heading
:param x: x coordinate
:param y: y coordinate
*/
void change_heading_for_coordinates(uint16_t x, uint16_t y) {
  float required_angle = atan2((y - robot_location_center[1]), (x - robot_location_center[0]));
  change_heading(required_angle);
}

/*
MOVE TO BEACON
*/

/*
move to a beacon following direct path
:param frequency: frequency of the beacon
:return: if routine completed successfully
*/
bool move_to_beacon(uint16_t frequency) {
  uint16_t period_micros = 1000000 / frequency;
  return move_to_template(
    frequency,
    period_micros,
    &beacon_reached,
    &beacon_straight_ahead,
    &change_heading_for_beacon,
    &robot_stuck
  );
}

/*
determine if beacon is reached
:param frequency:     beacon frequency
:param period_micros: period in microseconds
:return: if beacon has been reached
*/
bool beacon_reached(uint16_t frequency, uint16_t period_micros) {
  if (!beacon_straight_ahead(frequency, period_micros)) {
    return false;
  }
  return digitalRead(FRONT_BEACON_SIGNAL_PIN) == HIGH;
}

/*
determine if beacon is straight ahead
:param frequency:     beacon frequency
:param period_micros: period in microseconds
:return: if beacon heading has been reached
*/
bool beacon_straight_ahead(uint16_t frequency, uint16_t period_micros) {
  unsigned long current_time = micros();
  uint16_t noise = 10000 / frequency;
  uint16_t lower_bound = period_micros - noise;
  uint16_t upper_bound = period_micros + noise;
  if ((current_time - l_detector_times[TIMES_STORED - 1]) > upper_bound ||
      (current_time - r_detector_times[TIMES_STORED - 1]) > upper_bound) {
    return false;
  }
  uint16_t left_reading = get_detector_period(L);
  uint16_t right_reading = get_detector_period(R);
  return (left_reading < upper_bound && left_reading > lower_bound &&
          right_reading < upper_bound && right_reading > lower_bound);
}

/*
change heading to reach required heading for beacon
:param frequency:     beacon frequency
:param period_micros: period in microseconds
*/
void change_heading_for_beacon(uint16_t frequency, uint16_t period_micros) {
  unsigned long current_time = micros();
  uint16_t noise = 10000 / frequency;
  uint16_t lower_bound = period_micros - noise;
  uint16_t upper_bound = period_micros + noise;
  if ((current_time - r_detector_times[TIMES_STORED - 1]) > upper_bound) {
    robot_rotate(LEFT);
    return;
  }
  uint16_t right_reading = get_detector_period(R);
  if (right_reading < upper_bound && right_reading > lower_bound) {
    robot_rotate(RIGHT);
  } else {
    robot_rotate(LEFT);
  }
}

/*
get period reading from beacon detector
:param detector: given detector
:return: period in micros
*/
unsigned long get_detector_period(Detector detector) {
  volatile unsigned long *times = (detector == R) ? r_detector_times : l_detector_times;
  if ((micros() - times[TIMES_STORED - 1]) > 100000) {
    return 1;
  }
  unsigned long period = 0;
  for (int i = 1; i < TIMES_STORED; i++) {
    unsigned long reading = times[i] - times[i - 1];
    if ((reading > period) && (reading < 1000000)) {
      period = reading;
    }
  }
  return period + 1;
}