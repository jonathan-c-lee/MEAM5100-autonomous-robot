/*
Name: datatypes.h
Author: Jonathan Lee
Description: header file for datatypes (enums)
*/

#ifndef DATATYPES_H
#define DATATYPES_H

/*
Axis Type
  X:      x-axis
  Y:      y-axis
*/
enum Axis {
  X,
  Y
};

/*
Movement Type
  FORWARDS:  move forwards
  BACKWARDS: move backwards
  RIGHT:     move or rotate right
  LEFT:      move or rotate left
*/
enum Movement {
  FORWARDS,
  BACKWARDS,
  RIGHT,
  LEFT
};

/*
Movement Pattern Type
  XY:     move along x-axis, then y-axis
  YX:     move along y-axis, then x-axis
  DIRECT: move directly to position
*/
enum MovementPattern {
  XY,
  YX,
  DIRECT
};

/*
Motor Type
  FR: front right
  FL: front left
  RR: rear right
  RL: rear left
*/
enum Motor {
  FR,
  FL,
  RR,
  RL
};

/*
Claw Status
  OPEN:   claw is open
  CLOSED: claw is closed
*/
enum ClawStatus {
  OPEN,
  CLOSED
};

/*
IR Detector
  R: right detector
  L: left detector
*/
enum Detector {
  R,
  L
};

#endif