from enum import *

class ControlState(IntEnum):
  FAIL      = 0
  STOP      = 1
  TAKEOFF   = 2
  LANDING   = 3
  STANDBY   = 4
  FORWARD   = 5
  BACKWARD  = 6
  RIGHTSIDE = 7
  LEFTSIDE  = 8

class FlightState(IntEnum):
  FAIL          = 0
  STANDBY_GND   = 1
  STANDBY_AIR   = 2
  ONMISSION     = 3