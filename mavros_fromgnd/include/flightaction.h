enum BasicStatus
{
  FAIL = 0,
  ONGROUND,  // Stand by on the ground
  TAKEOFF,  // Take off
  LANDING,  // Landing
  ONAIR,    // On the air
};

enum FlightStatus
{
  FAIL = 0,
  STANDBY,
  ONMISSION,
};

enum FlightAction
{
  FAIL = 0,
  STOP,
  STANDBY,
  FORWARD,
  BACKWARD,
  RIGHTSIDE,
  LEFTSIDE,
  UPSIDE,
  DOWNSIDE,
};