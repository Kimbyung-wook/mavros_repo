
class MavParam():
  def __init__(self,name,min,max,defaults,unit="",types="FLOAT"):
    self.name = name
    self.min = min
    self.max = max
    self.default = defaults
    self.unit = unit
    self.type = types

class MavparamList():
  def __init__(self):
    self.MPC_LAND_ALT1    = MavParam("MPC_LAND_ALT1"    , 0.0, 122.0, 10.0,"m","FLOAT") # Altitude for 1. step of slow landing (descend)
    self.MPC_LAND_ALT2    = MavParam("MPC_LAND_ALT2"    , 0.0, 122.0,  5.0,"m","FLOAT") # Altitude for 2. step of slow landing (landing)
    self.MPC_LAND_SPEED   = MavParam("MPC_LAND_SPEED"   , 0.6,  10.0,  0.7,"m/s","FLOAT") # Landing descend rate
    self.MPC_MAN_Y_MAX    = MavParam("MPC_MAN_Y_MAX"    , 0.0, 400.0,200.0,"deg/s","FLOAT") # Max manual yaw rate
    self.MPC_TILTMAX_AIR  = MavParam("MPC_TILTMAX_AIR"  ,20.0, 180.0, 45.0,"deg","FLOAT") # Maximum tilt angle in air
    self.MPC_TILTMAX_LND  = MavParam("MPC_TILTMAX_LND"  ,10.0,  90.0, 12.0,"deg","FLOAT") # Maximum tilt during landing
    self.MPC_TKO_SPEED    = MavParam("MPC_TKO_SPEED"    , 1.0,   5.0,  1.5,"m/s","FLOAT") # Takeoff climb rate
    self.MPC_VEL_MANAUL   = MavParam("MPC_VEL_MANUAL"   , 3.0,  20.0, 10.0,"m/s","FLOAT") # Maximum horizontal velocity setpoint for manual controlled mode If velocity setpoint larger than MPC_XY_VEL_MAX is set, then the setpoint will be capped to MPC_XY_VEL_MAX
    self.MPC_XY_CRUISE    = MavParam("MPC_XY_CRUISE"    , 3.0,  20.0,  5.0,"m/s","FLOAT") # Maximum horizontal velocity in mission
    self.MPC_Z_VEL_MAX_DN = MavParam("MPC_Z_VEL_MAX_DN" , 0.5,   4.0,  1.0,"m/s","FLOAT") # Maximum vertical descent velocity
    self.MPC_Z_VEL_MAX_UP = MavParam("MPC_Z_VEL_MAX_UP" , 0.5,   8.0,  3.0,"m/s","FLOAT") # Maximum vertical ascent velocity
    # self.MPC_VEL_MANAUL = "MPC_VEL_MANUAL"
    # self.MPC_XY_CRUISE = "MPC_XY_CRUISE"
