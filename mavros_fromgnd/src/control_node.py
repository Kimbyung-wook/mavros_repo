#!/usr/bin/env python
import ros, rospy
from std_msgs.msg import *
from mavros_commander import MavrosCommander
from mavros_fromgnd.msg import log2comm
from mavros_fromgnd.srv import ctrl_srv
from pymavlink import mavutil


class ControlNode():
  def __init__(self,name="control_node"):
    self.mavros_controller    = MavrosCommander()
    self.control_cmd_received = "ONGROUND" # now control state
    self.control_state        = "ONGROUND" # now control state
    self.position_cmd         = [0.0,0.0,0.0,0.0]
    self.position             = [0.0,0.0,0.0,0.0]
    self.JumpRange            = 5.0
    self.TurnRange            = 30.0

    # Publisher
    self.log2comm_pub         = rospy.Publisher(node_name+"/log2comm",log2comm,queue_size=10)
    # Service Server
    self.control_cmd_srv      = rospy.Service(node_name+"/control_cmd",ctrl_srv, self.ctrl_cb)
    
  def setup(self):
    self.mavros_controller.setup()
    print(self.mavros_controller.sub_topics_ready)
    self.mavros_controller.set_TKO_SPEED(0.5)
    self.mavros_controller.set_XY_VEL_MAX(2.0)
    self.mavros_controller.set_MPC_YAWRAUTO_MAX(5.0)
    self.mavros_controller.wait_for_topics(5)
    self.mavros_controller.wait_for_landed_state(
                                mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                5, -1)

  def ctrl_cb(self,req):
    self.control_cmd_received=req.control_cmd
    rospy.loginfo("Order Received " + str(self.control_cmd_received))
    return self.state_transition()

  def state_transition(self):
    if self.is_ONGROUND(): # ON GROUND
      if self.is_control_cmd("TAKEOFF"):
        return self.do_TAKEOFF()
      else:
        return False

    elif self.is_STANDBY(): # STANDBY
      if self.is_control_cmd("LANDING"):
        return self.do_LANDING()
      elif self.is_control_cmd("FORWARD"):
        return self.go_FORWARD()
      elif self.is_control_cmd("BACKWARD"):
        return self.go_BACKWARD()
      elif self.is_control_cmd("RIGHTSIDE"):
        return self.go_RIGHTSIDE()
      elif self.is_control_cmd("LEFTSIDE"):
        return self.go_LEFTSIDE()
      elif self.is_control_cmd("UPWARD"):
        return self.go_UPWARD()
      elif self.is_control_cmd("DOWNWARD"):
        return self.go_DOWNWARD()
      elif self.is_control_cmd("RIGHTTURN"):
        return self.go_RIGHTTURN()
      elif self.is_control_cmd("LEFTTURN"):
        return self.go_LEFTTURN()
      elif self.is_control_cmd("RTH"):
        return self.go_RTH()
      elif self.is_control_cmd("STOP"):
        return self.go_STOP()
      else:
        return False
    else:
      return False

  def is_ONGROUND(self):
    if self.control_state == "ONGROUND":
      rospy.loginfo("On ground")
      return True
    else:
      # rospy.loginfo("Not on ground")
      return False

  def is_STANDBY(self):
    if self.control_state == "STANDBY":
      rospy.loginfo("Standby")
      return True
    else:
      # rospy.loginfo("Not standby")
      return False

  def is_control_cmd(self,cmd):
    if self.control_cmd_received == cmd:
      print(str(self.control_cmd_received) + " + " + str(cmd))
      return True
    else :
      return False

  def do_TAKEOFF(self):
    rospy.loginfo("do takeoff")
    self.mavros_controller.set_mode("OFFBOARD", 5)
    self.mavros_controller.set_arm(True, 5)
    self.position_cmd = self.mavros_controller.get_local_position()
    self.position = self.position_cmd
    if self.mavros_controller.get_arm(True):
      self.position_cmd[2] = 3.0
      self.position_cmd[3] = self.mavros_controller.get_yaw()
      if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                  self.position_cmd[1],
                                                  self.position_cmd[2],
                                                  self.position_cmd[3], 10):
        self.control_state = "STANDBY"
        self.position = self.position_cmd
        rospy.loginfo("STANDBY")
        return True
    else:
      return False

  def do_LANDING(self):
    rospy.loginfo("do landing")
    self.mavros_controller.set_mode("AUTO.LAND", 5)
    if self.mavros_controller.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0):
      self.mavros_controller.set_arm(False, 5)
      self.control_state = "ONGROUND"
      self.position_cmd = self.mavros_controller.get_local_position()
      self.position = self.position_cmd
      rospy.loginfo("LANDING")
      return True
    else:
      return False

  def go_FORWARD(self):
    self.position_cmd[0] += self.JumpRange
    rospy.loginfo("go Forward")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_BACKWARD(self):
    self.position_cmd[0] += -self.JumpRange
    rospy.loginfo("go Backward")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_RIGHTSIDE(self):
    self.position_cmd[1] += -self.JumpRange
    rospy.loginfo("go RightSide")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False
      
  def go_LEFTSIDE(self):
    self.position_cmd[1] += +self.JumpRange
    rospy.loginfo("go LeftSide")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_UPWARD(self):
    self.position_cmd[2] += 2.0
    rospy.loginfo("go Upward")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_DOWNWARD(self):
    if self.position[2] < 2.0:
      return False
      
    self.position_cmd[2] += -2.0
    rospy.loginfo("go Downward")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_RIGHTTURN(self):
    self.position_cmd[3] += -self.TurnRange
    rospy.loginfo("go RightTurn")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

  def go_LEFTTURN(self):      
    self.position_cmd[3] += +self.TurnRange
    rospy.loginfo("go LeftTurn")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False
      
  def go_RTH(self):
    self.position_cmd = [0.0, 0.0, 5.0, 0.0]
    rospy.loginfo("go HOME")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False
      
  def go_STOP(self):
    self.position_cmd = self.mavros_controller.get_local_position()
    self.position_cmd = [0.0, 0.0, 5.0, 0.0]
    rospy.loginfo("go HOME")
    if self.mavros_controller.reach_position_yaw(self.position_cmd[0],
                                                self.position_cmd[1],
                                                self.position_cmd[2],
                                                self.position_cmd[3], 10):
      self.control_state = "STANDBY"
      self.position = self.position_cmd
      rospy.loginfo("STANDBY")
      return True
    else:
      return False

    
if __name__ == "__main__":
  node_name = "control_node"
  rospy.init_node(node_name)
  controller = ControlNode()
  # rospy.sleep(9)
  controller.setup()
  rospy.loginfo("Engage : " + node_name)
  rospy.loginfo("Waiting for the order")
  rospy.spin()
