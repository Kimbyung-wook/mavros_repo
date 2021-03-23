# !/usr/bin/env python2
# This code is refered from following code of PX4-Autopilot repository
# PX4-Autopilot/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
# PX4-Autopilot/integrationtests/python_src/px4_it/mavros/mavros_offboard_posctl_test.py

import ros, rospy
import math
import numpy as np
from threading import Thread
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
            WaypointList, ParamValue
from mavros_msgs.srv import CommandBool, ParamSet, SetMode, WaypointClear, \
            WaypointPush
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import quaternion_from_euler
from pymavlink import mavutil
from six.moves import xrange

from mavros_commons import MavrosCommons

class MavrosCommander(MavrosCommons):
  def __init__(self):
    super(MavrosCommander, self).__init__()

  def setup(self):
    super(MavrosCommander, self).setup()
    self.pos = PoseStamped()
    self.radius = 1

    self.pos_setpoint_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=1)

    self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
    # send setpoints in seperate thread to better prevent failsafe
    self.pos_thread = Thread(target=self.send_pos, args=())
    self.pos_thread.daemon = True
    self.pos_thread.start()
    print("mavros_commander setup")

  #
  # Helper methods
  #
  def set_param(self, parameter, value, timeout):
    """set Parameters: , timeout(int): seconds"""
    param_get = False
    param_set = False
    prev_param = 0
    now_param = 0
    # Get prev_param
    # rospy.loginfo("get FCU parameter: {0} to {1}".format(parameter, value))
    loop_freq = 2  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
      # Get parameter
      if False == param_get:
        try:
          res = self.get_param_srv(parameter)
          if res.success == True:
            if res.value.integer != 0:
              prev_param = res.value.integer
            else:
              prev_param = res.value.real
            param_get = True
        except rospy.ServiceException as e:
          rospy.logerr(e)
      # Set pararmeter
      if (True == param_get) & (False == param_set):
        try:
          req = ParamValue(0,0.)
          if type(value) == int:
            req = ParamValue(value,0.0)
          else:
            req = ParamValue(0,value)
          res = self.set_param_srv(parameter,req)
          if res.success == True:
            if res.value.integer != 0:
              now_param = res.value.integer
            else:
              now_param = res.value.real
            rospy.loginfo(
              "Param changed | {0} : {1:.4} -> {2:.4} | seconds: {3} of {4}".
              format(parameter, prev_param, now_param, i / loop_freq, timeout))
            param_set = True
        except rospy.ServiceException as e:
          rospy.logerr(e)
      # All Done?
      if (param_set == True) & (param_get == True):
        break
      # sleep
      try:
        rate.sleep()
      except rospy.ROSException as e:
        rospy.logfatal(e)
    # Didn't you do that?
    if (param_set == False) | (param_get == False):
      rospy.logerr("failed to set parameter | param {0} | timeout(seconds): {1}".
        format(parameter, timeout))

  def send_pos(self):
      rate = rospy.Rate(10)  # Hz
      self.pos.header = Header()
      self.pos.header.frame_id = "base_footprint"

      while not rospy.is_shutdown():
          self.pos.header.stamp = rospy.Time.now()
          self.pos_setpoint_pub.publish(self.pos)
          try:  # prevent garbage in console output when thread is killed
              rate.sleep()
          except rospy.ROSInterruptException:
              pass

  def is_at_position(self, x, y, z, offset):
      """offset: meters"""
      rospy.logdebug(
          "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
              self.local_position.pose.position.x, self.local_position.pose.
              position.y, self.local_position.pose.position.z))

      desired = np.array((x, y, z))
      pos = np.array((self.local_position.pose.position.x,
                      self.local_position.pose.position.y,
                      self.local_position.pose.position.z))
      return np.linalg.norm(desired - pos) < offset

  def reach_position_yaw(self, x, y, z, yaw_in, timeout):
    """
    x, y, z(float) : m
    timeout(int): seconds
    """
    # set a position setpoint
    self.pos.pose.position.x = x
    self.pos.pose.position.y = y
    self.pos.pose.position.z = z
    yaw_degrees = yaw_in # North
    yaw = math.radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)
    self.pos.pose.orientation = Quaternion(*quaternion)
    rospy.loginfo("target  position | x: {0:.2f}, y: {1:.2f}, z: {2:.2f}, yaw: {3:.2f}".
                format( x, y, z, yaw_degrees))
    rospy.loginfo("current position | x: {0:.2f}, y: {1:.2f}, z: {2:.2f}, yaw: {3:.2f}".
                format(
                self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z,
                self.local_position.pose.orientation.z))

    # does it reach the position in 'timeout' seconds?
    loop_freq = 2  # Hz
    rate = rospy.Rate(loop_freq)
    reached = False
    for i in xrange(timeout * loop_freq):
        if self.is_at_position(self.pos.pose.position.x,
                                self.pos.pose.position.y,
                                self.pos.pose.position.z, self.radius):
            rospy.loginfo("position reached | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            reached = True
            break

        try:
            rate.sleep()
        except rospy.ROSException as e:
            rospy.logerr('ROSException')

    if reached==False:
      rospy.logerr("took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f}, yaw: {3:.2f} | timeout(seconds): {4}".
      format(self.local_position.pose.position.x,
              self.local_position.pose.position.y,
              self.local_position.pose.position.z,
              self.local_position.pose.orientation.z, timeout))
      return False
    else:
      return True

  def reach_position(self, x, y, z, timeout):
    """
    x, y, z(float) : m
    timeout(int): seconds
    """
    self.pos.pose.position.x = x
    self.pos.pose.position.y = y
    self.pos.pose.position.z = z
    rospy.loginfo(
        "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
        format(x, y, z, self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z))

    # For demo purposes we will lock yaw/heading to north.
    yaw_degrees = 0  # North
    yaw = math.radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)
    self.pos.pose.orientation = Quaternion(*quaternion)

    # does it reach the position in 'timeout' seconds?
    loop_freq = 2  # Hz
    rate = rospy.Rate(loop_freq)
    reached = False
    for i in xrange(timeout * loop_freq):
        if self.is_at_position(self.pos.pose.position.x,
                                self.pos.pose.position.y,
                                self.pos.pose.position.z, self.radius):
            rospy.loginfo("position reached | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            reached = True
            break

        try:
            rate.sleep()
        except rospy.ROSException as e:
            rospy.logerr('ROSException')

    if reached==False:
      rospy.logerr("took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
      format(self.local_position.pose.position.x,
              self.local_position.pose.position.y,
              self.local_position.pose.position.z, timeout))
      return False
    else:
      return True
      
if __name__=="__main__":
  rospy.init_node("test_for_mavros_commander")
  print("mavros_commander")
  mavros_commander = MavrosCommander()
  mavros_commander.setup()
  
  """Test offboard position control"""
  # make sure the simulation is ready to start the mission
  mavros_commander.wait_for_topics(5)
  mavros_commander.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                              5, -1)

  mavros_commander.set_mode("OFFBOARD", 5)
  mavros_commander.set_param("MPC_XY_VEL_MAX", 3.0,2)
  mavros_commander.set_param("MPC_XY_CRUISE", 3.0,2)
  mavros_commander.set_param("MPC_TKO_SPEED", 0.05,2)
  mavros_commander.set_param("MPC_YAWRAUTO_MAX", 30.0,2)
  mavros_commander.set_arm(True, 2)
  # positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
  #               (0, 0, 20))
  # positions = ((50, 50, 20), (50, -50, 20), (-50, -50, 20),
  #               (0, 0, 20))
  positions = ((0, 0, 3, 0),(10, 10, 5, 30), (10, -10, 5, 60), (-10, -10, 5, 30),
                (0, 0, 5, 0))
  # mavros_commander
  for i in xrange(len(positions)):
    if mavros_commander.get_arm(True):
      mavros_commander.set_arm(True,5)
    mavros_commander.reach_position_yaw(
                positions[i][0],
                positions[i][1],
                positions[i][2],
                positions[i][3],
                30)

  mavros_commander.set_mode("AUTO.LAND", 5)
  mavros_commander.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                              45, 0)
  mavros_commander.set_arm(False, 5)