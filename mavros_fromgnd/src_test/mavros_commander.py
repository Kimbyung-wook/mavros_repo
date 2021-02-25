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
    rospy.loginfo("setting FCU parameter: {0} - {1}".format(parameter, value))
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    param_set = False
    try:
      res = self.get_param_srv(parameter)
      if res.success:
        rospy.loginfo(
          "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
          format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                .name, i / loop_freq, timeout))
    except rospy.ServiceException as e:
      rospy.logerr(e)
    loaded_param_name
    for i in xrange(timeout * loop_freq):

      if self.state.armed == arm:
        param_set = True
        rospy.loginfo("set arm success | seconds: {0} of {1}".format(
          i / loop_freq, timeout))
        break
      else:
        try:
          res = self.set_param_srv(parameter,value)
          if not res.success:
            rospy.logerr("failed to send arm command")
        except rospy.ServiceException as e:
          rospy.logerr(e)

      try:
        rate.sleep()
      except rospy.ROSException as e:
        rospy.logfatal(e)

    if True==param_set:
      rospy.logerr("failed to set parameter | param {0} - {1} | timeout(seconds): {2}".
        format(arm, old_arm, timeout))

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

  def reach_position(self, x, y, z, timeout):
      """timeout(int): seconds"""
      # set a position setpoint
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

      if reached==True:
        rospy.logerr("took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
        format(self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z, timeout))



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

  # mavros_commander.log_topic_vars()
  print("set mode")
  mavros_commander.set_mode("OFFBOARD", 5)
  print("set arm")
  mavros_commander.set_arm(True, 5)
  res = mavros_commander.get_param_srv("MPC_XY_VEL_MAX")
  print("Parameter MPC_XY_CRUISE resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_XY_CRUISE")
  print("Parameter MPC_XY_CRUISE resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_TKO_SPEED")
  print("Parameter MPC_TKO_SPEED resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_VEL_MANUAL")
  print("Parameter MPC_VEL_MANUAL resp " + str(res.success) + " value " + str(res.value.real))

  req = ParamValue(0,3.0)
  res = mavros_commander.set_param_srv("MPC_XY_VEL_MAX",req)
  req = ParamValue(0,1.0)
  res = mavros_commander.set_param_srv("MPC_XY_CRUISE",req)
  req = ParamValue(0,0.3)
  res = mavros_commander.set_param_srv("MPC_TKO_SPEED",req)
  req = ParamValue(0,1.0)
  res = mavros_commander.set_param_srv("MPC_VEL_MANUAL",req)

  rospy.sleep(3.0)

  res = mavros_commander.get_param_srv("MPC_XY_VEL_MAX")
  print("Parameter MPC_XY_CRUISE resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_XY_CRUISE")
  print("Parameter MPC_XY_CRUISE resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_TKO_SPEED")
  print("Parameter MPC_TKO_SPEED resp " + str(res.success) + " value " + str(res.value.real))
  res = mavros_commander.get_param_srv("MPC_VEL_MANUAL")
  print("Parameter MPC_VEL_MANUAL resp " + str(res.success) + " value " + str(res.value.real))
  # positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
  #               (0, 0, 20))
  positions = ((50, 50, 20), (50, -50, 20), (-50, -50, 20),
                (0, 0, 20))
  mavros_commander
  for i in xrange(len(positions)):
      mavros_commander.reach_position(positions[i][0], positions[i][1],
                          positions[i][2], 30)

  mavros_commander.set_mode("AUTO.LAND", 5)
  mavros_commander.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                              45, 0)
  mavros_commander.set_arm(False, 5)