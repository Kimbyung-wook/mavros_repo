#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu

class OffbNode:
  def __init__(self):
    self.altitude = Altitude()
    self.extended_state = ExtendedState()
    self.global_position = NavSatFix()
    self.imu_data = Imu()
    self.home_position = HomePosition()
    self.local_position = PoseStamped()
    self.mission_wp = WaypointList()
    self.state = State()
    self.mav_type = None

    self.sub_topics_ready = {
        key: False
        for key in [
            'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
            'mission_wp', 'state', 'imu'
        ]
    }

    # ROS services
    service_timeout = 30
    rospy.loginfo("waiting for ROS services")
    try:
        rospy.wait_for_service('mavros/param/get', service_timeout)
        rospy.wait_for_service('mavros/cmd/arming', service_timeout)
        rospy.wait_for_service('mavros/mission/push', service_timeout)
        rospy.wait_for_service('mavros/mission/clear', service_timeout)
        rospy.wait_for_service('mavros/set_mode', service_timeout)
        rospy.loginfo("ROS services are up")
    except rospy.ROSException:
        rospy.logerr("failed to connect to services")
    self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
    self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                              CommandBool)
    self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
    self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                            WaypointClear)
    self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                          WaypointPush)

    # ROS subscribers
    self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                            NavSatFix,
                                            self.global_position_callback)
    self.imu_data_sub = rospy.Subscriber('mavros/imu/data',
                                            Imu,
                                            self.imu_data_callback)
    self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                          HomePosition,
                                          self.home_position_callback)
    self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                          PoseStamped,
                                          self.local_position_callback)
    self.state_sub = rospy.Subscriber('mavros/state', State,
                                      self.state_callback)
  # from mavros_test_common.py
  # Helper methods
  #
  def set_arm(self, arm, timeout):
      """arm: True to arm or False to disarm, timeout(int): seconds"""
      rospy.loginfo("setting FCU arm: {0}".format(arm))
      old_arm = self.state.armed
      loop_freq = 1  # Hz
      rate = rospy.Rate(loop_freq)
      arm_set = False
      for i in xrange(timeout * loop_freq):
          if self.state.armed == arm:
              arm_set = True
              rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                  i / loop_freq, timeout))
              break
          else:
              try:
                  res = self.set_arming_srv(arm)
                  if not res.success:
                      rospy.logerr("failed to send arm command")
              except rospy.ServiceException as e:
                  rospy.logerr(e)

          try:
              rate.sleep()
          except rospy.ROSException as e:
              self.fail(e)

      self.assertTrue(arm_set, (
          "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
          format(arm, old_arm, timeout)))

  def set_mode(self, mode, timeout):
      """mode: PX4 mode string, timeout(int): seconds"""
      rospy.loginfo("setting FCU mode: {0}".format(mode))
      old_mode = self.state.mode
      loop_freq = 1  # Hz
      rate = rospy.Rate(loop_freq)
      mode_set = False
      for i in xrange(timeout * loop_freq):
          if self.state.mode == mode:
              mode_set = True
              rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                  i / loop_freq, timeout))
              break
          else:
              try:
                  res = self.set_mode_srv(0, mode)  # 0 is custom mode
                  if not res.mode_sent:
                      rospy.logerr("failed to send mode command")
              except rospy.ServiceException as e:
                  rospy.logerr(e)

          try:
              rate.sleep()
          except rospy.ROSException as e:
              self.fail(e)

      self.assertTrue(mode_set, (
          "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
          format(mode, old_mode, timeout)))

  # from mavros_test_common.py
  # Callback functions
  #
  def global_position_callback(self, data):
      self.global_position = data

      if not self.sub_topics_ready['global_pos']:
          self.sub_topics_ready['global_pos'] = True

  def imu_data_callback(self, data):
      self.imu_data = data

      if not self.sub_topics_ready['imu']:
          self.sub_topics_ready['imu'] = True

  def home_position_callback(self, data):
      self.home_position = data

      if not self.sub_topics_ready['home_pos']:
          self.sub_topics_ready['home_pos'] = True

  def local_position_callback(self, data):
      self.local_position = data

      if not self.sub_topics_ready['local_pos']:
          self.sub_topics_ready['local_pos'] = True

  def state_callback(self, data):
      if self.state.armed != data.armed:
          rospy.loginfo("armed state changed from {0} to {1}".format(
              self.state.armed, data.armed))

      if self.state.connected != data.connected:
          rospy.loginfo("connected changed from {0} to {1}".format(
              self.state.connected, data.connected))

      if self.state.mode != data.mode:
          rospy.loginfo("mode changed from {0} to {1}".format(
              self.state.mode, data.mode))

      if self.state.system_status != data.system_status:
          rospy.loginfo("system_status changed from {0} to {1}".format(
              mavutil.mavlink.enums['MAV_STATE'][
                  self.state.system_status].name, mavutil.mavlink.enums[
                      'MAV_STATE'][data.system_status].name))

      self.state = data

      # mavros publishes a disconnected state message on init
      if not self.sub_topics_ready['state'] and data.connected:
          self.sub_topics_ready['state'] = True

  def show_status(self):
    rospy.loginfo("Armming {0}".format(self.state.armed))
    rospy.loginfo("Connect {0}".format(self.state.connected))
    rospy.loginfo("Status  {0}".format(self.state.system_status))
    rospy.loginfo("mode    {0}".format(self.state.mode))

  def log_topic_vars(self):
      """log the state of topic variables"""
      rospy.loginfo("========================")
      rospy.loginfo("===== topic values =====")
      rospy.loginfo("========================")
      rospy.loginfo("altitude:\n{}".format(self.altitude))
      rospy.loginfo("========================")
      rospy.loginfo("extended_state:\n{}".format(self.extended_state))
      rospy.loginfo("========================")
      rospy.loginfo("global_position:\n{}".format(self.global_position))
      rospy.loginfo("========================")
      rospy.loginfo("home_position:\n{}".format(self.home_position))
      rospy.loginfo("========================")
      rospy.loginfo("local_position:\n{}".format(self.local_position))
      rospy.loginfo("========================")
      rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
      rospy.loginfo("========================")
      rospy.loginfo("state:\n{}".format(self.state))
      rospy.loginfo("========================")

if __name__ == "__main__":
  rospy.init_node("offb_node", anonymous=True)
  rospy.loginfo("offb_node")
  rate = rospy.Rate(10.0)

  offb = OffbNode()

  rospy.loginfo("")
  last_request = rospy.Time.now()

  try:
    while not rospy.is_shutdown():
      if (rospy.Time.now() - last_request) > rospy.Duration(1.0):
        offb.show_status()
        last_request = rospy.Time.now()
      rate.sleep()
  except rospy.ROSInterruptException:
    print("ROS terminated")
    pass