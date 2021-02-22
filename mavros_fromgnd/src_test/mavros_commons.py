#!/usr/bin/env python2
# PX4 repository의 다음 코드/문서를 참고했습니다.
# This code is refered from following code of PX4-Autopilot repository
# PX4-Autopilot/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
# PX4-Autopilot/integrationtests/python_src/px4_it/mavros/mavros_offboard_posctl_test.py

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu

class MavrosCommon():
  def __init__(self):
    self.test = True

  def setup(self):
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
    self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                    self.altitude_callback)
    self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                          ExtendedState,
                                          self.extended_state_callback)
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
    self.mission_wp_sub = rospy.Subscriber(
        'mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
    self.state_sub = rospy.Subscriber('mavros/state', State,
                                      self.state_callback)


    #
    # Callback functions
    #
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

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

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

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
