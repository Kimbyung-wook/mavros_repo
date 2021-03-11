#!/usr/bin/env python
#! Test command
# rosservice call /cam_node/cam_OnOFF 1
# rosservice call /cam_node/cam_OnOFF 0
import ros
import rospy
from std_msgs.msg import *

from mavros_fromgnd.srv import *
from mavros_fromgnd.msg import camstatus

class CamModel:
  def __init__(self,node_name="cam_node",hz=10.0):
    self._CamState = False
    self._CamState_prev = False
    self._node_name = node_name
    rospy.init_node(self._node_name)
    self._rate = rospy.Rate(10)

    # Cam Power On/Off Service
    self._cam_server = rospy.Service(node_name+"/cam_power",campower_srv,self.cam_power_cb)
    # Cam State Publisher
    self._cam_state_publisher = rospy.Publisher(node_name+"/cam_state",camstatus,queue_size=10)

  def run(self):
    rospy.loginfo("Ready to take video")
    while not rospy.is_shutdown():
      if self._CamState != self._CamState_prev:
        rospy.loginfo("CamState :" + str(self._CamState))
        self._CamState_prev = self._CamState

      if self._CamState == True:
        self._cam_state_publisher.publish(True)
      else:
        self._cam_state_publisher.publish(False)
      self._rate.sleep()
    
  def cam_power_cb(self,req):
    if req.OnOff != self._CamState:
      self._CamState = req.OnOff
      return True
    else:
      return False

if __name__ == "__main__":
  node_name = "cam_node"
  loop_rate = 10.0
  cam_model = CamModel(node_name,loop_rate)
  cam_model.run()