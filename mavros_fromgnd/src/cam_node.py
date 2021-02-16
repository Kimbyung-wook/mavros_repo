# Test command
# rosservice call /cam_node/cam_OnOFF 1
# rosservice call /cam_node/cam_OnOFF 0
import ros
import rospy
from std_msgs.msg import *

from mavros_fromgnd.srv import *

CamState = False

def cam_power_cb(req):
  if req.OnOff == True:
    rospy.loginfo("Cam Power On")
    CamState = True
  else:
    rospy.loginfo("Cam Power Off")
    CamState = False
  return Cam_srvResponse(True)

if __name__ == "__main__":
  node_name = "cam_node"
  rospy.init_node(node_name)
  rate = rospy.Rate(10)

  # Cam Power On/Off Service
  cam_server = rospy.Service(node_name+"/cam_OnOFF",Cam_srv,cam_power_cb)
  # Cam State Publisher
  cam_state_publisher = rospy.Publisher(node_name+"/cam_State",Bool,queue_size=10)

  rospy.loginfo("Ready to take video")
  while not rospy.is_shutdown():
    cam_state_publisher.publish(CamState)
    rate.sleep()