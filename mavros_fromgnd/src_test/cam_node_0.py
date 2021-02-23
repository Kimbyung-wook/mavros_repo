# Test command
# rosservice call /cam_node/cam_OnOFF 1
# rosservice call /cam_node/cam_OnOFF 0
import ros
import rospy
from std_msgs.msg import *

from mavros_fromgnd.srv import *
from mavros_fromgnd.msg import camstatus

CamState = False
CamState_prev = False

def cam_power_cb(req):
  print("Cam Power CB : " + str(req.OnOff))
  if req.OnOff == True:
    rospy.loginfo("Cam Power On")
    CamState = True
  else:
    rospy.loginfo("Cam Power Off")
    CamState = False
  return campower_srvResponse(True)

if __name__ == "__main__":
  node_name = "cam_node"
  rospy.init_node(node_name)
  rate = rospy.Rate(10)

  # Cam Power On/Off Service
  cam_server = rospy.Service(node_name+"/cam_power",campower_srv,cam_power_cb)
  # Cam State Publisher
  cam_state_publisher = rospy.Publisher(node_name+"/cam_state",camstatus,queue_size=10)

  rospy.loginfo("Ready to take video")
  while not rospy.is_shutdown():
    if CamState != CamState_prev:
      rospy.loginfo("CamState :" + str(CamState))
      CamState_prev = CamState
    if CamState == True:
      cam_state_publisher.publish(True)
    else:
      cam_state_publisher.publish(False)
    rate.sleep()