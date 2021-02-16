import ros, rospy
from std_msgs.msg import *

if __name__ == "__main__":
  node_name = "control_node"
  rospy.init_node(node_name)

  # Publisher
  local_position_sp_pub = rospy.Publisher()

  # Subscriber
  local_position_sub = rospy.Subscriber()
  mode_sub = rospy.Subscriber()
  arm_sub = rospy.Subscriber()


  # Service/Client
  communicator_cli = rospy.ServiceProxy("CtrlCmd",UInt8)
  arm_cli = rospy.ServiceProxy()
  land_cli = rospy.ServiceProxy()
  set_mode_cli = rospy.ServiceProxy()

  # PX4 param
  rospy.set_param()
  