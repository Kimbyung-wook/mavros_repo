import ros, rospy
import sys


from mavros_fromgnd.srv import *
from std_msgs.msg import *

class FakeSerialModel_Client:
  def __init__(self,node_name="fake_serial",hz=10.0):
    self._node_name = node_name
    self._hz = hz
    rospy.init_node(self._node_name)
    self._rate = rospy.Rate(self._hz)

    self._fake_seial_srv = rospy.ServiceProxy(self._node_name+"/topics",fakestr_srv)

  def run(self,msgstr):
    if not rospy.is_shutdown():
      print("Send : " + msgstr)
      self._fake_seial_srv(msgstr)

if __name__=="__main__":
  node_name = "fake_serial"
  hz = 10.0
  
  if len(sys.argv) == 2:  # Port
    msgstr = sys.argv[1]
    rospy.loginfo("Send Message {0}".format(msgstr))
  else:
    rospy.loginfo("Not enough arguments!")
    exit()
  
  fake_serial_model = FakeSerialModel_Client(node_name,hz)
  fake_serial_model.run(msgstr)
