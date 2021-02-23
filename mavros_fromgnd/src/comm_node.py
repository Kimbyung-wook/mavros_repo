# Terminal 1 - minicom
#  and type anything
# Terminal 2
#  python communicator_node /dev/ttyUSB0 57600

import serial
from serial import Serial, SerialException, SerialTimeoutException
import rospy
import sys
import threading
from ringbuf import RingBuffer

from mavros_fromgnd.srv import *
from mavros_fromgnd.msg import camstatus
from std_msgs.msg import *

class FakeSerialModel_Server:
  def __init__(self):
    self._msgs = bytearray()
    self._fake_serial_server = rospy.Service("fake_serial/topics",fakestr_srv,self.fake_serial_cb)
    
  def read(self,bytes_remaining):
    temp = bytearray()
    if self._msgs.__len__ > bytes_remaining:
      temp = self._msgs
      self._msgs = bytearray()
    else:
      temp = self._msgs[:bytes_remaining]
      self._msgs[bytes_remaining:]
    return temp

  def write(self,str2send):
    rospy.logwarn("Fake Serial Send : "+str2send)

  def fake_serial_cb(self, req):
    fake_serial_read = bytearray(req.msgs)
    self._msgs += fake_serial_read
    return True


class CommunicatorModel:
  def __init__(self,node_name="comm_node",hz=10.0,port_name="/dev/ttyUSB0",baudrate=57600,serial_timeout=0.0,RingBufferSize=10):
    rospy.init_node(node_name)
    self._rate = rospy.Rate(hz)
    self._port_name = port_name
    self._baudrate = baudrate
    self._serial_timeout = serial_timeout
    self._port = FakeSerialModel_Server()

    self._cam_state = False
    self._msg_queue = RingBuffer(RingBufferSize)

    self._cam_state_sub = rospy.Subscriber("cam_node/cam_state",camstatus,self.cam_state_cb)
    self._cam_power_call = rospy.ServiceProxy("cam_node/cam_power",campower_srv)
    if self._port_name[:4] != "fake":
      try:
        self._port = serial.Serial(self._port_name,self._baudrate,timeout=self._serial_timeout)
      except SerialException as e:
        rospy.logerr("Error opening serial: %s", e)
        pass
      
  def run(self):
    start_time = rospy.Time.now()
    Timeidx = 1.0
    while not rospy.is_shutdown():
      #
      # Show States
      #
      if rospy.Time.now() - start_time > rospy.Duration(Timeidx):
        if self._cam_state == True:
          self._cam_power_call(False)
        else:
          self._cam_power_call(True)
        rospy.loginfo("Time! {0:.0f}, Cam {1}".format(Timeidx,self._cam_state))
        self._port.write("Time! {0:.0f}, Cam {1}\n".format(Timeidx,self._cam_state))
        Timeidx += 1.0

      #
      # Read from serial port
      #
      bytes_remaining = 100
      received = self._port.read(bytes_remaining)
      if len(received) is not 0:
        rospy.loginfo("remaining {0}, received {1}".format(bytes_remaining,received))
      self._msg_queue.write(received)
      # Read from mavros_node

      # Read from cam_node

      # Read from 

      # sleep
      self._rate.sleep()


    
  def cam_state_cb(self, req):
    if req.power == True:
      self._cam_state = True
    else:
      self._cam_state = False

if __name__ == "__main__":
  node_name = "comm_node"
  hz = 10.0
  baudrate = 921600
  port_name = "/dev/ttyUSB0"
  serial_timeout = 0.0

  print("Wait for messages")

  #################################################
  # Argumentations
  #################################################
  bUseSerial = False
  if len(sys.argv) == 1:
    bUseSerial = False
    rospy.loginfo("Not use serial port")
  elif len(sys.argv) == 2:  # Port
    bUseSerial = True
    port_name = sys.argv[1]
    rospy.loginfo("Port Name {0}".format(port_name))
  elif len(sys.argv) == 3: # Port and baudrate
    bUseSerial = True
    port_name = sys.argv[1]
    rospy.loginfo("Port Name {0}".format(port_name))
    baudrate = sys.argv[2]
    rospy.loginfo("Baudrate {0}".format(baudrate))
  else:
    rospy.loginfo("Not enough arguments!")
    exit()
  
  comm_model = CommunicatorModel(node_name,hz,port_name,baudrate,serial_timeout)
  comm_model.run()