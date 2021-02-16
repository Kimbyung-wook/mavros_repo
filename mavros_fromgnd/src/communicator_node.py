# Terminal 1 - minicom
#  and type anything
# Terminal 2
#  python communicator_node /dev/ttyUSB0 57600

import serial
from serial import Serial, SerialException, SerialTimeoutException
import ros
import rospy
import sys


class Communicator():
  def __init__(self, port=None, baud=57600, timeout=3.0):
    self.timeout = timeout
    
    if port is None:
      # no port specified, listen for any new port?
      pass
    elif hasattr(port, 'read'):
      #assume its a filelike object
      self.port=port
    else:
      # open a specific port
      while not rospy.is_shutdown():
        try:
          self.port = Serial(port, baud, timeout=self.timeout)
          break
        except SerialException as e:
          rospy.logerr("Error opening serial: %s", e)

    if rospy.is_shutdown():
        return

  def tryRead(self, length):
    try:
      bytes_remaining = length
      result = bytearray()
      while bytes_remaining != 0 :
        received = self.port.read(bytes_remaining)
        if len(received) != 0:
          self.last_read = rospy.Time.now()
          result.extend(received)
          bytes_remaining -= len(received)

      if bytes_remaining != 0:
        raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

      return bytes(result)
    except Exception as e:
      raise IOError("Serial Port read failure: %s" % e)


if __name__ == "__main__":
  rospy.init_node("communicator_node")
  rate = rospy.Rate(10)

  baudrate = 921600
  port_name = "/dev/ttyUSB0"
  serial_timeout = 0.0

  if len(sys.argv) == 2:  # Port
    port_name = sys.argv[1]
    rospy.loginfo("Port Name {0}".format(port_name))
  elif len(sys.argv) == 3: # Port and baudrate
    port_name = sys.argv[1]
    rospy.loginfo("Port Name {0}".format(port_name))
    baudrate = sys.argv[2]
    rospy.loginfo("Baudrate {0}".format(baudrate))
  else:
    rospy.loginfo("Not enough arguments!")
    exit()
  
  try:
    port = serial.Serial(port_name,baudrate,timeout=serial_timeout)
  except SerialException as e:
    rospy.logerr("Error opening serial: %s", e)
    pass

  start_time = rospy.Time.now()
  Timeidx = 1.0
  bytes_remaining = 100
  while not rospy.is_shutdown():
    #
    # Show States
    #
    if rospy.Time.now() - start_time > rospy.Duration(Timeidx):
      rospy.loginfo("Time! {0:.0f}".format(Timeidx))
      port.write("Time! {0:.0f}\n".format(Timeidx))
      Timeidx += 1.0
    
    #
    # Read from serial port
    #
    received = port.read(bytes_remaining)
    if len(received) is not 0:
      rospy.loginfo("remaining {0}, received {1}".format(bytes_remaining,received))
    
    # Read from mavros_node

    # Read from cam_node

    # Read from 

    # sleep
    rate.sleep()