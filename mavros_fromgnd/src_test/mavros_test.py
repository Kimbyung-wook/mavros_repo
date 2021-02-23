import ros, rospy
import math

from mavros_commons import MavrosCommons


class MavrosComm(MavrosCommons):
  def __init__(self):

    self.test = 1


if __name__=="__main__":
  mavros_comm = MavrosComm()