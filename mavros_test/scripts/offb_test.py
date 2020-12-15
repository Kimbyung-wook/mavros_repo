
import rospy
import thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint
from mavros import command
from tf.transformations import quaternion_from_euler


def _check_ret(args, ret):
    if not ret.success:
        fault("Request failed. Check mavros logs. ACK:", ret.result)

    print_if(args.verbose, "Command ACK:", ret.result)


def do_set_home(args):
    try:
        ret = command.set_home(current_gps=args.current_gps,
                          latitude=args.latitude,
                          longitude=args.longitude,
                          altitude=args.altitude)
    except rospy.ServiceException as ex:
        fault(ex)

    _check_ret(args, ret)


def do_takeoff(args):
    try:
        ret = command.takeoff(min_pitch=args.min_pitch,
                         yaw=args.yaw,
                         latitude=args.latitude,
                         longitude=args.longitude,
                         altitude=args.altitude)
    except rospy.ServiceException as ex:
        fault(ex)

    _check_ret(args, ret)


def do_land(args):
    try:
        ret = command.land(min_pitch=0.0,
                      yaw=args.yaw,
                      latitude=args.latitude,
                      longitude=args.longitude,
                      altitude=args.altitude)
    except rospy.ServiceException as ex:
        fault(ex)

    _check_ret(args, ret)

if __name__ == '__main__':
    rospy.init_node('offb_test', anonymous=True)
    rospy.loginfo("Boot Program")
    mavros.set_namespace()  # initialize mavros module with default namespace
    
    
    rate = rospy.Rate(10)
    rospy.loginfo("Climb")


