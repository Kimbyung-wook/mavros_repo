#include "../include/OFFB_CTRL.h"

using namespace TEST_OFFB;

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_class_test");
    ROS_INFO("Create Node");
    OFFB_CTRL cOFFB;

    ROS_INFO("START");
    ros::Rate loop_rate(10.0);
    cOFFB.set_loop_rate(10.0);
    // cOFFB.set_offboard();
    cOFFB.circle_path_motion(loop_rate,control_mode::POSITION);
    ROS_INFO("END");

}