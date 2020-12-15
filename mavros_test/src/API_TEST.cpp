#include<ros/ros.h>
#include "MAVROS_API.h"
#include "rostimer.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "API_TEST");
    ros::NodeHandle nh;
    // MAVROS_API ros_api(nh);


    // regist pub-sub
    // ros_api.initPub_setpoint_local_position(10);
    // ros_api.initSub_state(10);
    // ros_api.initSub_battery_state(10);
    // ros_api.initSub_local_position(10);
    // ros_api.initSub_global_position(10);
    // ros_api.initSub_setpoint_global_position(10);
    // ros_api.initSub_setpoint_local_position(10);

    // // regist service
    // ros_api.initSrv_arming();
    // ros_api.initSrv_set_mode();
    // ros_api.initSrv_takeoff();
    // ros_api.initSrv_land();

    // // the setpoint publishing raresponsee MUST be faster than 2Hz
    // ros::Rate rate(20.0);

    // // wait for FCU connection
    // rostimer timer_2connect(1.0);
    // while(ros::ok() && !ros_api.get_current_state().connected){
    //     ros::spinOnce();
    //     rate.sleep();
    //     if(!timer_2connect.check_duration()){
    //         ROS_INFO("Wait for connect..");
    //         timer_2connect.start_timer();
    //     }
    // }

    // // 
    // ros_api.set_arming();
    // ros_api.set_offboard();
    // ros_api.set_takeoff(10.0);


    // ros_api.set_landing(10.0);


}