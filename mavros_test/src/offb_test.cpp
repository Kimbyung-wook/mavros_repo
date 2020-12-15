#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include "rostimer.h"
#include "state_enum.h"
// #include "mavros_api.h"
// #include "../include/rostimer.h"
// #include "../include/rostimer.h"
// #include "../include/mavros_api.h"

mavros_msgs::State msg_current_state;
geometry_msgs::PoseStamped msg_current_local_position;
sensor_msgs::NavSatFix msg_current_global_position;
geometry_msgs::PoseStamped msg_current_setpoint_local_position;
void CALLBACK_state(const mavros_msgs::State::ConstPtr& msg){
    msg_current_state = *msg;
}
void CALLBACK_local_position(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_current_local_position = *msg;
}
void CALLBACK_global_position(const sensor_msgs::NavSatFix::ConstPtr& msg){
    msg_current_global_position = *msg;
}
void CALLBACK_setpoint_local_position(const geometry_msgs::PoseStamped::ConstPtr& msg){
    msg_current_setpoint_local_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "offb_node");
    ros::NodeHandle nh;
    // MAVROS_API api(nh);
    // rostimer test(5.0);

    ROS_INFO("MAVROS NODE boost");
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, CALLBACK_state);
    ros::Subscriber sub_setpoint_local_position = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10, CALLBACK_setpoint_local_position);
    ros::Subscriber sub_global_position = nh.subscribe<sensor_msgs::NavSatFix>
        ("mavros/global_position/global", 10, CALLBACK_global_position);
    ros::Subscriber sub_local_position = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, CALLBACK_local_position);
        
    // Publisher : <template> topic, hz
    ros::Publisher pub_setpoint_local_position = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    // Service Client : 
    ros::ServiceClient srv_arm = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient srv_set_home = nh.serviceClient<mavros_msgs::CommandHome>
        ("mavros/cmd/set_home");
    ros::ServiceClient srv_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/takeoff");
    ros::ServiceClient srv_land = nh.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/land");
    ros::ServiceClient srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    // the setpoint publishing response MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !msg_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // Send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     pub_setpoint_local_position.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::CommandBool msg_arm;
    msg_arm.request.value = true;

    mavros_msgs::SetMode msg_set_mode;
    msg_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandTOL msg_takeoff;
    msg_takeoff.request.latitude = msg_current_global_position.latitude;
    msg_takeoff.request.longitude= msg_current_global_position.longitude;
    msg_takeoff.request.altitude = 3.0;

    char temp[200];
    sprintf(temp,"LLA %9.6f %9.6f %5.1f %6.1f %6.1f",
                msg_takeoff.request.latitude,
                msg_takeoff.request.longitude,
                msg_takeoff.request.altitude,
                msg_takeoff.request.yaw,
                msg_takeoff.request.min_pitch);
    ROS_INFO(temp);
    char state_mode[100];
    sprintf(state_mode,"%s",msg_current_state.mode.c_str());
    sprintf(temp,"Mode : %s / arm %d / sys_status %d",
                state_mode,
                msg_current_state.armed,
                msg_current_state.system_status);
    ROS_INFO(temp);


    ros::Time last_request = ros::Time::now();
    ros::Duration(1.0);
    uint8_t sequence = 0;
    rostimer ticktok(1.0);
    rostimer ticktok_print(1.0);
    ROS_INFO("Start Loop");
    while(ros::ok()){
        if(ticktok_print.check_duration()){
            sprintf(temp,"Tick %s - %6.1f %6.1f %6.1f - %10.6f %10.6f %6.1f",
                msg_current_state.mode.c_str(),
                msg_current_local_position.pose.position.x,
                msg_current_local_position.pose.position.y,
                msg_current_local_position.pose.position.z,
                msg_current_global_position.latitude,
                msg_current_global_position.longitude,
                msg_current_global_position.altitude);
            ROS_INFO(temp);
            ticktok_print.reset_timer();
        }
        switch(sequence)
        {
            case 0: // change to OFFBOARD mode
                if(msg_current_state.mode != "OFFBOARD" && ticktok.check_duration()){
                    if( srv_set_mode.call(msg_set_mode) &&
                        msg_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enable");
                        sequence = 1;
                    }
                    ticktok.reset_timer();
                }
            break;
            case 1: // Arming
                if(!msg_current_state.armed && ticktok.check_duration()){
                    if(srv_arm.call(msg_arm)&&
                        msg_arm.response.success){
                            ROS_INFO("Vehicle armed");
                            sequence = 2;
                    }
                    ticktok.reset_timer();
                }
            break;
            case 2: // takeoff
                if(ticktok.check_duration()){         // Arming && in duration time
                    if(srv_takeoff.call(msg_takeoff)&&msg_takeoff.response.success){ // send request & check response
                        ROS_INFO("[INFO] Vehicle Takeoff");
                        sequence = 3;
                    }
                    ticktok.reset_timer();
                }
            break;
            case 3: // local planner
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}