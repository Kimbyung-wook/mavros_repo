#pragma once 
#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

class MAVROS_API
{
    public:
    ros::NodeHandle nh;
    MAVROS_API(ros::NodeHandle NodeHandle);
    //MAVROS_API(std::string NodeName);
    ~MAVROS_API();

    bool set_takeoff(float altitude_cmd);
    bool set_landing(float altitude_cmd);
    bool set_arming();
    bool set_disarming();
    bool set_offboard();
    bool set_mode(std::string mode);
    bool set_stabilized();

    // Publisher
    ros::Publisher initPub_setpoint_local_position(uint32_t queue_size);
    ros::Publisher initPub_setpoint_global_position(uint32_t queue_size);

    // Subscriber
    ros::Subscriber initSub_state(uint32_t queue_size);
    ros::Subscriber initSub_battery_state(uint32_t queue_size);
    ros::Subscriber initSub_local_position(uint32_t queue_size);
    ros::Subscriber initSub_global_position(uint32_t queue_size);
    ros::Subscriber initSub_setpoint_local_position(uint32_t queue_size);
    ros::Subscriber initSub_setpoint_global_position(uint32_t queue_size);

    // Callback function
    static void CALLBACK_state(const mavros_msgs::State::ConstPtr &msg);
    static void CALLBACK_battery_state(const sensor_msgs::BatteryState::ConstPtr &msg);
    static void CALLBACK_PoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg);
    static void CALLBACK_local_position(const geometry_msgs::PoseStamped::ConstPtr &msg);
    static void CALLBACK_global_position(const mavros_msgs::GlobalPositionTarget::ConstPtr &msg);

    // Service
    ros::ServiceClient initSrv_set_mode();
    ros::ServiceClient initSrv_arming();
    ros::ServiceClient initSrv_takeoff();
    ros::ServiceClient initSrv_land();
    ros::ServiceClient initSrv_set_home();

    // Topics
    mavros_msgs::State                  get_current_state();
    sensor_msgs::BatteryState           get_battery_state();
    geometry_msgs::PoseStamped          get_current_setpoint_local_position();
    mavros_msgs::GlobalPositionTarget   get_current_setpoint_global_position();
    geometry_msgs::PoseStamped          get_current_local_position();
    mavros_msgs::GlobalPositionTarget   get_current_global_position();
    
    private:
    mavros_msgs::SetMode msg_mode;
    mavros_msgs::CommandBool msg_arm;
    mavros_msgs::CommandTOL msg_takeoff;
    mavros_msgs::CommandTOL msg_land;

    ros::Publisher pub_setpoint_local_position;
    ros::Publisher pub_setpoint_global_position;
    // Subscriber
    ros::Subscriber sub_current_state;
    ros::Subscriber sub_current_battery_state;
    ros::Subscriber sub_current_setpoint_local_position;
    ros::Subscriber sub_current_setpoint_global_position;
    ros::Subscriber sub_current_local_position;
    ros::Subscriber sub_current_global_position;
    // Service
    ros::ServiceClient srv_arming;
    ros::ServiceClient srv_set_home;
    ros::ServiceClient srv_set_mode;
    ros::ServiceClient srv_takeoff;
    ros::ServiceClient srv_land;
    // Topics
    static mavros_msgs::State current_state;
    static sensor_msgs::BatteryState current_battery_state;
    static geometry_msgs::PoseStamped current_local_position;
    static mavros_msgs::GlobalPositionTarget current_global_position;
    geometry_msgs::PoseStamped current_setpoint_local_position;
    mavros_msgs::GlobalPositionTarget current_setpoint_global_position;
    static geometry_msgs::PoseStamped current_PoseStamped;

};