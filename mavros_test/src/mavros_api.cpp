#include "../include/mavros_api.h"
#include "../include/rostimer.h"

MAVROS_API::MAVROS_API(ros::NodeHandle NodeHandle)
{
    nh = NodeHandle;
}
bool MAVROS_API::set_takeoff(float altitude_cmd = 5.0f){
    ROS_INFO("[INFO] Try Takeoff");
    if(!current_state.armed && !current_state.connected){
        return false;
    }
    msg_takeoff.request.altitude = altitude_cmd;
    rostimer timer(3.0);
    while(ros::ok()){
        if(!current_state.armed && timer.check_duration()){         // Arming && in duration time
            if(srv_takeoff.call(msg_takeoff)&&msg_takeoff.response.success){ // send request & check response
                ROS_INFO("[INFO] Vehicle Takeoff");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to takeoff");
    return false;
}
bool MAVROS_API::set_landing(float altitude_cmd = 5.0f){
    ROS_INFO("[INFO] Try Landing");
    if(!current_state.armed && !current_state.connected){
        return false;
    }
    msg_land.request.altitude = altitude_cmd;
    rostimer timer(3.0);
    while(ros::ok()){
        if(!current_state.armed && timer.check_duration()){         // Arming && in duration time
            if(srv_land.call(msg_land)&&msg_land.response.success){ // send request & check response
                ROS_INFO("[INFO] Vehicle Landing");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to Landing");
    return false;
}
// blocking
bool MAVROS_API::set_arming(){
    ROS_INFO("[INFO] Try Arming");
    if(!current_state.connected){
        return false;
    }
    msg_arm.request.value = true;
    rostimer timer(3.0);
    while(ros::ok()){
        if(!current_state.armed && timer.check_duration()){         // Arming && in duration time
            if(srv_arming.call(msg_arm)&&msg_arm.response.success){ // send request & check response
                ROS_INFO("[INFO] Vehicle armed");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to arm");
    return false;
}
bool MAVROS_API::set_disarming(){
    ROS_INFO("[INFO] Try Disarming");
    msg_arm.request.value = false;
    rostimer timer(3.0);
    while(ros::ok()){
        if(!current_state.armed && timer.check_duration()){         // Arming && in duration time
            if(srv_arming.call(msg_arm)&&msg_arm.response.success){ // send request & check response
                ROS_INFO("[INFO] Vehicle disarmed");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to disarm");
    return false;
}
bool MAVROS_API::set_offboard(){
    ROS_INFO("[INFO] Try Mode Offboard");
    msg_mode.request.custom_mode = "OFFBOARD";
    rostimer timer(3.0);
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && timer.check_duration()){     // check state && in duration time
            if( srv_set_mode.call(msg_mode) && msg_mode.response.mode_sent){// send request & check response
                ROS_INFO("Offboard enable");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to offboard");
    return false;
}
bool MAVROS_API::set_mode(std::string modename){
    msg_mode.request.custom_mode = modename;
    rostimer timer(3.0);
    while(ros::ok()){
        if(current_state.mode != modename && timer.check_duration()){     // check state && in duration time
            if( srv_set_mode.call(msg_mode) && msg_mode.response.mode_sent){// send request & check response
                char temp[200];
                ROS_INFO("flight mode changed to");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to change mode");
    return false;
}
bool MAVROS_API::set_stabilized(){
    ROS_INFO("[INFO] Try Mode Stabilized");
    msg_mode.request.custom_mode = "OFFBOARD";
    rostimer timer(3.0);
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && timer.check_duration()){     // check state && in duration time
            if( srv_set_mode.call(msg_mode) && msg_mode.response.mode_sent){// send request & check response
                ROS_INFO("Stabilized enable");
                return true;
            }
        }
    }
    ROS_INFO("[ERROR] Failed to Stabilized");
    return false;
}

// Publisher
ros::Publisher MAVROS_API::initPub_setpoint_local_position(uint32_t queue_size = 10){
    pub_setpoint_local_position = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", queue_size);
}
ros::Publisher MAVROS_API::initPub_setpoint_global_position(uint32_t queue_size = 10){
    pub_setpoint_global_position = nh.advertise<mavros_msgs::GlobalPositionTarget>
        ("mavros/setpoint_position/global", queue_size);
}

// Subscriber
ros::Subscriber MAVROS_API::initSub_state(uint32_t queue_size){
    sub_current_state = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/state", queue_size, CALLBACK_state);
}
ros::Subscriber MAVROS_API::initSub_battery_state(uint32_t queue_size){
    sub_current_battery_state = nh.subscribe<sensor_msgs::BatteryState>
        ("sensor_msgs/BatteryState", queue_size, CALLBACK_battery_state);
}
ros::Subscriber MAVROS_API::initSub_local_position(uint32_t queue_size){
    sub_current_setpoint_local_position = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", queue_size, CALLBACK_local_position);
}
ros::Subscriber MAVROS_API::initSub_global_position(uint32_t queue_size){
    sub_current_setpoint_global_position = nh.subscribe<mavros_msgs::GlobalPositionTarget>
        ("mavros/setpoint_position/global", queue_size, CALLBACK_global_position);
}
ros::Subscriber MAVROS_API::initSub_setpoint_local_position(uint32_t queue_size){
    sub_current_local_position = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", queue_size, CALLBACK_local_position);
}
ros::Subscriber MAVROS_API::initSub_setpoint_global_position(uint32_t queue_size){
    sub_current_global_position = nh.subscribe<mavros_msgs::GlobalPositionTarget>
        ("mavros/global_position/global", queue_size, CALLBACK_global_position);
}
// Callback function
void MAVROS_API::CALLBACK_state(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}
void MAVROS_API::CALLBACK_battery_state(const sensor_msgs::BatteryState::ConstPtr&msg){
    current_battery_state = *msg;
}
void MAVROS_API::CALLBACK_PoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_PoseStamped = *msg;
}
void MAVROS_API::CALLBACK_local_position(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_position = *msg;
}
void MAVROS_API::CALLBACK_global_position(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg){
    current_global_position = *msg;
}
// Service Client
ros::ServiceClient MAVROS_API::initSrv_arming(){
    srv_arming = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    return srv_arming;
}
ros::ServiceClient MAVROS_API::initSrv_set_home(){
    srv_set_home = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/set_home");
    return srv_set_home;
}
ros::ServiceClient MAVROS_API::initSrv_set_mode(){
    srv_set_mode = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/set_mode");
    return srv_set_mode;
}
ros::ServiceClient MAVROS_API::initSrv_takeoff(){
    srv_takeoff = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/takeoff");
    return srv_takeoff;
}
ros::ServiceClient MAVROS_API::initSrv_land(){
    srv_land = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/land");
    return srv_land;
}

// get Topic msgs
mavros_msgs::State MAVROS_API::get_current_state(){
    return current_state;
}
geometry_msgs::PoseStamped MAVROS_API::get_current_setpoint_local_position(){
    return current_setpoint_local_position;
}
geometry_msgs::PoseStamped MAVROS_API::get_current_local_position(){
    return current_local_position;
}
mavros_msgs::GlobalPositionTarget MAVROS_API::get_current_setpoint_global_position(){
    return current_setpoint_global_position;
}
mavros_msgs::GlobalPositionTarget MAVROS_API::get_current_global_position(){
    return current_global_position;
}