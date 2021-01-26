/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

mavros_msgs::State current_state;
sensor_msgs::Imu current_imu;
sensor_msgs::NavSatFix current_global_pos;
geometry_msgs::PoseStamped current_setpoint_local_pos;
geometry_msgs::PoseStamped current_local_pos;
// For Subscribing message from px4
void callback_state(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void callback_global_pos(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_global_pos = *msg;
}
void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){
    current_imu = *msg;
}
void callback_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = * msg;
}
void callback_setpoint_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_setpoint_local_pos = * msg;
}
double RangeBetweenPoseStamped(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2){
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    return sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_to2");
    ros::NodeHandle nh;
    ROS_INFO("Create Node");
    // Publisher
    ros::Publisher pub_local_pos = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Subscriber
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, callback_state);
    ros::Subscriber sub_setpoint_local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10, callback_setpoint_local_pos);
    ros::Subscriber sub_local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, callback_local_pos);

    // Service List
    ros::ServiceClient srv_arming = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient srv_land = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    
    ROS_INFO("Connected to topics");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("Wait for FCU connection");
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.pose.position.x = 0;
    msg_pose.pose.position.y = 0;
    msg_pose.pose.position.z = 2;

    ROS_INFO("send setpoints");
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_local_pos.publish(msg_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 1. Set mode to offboard
    mavros_msgs::SetMode msg_mode;
    msg_mode.request.custom_mode = "OFFBOARD";
    // 3. Arming
    mavros_msgs::CommandBool msg_arm;
    msg_arm.request.value = true;
    // 5. Landing
    mavros_msgs::CommandTOL msg_land;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();
    double Time_Duration = 1.0;
    char temp[200];

    enum Transitions : uint8_t{
        standby = 0,
        offboard,
        arm,
        land,
        disarm,
        end,
    };

    uint8_t trans = Transitions::standby;


    ROS_INFO("Program Plan");
    ROS_INFO(" 1. Set Takeoff pose");
    ROS_INFO(" 2. Set offboard");
    ROS_INFO(" 3. Arming (Takeoff)");
    ROS_INFO(" 4. Land");
    ROS_INFO(" 5. Disarm");
    ROS_INFO("Program End");

    ROS_INFO("Start Loop");
    while(ros::ok()){
        double range = RangeBetweenPoseStamped(current_setpoint_local_pos,current_local_pos);
        // Show states
        if(ros::Time::now() - start_time > ros::Duration(Time_Duration)){
            Time_Duration += 1.0;
            // start_time = ros::Time::now();
            // geometry_msgs::Point dXYZ = (current_setpoint_local_pos.pose.position - current_local_pos.pose.position);
            // double range = sqrt(pow(dXYZ.x,2.0)+pow(dXYZ.y,2.0)+pow(dXYZ.z,2.0));
            // double dX = current_setpoint_local_pos.pose.position.x - current_local_pos.pose.position.x;
            // double dY = current_setpoint_local_pos.pose.position.y - current_local_pos.pose.position.y;
            // double dZ = current_setpoint_local_pos.pose.position.z - current_local_pos.pose.position.z;
            // double range = sqrt(pow(dX,2.0)+pow(dY,2.0)+pow(dZ,2.0));
            sprintf(temp,"Tick %5.1f : %s / %d - R %4.2f %4.1f %4.1f %4.1f -> %4.1f %4.1f %4.1f",
                    Time_Duration,
                    current_state.mode.c_str(),
                    current_state.armed,
                    range,
                    current_local_pos.pose.position.x,
                    current_local_pos.pose.position.y,
                    current_local_pos.pose.position.z,
                    current_setpoint_local_pos.pose.position.x,
                    current_setpoint_local_pos.pose.position.y,
                    current_setpoint_local_pos.pose.position.z);
            ROS_INFO(temp);
        }
        switch (trans)
        {
        case Transitions::standby: //****************************************
            trans = Transitions::offboard;
            break;
        case Transitions::offboard: //****************************************
            // State Action
            msg_mode.request.custom_mode = "OFFBOARD";
            if( current_state.mode != "OFFBOARD"){
                if(ros::Time::now() - last_request > ros::Duration(1.0)){
                    if( srv_set_mode.call(msg_mode) &&
                        msg_mode.response.mode_sent){
                        ROS_INFO("[INFO] Offboard enabled");
                    }
                    last_request = ros::Time::now();
                }
            }
            else{
                ROS_INFO("[INFO] To arm");
                trans = Transitions::arm;
            }
            break;
        case Transitions::arm: //****************************************
            // No Offboard
            if( current_state.mode != "OFFBOARD"){
                trans = Transitions::offboard;
                break;
            }
            // State Action
            if( !current_state.armed){
                if(ros::Time::now() - last_request > ros::Duration(1.0)){
                    if( srv_arming.call(msg_arm) &&
                        msg_arm.response.success){
                        ROS_INFO("[INFO] Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            // Transition
            // Ready to Takeoff, wait 10 sec after reach to the target
            if( current_state.mode == "OFFBOARD" && current_state.armed){
                range = RangeBetweenPoseStamped(current_setpoint_local_pos,current_local_pos);
                // Close to the target
                if(range < 1.0){
                    if(ros::Time::now() - last_request > ros::Duration(10.0)){
                        trans = Transitions::land;
                        ROS_INFO("[INFO] To land");
                        last_request = ros::Time::now();
                    }
                }
                // far to the target
                else{
                    last_request = ros::Time::now();
                }
            }
            break;
        case Transitions::land: //****************************************
            // State Action
            if( current_state.mode == "OFFBOARD" && current_state.armed){
                if(ros::Time::now() - last_request > ros::Duration(1.0)){
                    if( srv_land.call(msg_land) &&
                        msg_land.response.success){
                        ROS_INFO("[INFO] Vehicle land");
                    }
                    last_request = ros::Time::now();
                }
            }
            // Transition : Wait 10 sec
            if( current_state.mode == "AUTO.LAND"){
                if(current_local_pos.pose.position.z < 0.1){
                    if(ros::Time::now() - last_request > ros::Duration(10.0)){
                        trans = Transitions::disarm;
                        ROS_INFO("[INFO] To disarm");
                        last_request = ros::Time::now();
                    }
                }
                // far to the target
                else{
                    last_request = ros::Time::now();
                }
            }
            break;
        case Transitions::disarm:
            if( current_state.mode == "AUTO.LAND"){
                if(current_state.armed){
                    msg_arm.request.value = false;
                    if(ros::Time::now() - last_request > ros::Duration(1.0)){
                        if( srv_arming.call(msg_arm) &&
                            msg_arm.response.success){
                            ROS_INFO("[INFO] Vehicle disarmed");
                        }
                        last_request = ros::Time::now();
                    }
                }
                else{
                    trans = Transitions::end;
                }
            }
            else{
                trans = Transitions::land;
            }
        case Transitions::end:
            ROS_INFO("Program End");
            return 0;
            break;
        default:
            break;
        }
            

        pub_local_pos.publish(msg_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}