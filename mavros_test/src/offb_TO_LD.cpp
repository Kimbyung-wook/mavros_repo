/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Vector3.h>


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
double get_range_to_target(const geometry_msgs::PoseStamped& target, const geometry_msgs::PoseStamped& me){
    
    double dX = target.pose.position.x - me.pose.position.x;
    double dY = target.pose.position.y - me.pose.position.y;
    double dZ = target.pose.position.z - me.pose.position.z;
    double range = sqrt(pow(dX,2.0)+pow(dY,2.0)+pow(dZ,2.0));
    return range;
}
geometry_msgs::PoseStamped speed_limiter(const geometry_msgs::PoseStamped& target, const geometry_msgs::PoseStamped& me,double MaxSpeed = 2.0){
}
geometry_msgs::PoseStamped get_localpose_limitedSpeed(
    const geometry_msgs::PoseStamped& target,
    const geometry_msgs::PoseStamped& from,
    double MaxSpeedXY = 2.0,
    double MaxSpeedZ = 1.0){
    tf::Vector3 targetV2d(target.pose.position.x,target.pose.position.y,0.0);
    tf::Vector3 fromV2d(from.pose.position.x,from.pose.position.y,0.0);
    tf::Vector3 temp = targetV2d - fromV2d;
    geometry_msgs::PoseStamped Scaled_dPose;
    double dXY = temp.length();
    double dZ = target.pose.position.z - from.pose.position.z;
    temp.normalized();
    if(dXY > MaxSpeedXY){
        dXY = MaxSpeedXY;
    }
    Scaled_dPose.pose.position.x = dXY * temp.getX() + from.pose.position.x;
    Scaled_dPose.pose.position.y = dXY * temp.getY() + from.pose.position.y;

    if(abs(dZ) > MaxSpeedZ){
        dZ = dZ * MaxSpeedZ / abs(dZ);
    }
    Scaled_dPose.pose.position.z = target.pose.position.z;
    return Scaled_dPose;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_TO_LD");
    ros::NodeHandle nh;
    ROS_INFO("Create Node");
    // Publisher
    ros::Publisher pub_local_pos = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Subscriber
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, callback_state);
    ros::Subscriber sub_global_pos = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, callback_global_pos);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, callback_imu);
    ros::Subscriber sub_setpoint_local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10, callback_setpoint_local_pos);
    ros::Subscriber sub_local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, callback_local_pos);
    
    // Service List
    ros::ServiceClient srv_arming = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient srv_home = nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/set_home");
    ros::ServiceClient srv_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::ServiceClient srv_land = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    
    ROS_INFO("Connected to topics");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    ROS_INFO("Wait for FCU connection");
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 2;
    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 20;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = 3;
    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 0;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = 4;
    pose = pose1;

    ROS_INFO("send setpoints");
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_local_pos.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 1. Set mode to offboard
    mavros_msgs::SetMode msg_mode;
    msg_mode.request.custom_mode = "OFFBOARD";
    // 2. Set home 
    mavros_msgs::CommandHome msg_set_home;
    msg_set_home.request.current_gps = true;
    // 3. Arming
    mavros_msgs::CommandBool msg_arm;
    msg_arm.request.value = true;
    // 4. Take off
    mavros_msgs::CommandTOL msg_takeoff;
    msg_takeoff.request.latitude    = current_global_pos.latitude;
    msg_takeoff.request.longitude   = current_global_pos.longitude;
    msg_takeoff.request.altitude    = current_global_pos.altitude + 2.0;
    msg_takeoff.request.yaw         = 0.0;
    // 5. Landing
    mavros_msgs::CommandTOL msg_land;
    msg_land.request.latitude    = current_global_pos.latitude;
    msg_land.request.longitude   = current_global_pos.longitude;
    msg_land.request.altitude    = current_global_pos.altitude - 10.0;
    msg_land.request.yaw         = 0.0;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();
    double Time_Duration = 1.0;
    char temp[200];

    enum Transitions : uint8_t{
        standby = 0,
        sethome,
        offb,
        arm,
        to,
        move,
        ld,
        disarm,
    };
    double min_range = 1.0;
    double range = 0.0;
    uint8_t trans = Transitions::standby;
    ROS_INFO("0Start Loop");
    while(ros::ok()){
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
            range = get_range_to_target(current_setpoint_local_pos,current_local_pos);
            sprintf(temp,"Tick %5.1f : %s / %d - %6.1f %6.1f %6.1f to %6.1f",
                    Time_Duration,
                    current_state.mode.c_str(),
                    current_state.armed,
                    current_local_pos.pose.position.x,
                    current_local_pos.pose.position.y,
                    current_local_pos.pose.position.z,
                    range);
            ROS_INFO(temp);
        }
        // Change mode
        switch (trans)
        {
        case Transitions::standby:
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( srv_set_mode.call(msg_mode) &&
                    msg_mode.response.mode_sent){
                    ROS_INFO("1Offboard enabled");
                    trans = Transitions::sethome;
                }
                last_request = ros::Time::now();
            }
            else if(current_state.mode == "OFFBOARD"){
                trans = Transitions::sethome;
            }
            break;
        case Transitions::sethome:
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                if(srv_home.call(msg_set_home) &&
                    msg_set_home.response.success){
                    ROS_INFO("2Set Home");
                    trans = Transitions::offb;
                }
                last_request = ros::Time::now();
            }
        break;
        case Transitions::offb:
            if(!current_state.armed){
                if(ros::Time::now() - last_request > ros::Duration(5.0)){
                    if( srv_arming.call(msg_arm) &&
                        msg_arm.response.success){
                        ROS_INFO("3Vehicle armed");
                        trans = Transitions::arm;
                    }
                    last_request = ros::Time::now();
                }
            }
            else
            {
                trans = Transitions::arm;
            }
        break;
        case Transitions::arm:
            if(current_state.armed){
                pose = pose1;
                if(ros::Time::now() - last_request > ros::Duration(5.0)){
                    ROS_INFO("4publish local pose");
                    // pose = get_localpose_limitedSpeed(pose1,current_local_pos);
                    pose = pose1;  
                    range = get_range_to_target(pose,current_local_pos);
                    if(range < min_range){
                        ROS_INFO("4Change setpoint");
                        trans = Transitions::to;
                    }
                    last_request = ros::Time::now();
                } 
            }
            else
            {   // if vehicle is disarmed, try to arm
                ROS_INFO("Disarmed. retry arm");
                trans = Transitions::offb;
            }
            break;
        case Transitions::to:
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("5publish local pose");
                // pose = get_localpose_limitedSpeed(pose2,current_local_pos);
                pose = pose2;
                range = get_range_to_target(pose,current_local_pos);
                if(range < min_range){
                    ROS_INFO("5Change setpoint");
                    trans = Transitions::move;
                }
                last_request = ros::Time::now();
            } 
        break;
        case Transitions::move:
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                ROS_INFO("6publish local pose");
                // pose = get_localpose_limitedSpeed(pose3,current_local_pos);
                pose = pose3;
                range = get_range_to_target(pose,current_local_pos);
                if(range < min_range){
                    ROS_INFO("6Change setpoint");
                    trans = Transitions::ld;
                }
                last_request = ros::Time::now();
            } 
        break;
        case Transitions::ld:
            if((ros::Time::now() - last_request > ros::Duration(5.0))){
                if( srv_land.call(msg_land) &&
                    msg_land.response.success){
                    ROS_INFO("7Vehicle land");
                    trans = Transitions::disarm;
                }
                last_request = ros::Time::now();
            }
        break;
        case disarm:
            msg_arm.request.value = false;
            if(current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( srv_arming.call(msg_arm) &&
                    msg_arm.response.success){
                    ROS_INFO("8Vehicle disarmed");
                    trans = Transitions::disarm;
                }
                last_request = ros::Time::now();
            }
        break;
        default:
            break;
        }
        pub_local_pos.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}