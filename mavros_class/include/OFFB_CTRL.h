#pragma once

#include <array>
#include <random>
#include <ros/ros.h>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

namespace TEST_OFFB{
typedef enum {
    POSITION,
    VELOCITY,
    ACCELERATION
} control_mode;

void wait_and_move(geometry_msgs::PoseStamped target);

class OFFB_CTRL{
public:
    OFFB_CTRL() :
		nh("~"),
        rateHz(10.0),
        // Publisher
		pub_sp_local_pos(nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),
		pub_sp_vel(nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10)),
        // Subscriber
        sub_state(nh.subscribe("/mavros/state", 10, &OFFB_CTRL::callback_state, this)),
		sub_local_pos(nh.subscribe("/mavros/local_position/local", 10, &OFFB_CTRL::callback_local_pos, this)),
        // Service
        
        threshold(threshold_definition())
	{
        srv_set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    };

    void set_offboard();
    void set_loop_rate(double set_rate);
    void circle_path_motion(ros::Rate loop_rate, control_mode mode);
private:
    // Node Handle
	ros::NodeHandle nh;
    // Publisher
	ros::Publisher pub_sp_local_pos;
	ros::Publisher pub_sp_vel;
	// Subscriber
    ros::Subscriber sub_state;
    ros::Subscriber sub_local_pos;
    // Service
    ros::ServiceClient srv_set_mode;

    
    // Messages
    mavros_msgs::State current_state;
	geometry_msgs::PoseStamped current_local_pos, sp_local_pos;
	geometry_msgs::TwistStamped sp_vel;
    
    // Callback for Subscribing
	void callback_local_pos(const geometry_msgs::PoseStampedConstPtr& msg);
    void callback_state(const mavros_msgs::State::ConstPtr& msg);

    // 1st Properties
    double rateHz;
	control_mode mode;

	Eigen::Vector3d current;
	std::array<double, 100> threshold;

    void wait_and_move(geometry_msgs::PoseStamped target);
    std::array<double, 100> threshold_definition();

};
}