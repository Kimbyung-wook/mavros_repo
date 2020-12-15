#include "OFFB_CTRL.h"
// #include "math.h"
using namespace TEST_OFFB;

Eigen::Vector3d circle_shape(int angle){
    /** @todo Give possibility to user define amplitude of movement (circle radius)*/
    double r = 5.0f;	// 5 meters radius

    return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
            r * sin(angles::from_degrees(angle)),
            1.0f);
}
/**
 * @brief Gaussian noise generator for accepted position threshold
 */
std::array<double, 100> OFFB_CTRL::threshold_definition(){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::array<double, 100> th_values;

    std::normal_distribution<double> th(0.1f,0.05f);

    for (auto &value : th_values) {
        value = th(gen);
    }
    return th_values;
}
/**
 * @brief Defines the accepted threshold to the destination/target position
 * before moving to the next setpoint.
 */
// Callback for Subscribing
void OFFB_CTRL::callback_local_pos(const geometry_msgs::PoseStampedConstPtr& msg){
    current_local_pos = *msg;
}
void OFFB_CTRL::callback_state(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void OFFB_CTRL::set_offboard(){
    ros::Rate loop_rate(rateHz);
    ros::Time last_request = ros::Time::now();
    ROS_INFO("Enter set_offboard function");
    // 1. Set mode to offboard
    mavros_msgs::SetMode msg_mode;
    msg_mode.request.custom_mode = "OFFBOARD";
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            ROS_INFO("Send to set Offboard");
            if( srv_set_mode.call(msg_mode) &&
                msg_mode.response.mode_sent){
                ROS_INFO("Sent to set Offboard");
            }
            last_request = ros::Time::now();
        }
        if(current_state.mode == "OFFBOARD"){
            ROS_INFO("Engage OFFBOARD mode");
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Escape set_offboard function");
}
void OFFB_CTRL::wait_and_move(geometry_msgs::PoseStamped target){
    ros::Rate loop_rate(rateHz);
    ros::Time last_time = ros::Time::now();
    bool stop = false;

    Eigen::Vector3d dest;

    double distance;
    double err_th = threshold[rand() % threshold.size()];

    ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);

    while (ros::ok() && !stop) {
        tf::pointMsgToEigen(target.pose.position, dest);
        tf::pointMsgToEigen(current_local_pos.pose.position, current);

        distance = sqrt((dest - current).x() * (dest - current).x() +
                (dest - current).y() * (dest - current).y() +
                (dest - current).z() * (dest - current).z());

        if (distance <= err_th)
            stop = true;

        if (mode == POSITION) {
            pub_sp_local_pos.publish(target);
        }
        // else if (mode == VELOCITY) {
        // 	if (use_pid)
        // 		tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.twist.linear);
        // 	else
        // 		tf::vectorEigenToMsg(dest - current, vs.twist.linear);
        // 	vel_sp_pub.publish(vs);
        // }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }
        last_time = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
void OFFB_CTRL::circle_path_motion(ros::Rate loop_rate, control_mode mode=POSITION){
    ROS_INFO("Testing...");
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        tf::pointMsgToEigen(current_local_pos.pose.position, current);

        // starting point
        if (mode == POSITION) {
            tf::pointEigenToMsg(Eigen::Vector3d(5.0f, 0.0f, 1.0f), sp_local_pos.pose.position);
            pub_sp_local_pos.publish(sp_local_pos);
        }
        // else if (mode == VELOCITY) {
        //     if (use_pid)
        //         tf::vectorEigenToMsg(pid.compute_linvel_effort(
        //                     Eigen::Vector3d(5.0f, 0.0f, 1.0f), current, last_time), sp_vel.twist.linear);
        //     else
        //         tf::vectorEigenToMsg(Eigen::Vector3d(5.0f - current.x(), -current.y(), 1.0f - current.z()), sp_vel.twist.linear);
        //     pub_sp_vel.publish(sp_vel);
        // }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }

        wait_and_move(sp_local_pos);

        // motion routine
        for (int theta = 0; theta <= 360; theta++) {
            tf::pointMsgToEigen(current_local_pos.pose.position, current);

            if (mode == POSITION) {
                tf::pointEigenToMsg(circle_shape(theta), current_local_pos.pose.position);
                pub_sp_local_pos.publish(current_local_pos);
            }
            // else if (mode == VELOCITY) {
            //     if (use_pid)
            //         tf::vectorEigenToMsg(pid.compute_linvel_effort(circle_shape(theta), current, last_time), sp_vel.twist.linear);
            //     else
            //         tf::vectorEigenToMsg(circle_shape(theta) - current, sp_vel.twist.linear);
            //     pub_sp_vel.publish(sp_vel);
            // }
            else if (mode == ACCELERATION) {
                // TODO
                return;
            }
            if (theta == 360) {
                ROS_INFO("Test complete!");
                ros::shutdown();
            }
            last_time = ros::Time::now();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}
void OFFB_CTRL::set_loop_rate(double set_rate)
{
    rateHz = std::fmax(2.0,set_rate);
}