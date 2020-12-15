#pragma once 
#include <ros/ros.h>

// 1. create variable with time duration over 0.00001 sec
// 2. call start_timer to count time
// 3. 
class rostimer{
public:
    rostimer();
    rostimer(double sec);
    ~rostimer();
    void set_timer(double sec);
    void change_timer(double sec);
    void reset_timer();
    bool check_duration();
    
private:
    double timer_sec;
    bool bget_time;
    ros::Time last_request;

};