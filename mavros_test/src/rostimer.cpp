#include "rostimer.h"

rostimer::rostimer(){
    timer_sec = 1.0;
    bget_time = false;
}
// initialize timer
rostimer::rostimer(double sec){
    if(sec < 0.0001){
        ROS_ERROR("WRONG TIME SETTING FOR TIMER");
        timer_sec = 1.0;
    }
    else{
        timer_sec = sec;
        last_request = ros::Time::now();
        bget_time = true;
    }
}
rostimer::~rostimer(){

}
// initialize timer
void rostimer::set_timer(double sec){
    if(!bget_time){
        ROS_ERROR("Timer already set");
        return;
    }
    else if(sec < 0.0001){
        ROS_ERROR("invalid timer setting, it will be set 1.0 sec");
        timer_sec = 1.0;
        return;
    }
    else{
        last_request = ros::Time::now();
        timer_sec = sec;
        bget_time = true;
    }
}
void rostimer::change_timer(double sec){
    if(sec < 0.0001){
        ROS_ERROR("invalid timer setting, it will be set 1.0 sec");
        timer_sec = 1.0;
        return;
    }
    else{
        timer_sec = 1.0;
    }
}

// Set initial time
void rostimer::reset_timer(){
    if(!bget_time){
        ROS_ERROR("Timer already set");
        return;
    }
    else{
        last_request = ros::Time::now();
    }
}

// return true  : time over
// return false : in time
bool rostimer::check_duration(){
    if(!bget_time){
        ROS_ERROR("Timer already set");
        return false;
    }
    else{
        if(ros::Time::now() - last_request > ros::Duration(timer_sec)){
            return true;   // time over
        }
        else{
            return false;    // in time   
        }
    }
}