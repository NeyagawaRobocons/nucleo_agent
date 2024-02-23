#include "rclcpp/rclcpp.hpp"

class Scheduler
{
private:
    rclcpp::Clock clock_;
    rclcpp::Time last_time_;
    rclcpp::Time wait_time_;
public:
    Scheduler(rclcpp::Time wait_time): clock_(){
        this->last_time_ = this->clock_.now();
        this->wait_time_ = wait_time;
    }
    bool process(){
        rclcpp::Time now = this->clock_.now();
        if(now.nanoseconds() - this->last_time_.nanoseconds() > this->wait_time_.nanoseconds()){
            this->last_time_ = now;
            return true;
        }
    }
    void change_wait_time(rclcpp::Time wait_time){
        this->wait_time_ = wait_time;
    }
};
