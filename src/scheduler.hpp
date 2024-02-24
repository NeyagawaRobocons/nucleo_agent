#include "rclcpp/rclcpp.hpp"

class Scheduler
{
private:
    rclcpp::Clock clock_;
    rclcpp::Time last_time_;
    rclcpp::Duration wait_time_;
public:
    Scheduler(rclcpp::Duration wait_time): clock_(), last_time_(this->clock_.now()), wait_time_(wait_time){
    }
    bool process(){
        rclcpp::Time now = this->clock_.now();
        if(now - this->last_time_ > this->wait_time_){
            this->last_time_ = now;
            return true;
        }else{
            return false;
        }
    }
    void change_wait_time(rclcpp::Duration wait_time){
        this->wait_time_ = wait_time;
    }
};
