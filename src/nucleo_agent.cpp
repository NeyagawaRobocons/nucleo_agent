#include "rclcpp/rclcpp.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
#include "nucleo_agent/msg/actuator_commands.hpp"
#include "nucleo_agent/msg/sensor_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <filesystem>
#include <thread>
#include <vector>
#include <array>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <mutex>
#include <chrono>

#include "bfcobs2.hpp"
#include "scheduler.hpp"

class SerialPublisherNode : public rclcpp::Node {
public:
  SerialPublisherNode() : Node("nucleo_agent") {
    // init parameter
    this->declare_parameter("num_wheels", 4);
    this->declare_parameter("rate_limit_motor_speed", 0.01);
    this->declare_parameter("rate_limit_daiza_state", 0.03);
    this->declare_parameter("rate_limit_hina_state", 0.03);
    this->declare_parameter("gain_motor_4omni", [this]() {
      std::vector<double> gain;
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      return gain;
    }()
    );
    // トピックの初期化
    publisher_ = create_publisher<nucleo_agent::msg::OdometerData>("motor_speed", 10);
    if (this->get_parameter("num_wheels").as_int() == 4) {
      motor_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>("input_vel", 10, std::bind(&SerialPublisherNode::motor_4omni_callback, this, std::placeholders::_1));
    }else if (this->get_parameter("num_wheels").as_int() == 3) {
      motor_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>("input_vel", 10, std::bind(&SerialPublisherNode::motor_3omni_callback, this, std::placeholders::_1));
    }else{
      RCLCPP_ERROR(this->get_logger(), "invalid num_wheels : %d : num_wheel must be 4 or 3", this->get_parameter("num_wheels").as_int());
      rclcpp::shutdown();
    }
    daiza_cmd_sub_ = create_subscription<nucleo_agent::msg::ActuatorCommands>("daiza_clamp", 10, std::bind(&SerialPublisherNode::daiza_cmd_callback, this, std::placeholders::_1));
    daiza_sennsor_pub_ = create_publisher<nucleo_agent::msg::SensorStates>("daiza_state", 10);
    hina_cmd_sub_ = create_subscription<nucleo_agent::msg::ActuatorCommands>("hina_dastpan", 10, std::bind(&SerialPublisherNode::hina_cmd_callback, this, std::placeholders::_1));
    hina_sennsor_pub_ = create_publisher<nucleo_agent::msg::SensorStates>("hina_state", 10);
    bonbori_cmd_sub_ = create_subscription<std_msgs::msg::Bool>("bonbori_msg", 10, std::bind(&SerialPublisherNode::bonbori_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "nucleo_agent Node started");

    while(!open_serial_port("ttyNucleo") && rclcpp::ok()){
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // シリアルポートの読み取りを開始
    startReadingSerial();
    startReconnectionThread();
    RCLCPP_INFO(this->get_logger(), "Serial port started");
  }
  private:

  // if success return true
  bool open_serial_port(const std::string &port_search_key) {
    std::vector <std::string> devices;
    int num_devices = 0;
    for(const std::filesystem::directory_entry &i : std::filesystem::recursive_directory_iterator("/dev")){
        if(i.path().filename().string().find(port_search_key.c_str()) != std::string::npos){
            RCLCPP_INFO(this->get_logger(), "device found : /dev/%s", i.path().filename().string().c_str());
            devices.push_back("/dev/" + i.path().filename().string());
            num_devices++;
        }
    }
    if(num_devices == 0){
        RCLCPP_ERROR(this->get_logger(), "no device found");
        return false;
    }
    if(num_devices > 1){
        RCLCPP_ERROR(this->get_logger(), "multiple devices found");
        return false;
    }
    serial_fd = open(devices[0].c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "open error");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "serial port open success");

    struct termios tio;
    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None
    cfsetispeed( &tio, B115200 );
    cfsetospeed( &tio, B115200 );
    cfmakeraw(&tio);                    // RAWモード
    tcsetattr( serial_fd, TCSANOW, &tio );     // デバイスに設定を行う
    ioctl(serial_fd, TCSETS, &tio);            // ポートの設定を有効にする

    // clear read buffer
    tcflush(serial_fd, TCIFLUSH);
    return true;
  }

  void startReconnectionThread(){
    reconection_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        if(reconnection_flag_.load()){
          RCLCPP_INFO(this->get_logger(), "reconnection start");
          while(!open_serial_port("ttyNucleo") && rclcpp::ok()){
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          }
          reconnection_flag_.store(false);
          RCLCPP_INFO(this->get_logger(), "reconnection success");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));
      }
    });
  }

  void startReadingSerial() {
    // 非同期にシリアルポートの読み取りを行う
    serial_thread_ = std::thread([this]() {
      Scheduler motor_pub_scheduler(rclcpp::Duration::from_seconds(this->get_parameter("rate_limit_motor_speed").as_double()));
      Scheduler daiza_pub_scheduler(rclcpp::Duration::from_seconds(this->get_parameter("rate_limit_daiza_state").as_double()));
      Scheduler hina_pub_scheduler(rclcpp::Duration::from_seconds(this->get_parameter("rate_limit_hina_state").as_double()));
      int count = 0;
      bfcobs<256> cobs;
      // clear read buffer
      tcflush(serial_fd, TCIFLUSH);
      while (rclcpp::ok()) {
        char buffer[256];
        ssize_t len = read(this->serial_fd, buffer, sizeof(buffer));
        // if(len < 0) reconnection_flag_.store(true);
        if (len > 0) {
          // データを処理する部分
          for (size_t i = 0; i < len; i++)
          {
            int len = cobs.push(buffer[i]);
            if(len > 0){
              uint8_t data[256];
              size_t size;
              cobs.read(data, &size);
              if(size == 0 ){
                continue;
              }
              if(size == 25)if(data[0] == 0x01)if(motor_pub_scheduler.process()){
                // RCLCPP_INFO(this->get_logger(), "OdometerData received");
                auto message = nucleo_agent::msg::OdometerData();
                for (size_t i = 0; i < 3; i++)
                {
                  float rotation;
                  memcpy(&rotation, &data[1 + i * 4], 4);
                  message.rotation[i] = rotation;
                }
                for (size_t i = 0; i < 3; i++)
                {
                  float angular_vel;
                  memcpy(&angular_vel, &data[13 + i * 4], 4);
                  message.angular_vel[i] = angular_vel;
                }
                message.header.stamp = this->now();
                message.header.frame_id = "omni_4wheel";
                publisher_->publish(message);
                // RCLCPP_INFO(this->get_logger(), "OdometerData rotation : %f, %f, %f", message.rotation[0], message.rotation[1], message.rotation[2]);
                // RCLCPP_INFO(this->get_logger(), "OdometerData angular_vel : %f, %f, %f", message.angular_vel[0], message.angular_vel[1], message.angular_vel[2]);
              }
              if(size == 3)if(data[0] == 0x02)if(daiza_pub_scheduler.process()){
                // RCLCPP_INFO(this->get_logger(), "daiza_state received");
                auto message = nucleo_agent::msg::SensorStates();
                message.limit_switch_states.resize(1, false);
                message.cylinder_states.resize(4, false);
                for (size_t i = 0; i < 1; i++)
                {
                  message.limit_switch_states[i] = (data[1] >> i) & 0x01;
                }
                for (size_t i = 0; i < 4; i++)
                {
                  message.cylinder_states[i] = (data[2] >> i) & 0x01;
                }
                // message.header.stamp = this->now();
                // message.header.frame_id = "daiza_state";
                daiza_sennsor_pub_->publish(message);
              }
              if(size == 7)if(data[0] == 0x03)if(hina_pub_scheduler.process()){
                // RCLCPP_INFO(this->get_logger(), "hina_state received");
                auto message = nucleo_agent::msg::SensorStates();
                message.limit_switch_states.resize(5, false);
                message.cylinder_states.resize(2, false);
                message.potentiometer_angles.resize(1, 0.0);
                for (size_t i = 0; i < 5; i++)
                {
                  message.limit_switch_states[i] = (data[1] >> i) & 0x01;
                }
                for (size_t i = 0; i < 2; i++)
                {
                  message.cylinder_states[i] = (data[2] >> i) & 0x01;
                }
                for (size_t i = 0; i < 1; i++)
                {
                  float potentiometer_angles;
                  memcpy(&potentiometer_angles, &data[3 + i * 4], 4);
                  message.potentiometer_angles[i] = potentiometer_angles;
                }
                // message.header.stamp = this->now();
                // message.header.frame_id = "hina_state";
                hina_sennsor_pub_->publish(message);
              }
              if(size == 5) if(data[0] == 0xff){
                int16_t motor_outputs[2];
                memcpy(&motor_outputs[0], &data[1], 2);
                memcpy(&motor_outputs[1], &data[3], 2);
                // RCLCPP_INFO(this->get_logger(), "motors: %d, %d", motor_outputs[0], motor_outputs[1]);
              }
            }
          }
        } else if (len < 0) {
            // RCLCPP_ERROR(this->get_logger(), "Error reading from serial port : error = %d", len);
        }
      }
    });
  }
  void motor_3omni_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
    if(msg->data.size() == 3){
      std::array<uint8_t, 13> send_data;
      send_data[0] = 0x01;
      for (size_t i = 0; i < 3; i++)
      {
        // pwm[i] = (int16_t)(msg->data[i] * 0.95 * INT16_MAX / this->get_parameter("gain_motor_3omni").as_double_array()[i]); 
        float motor_target = msg->data[i];
        memcpy(&send_data[1 + i * 4], &motor_target, 4);
      }
      
      auto encoded_data = cobs_encode(send_data);

      auto write_return = write(this->serial_fd, encoded_data.data(), encoded_data.size());
      if(write_return < 0) reconnection_flag_.store(true);
      // RCLCPP_INFO(this->get_logger(), "motor write return : %ld", write_return);

      // RCLCPP_INFO(this->get_logger(), "motor_3omni message received : %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid motor_3omni message length (must be 3) : %ld", msg->data.size());
    }
  }
  void motor_4omni_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
    if(msg->data.size() == 4){
      std::array<uint8_t, 17> send_data;
      send_data[0] = 0x01;
      for (size_t i = 0; i < 4; i++)
      {
        // pwm[i] = (int16_t)(msg->data[i] * 0.95 * INT16_MAX / this->get_parameter("gain_motor_3omni").as_double_array()[i]); 
        float motor_target = msg->data[i];
        memcpy(&send_data[1 + i * 4], &motor_target, 4);
      }
      
      auto encoded_data = cobs_encode(send_data);

      auto write_return = write(this->serial_fd, encoded_data.data(), encoded_data.size());
      if(write_return < 0) reconnection_flag_.store(true);
      // RCLCPP_INFO(this->get_logger(), "motor write return : %ld", write_return);

      // RCLCPP_INFO(this->get_logger(), "motor_3omni message received : %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid motor_3omni message length (must be 4) : %ld", msg->data.size());
    }
  }
  void daiza_cmd_callback(const nucleo_agent::msg::ActuatorCommands::SharedPtr msg) const {
    if(msg->cylinder_states.size()==4 && msg->motor_expand.size()==0 && msg->motor_positions.size()==0){
      // RCLCPP_INFO(this->get_logger(), "daiza_clamp message received");
      std::array<uint8_t, 2> send_data;
      send_data[0] = 0x02;
      send_data[1] = 0x00;
      for (size_t i = 0; i < 4; i++)
      {
        send_data[1] |= (uint8_t)(msg->cylinder_states[i]) << i;
      }

      bool is_data_change = false;
      for (size_t i = 0; i < send_data.size(); i++){
        if(send_data[i] != daiza_last_send_data[i]){
          daiza_last_send_data[i] = send_data[i];
          // std::cout << "data change at" << i << std::endl;
          is_data_change = true;
        }
      }
      if(is_data_change == false) { return;}

      // std::cout << "send_data : ";
      // for (size_t i = 0; i < send_data.size(); i++)
      // {
      //   std::cout << std::hex << (int)send_data[i] << ", ";
      // }

      // for (size_t i = 0; i < 4; i++)
      // {
      //   std::cout << std::hex << (int)((send_data[1] >> i) & 0x01) << ", ";
      // }
      // std::cout << std::endl;
      
      auto encoded_data = cobs_encode(send_data);

      write(this->serial_fd, encoded_data.data(), encoded_data.size());
      auto write_return = write(this->serial_fd, encoded_data.data(), encoded_data.size());
      if(write_return < 0) reconnection_flag_.store(true);
      RCLCPP_INFO(this->get_logger(), "daiza write return : %ld", write_return);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid daiza_clamp message length (must be 4, 0, 0) : %ld, %ld, %ld", msg->cylinder_states.size(), msg->motor_expand.size(), msg->motor_positions.size());
    }
  }
  void hina_cmd_callback(const nucleo_agent::msg::ActuatorCommands::SharedPtr msg) const {
    if(msg->cylinder_states.size()==2 && msg->motor_expand.size()==1 && msg->motor_positions.size()==3){
      std::array<uint8_t, 15> send_data;
      send_data[0] = 0x03;
      send_data[1] = 0x00;
      for (size_t i = 0; i < 1; i++)
      {
        send_data[1] |= ((uint8_t)(msg->motor_expand[i]) << i);
      }
      send_data[2] = 0x00;
      for (size_t i = 0; i < 2; i++)
      {
        send_data[2] |= ((uint8_t)(msg->cylinder_states[i]) << i);
      }
      for (size_t i = 0; i < 3; i++)
      {
        float motor_positions = msg->motor_positions[i];
        memcpy(&send_data[3 + i * 4], &motor_positions, 4);
      }
      bool is_data_change = false;
      for (size_t i = 0; i < send_data.size(); i++){
        if(send_data[i] != hina_last_send_data[i]){
          hina_last_send_data[i] = send_data[i];
          is_data_change = true;
        }
      }
      // RCLCPP_INFO(this->get_logger(), "hina : %d, %d, %d, %d    changed : %d", send_data[0], send_data[1], send_data[2], send_data[3], is_data_change);
      if(is_data_change == false) return;
      
      auto encoded_data = cobs_encode(send_data);

      write(this->serial_fd, encoded_data.data(), encoded_data.size());
      auto write_return = write(this->serial_fd, encoded_data.data(), encoded_data.size());
      if(write_return < 0) reconnection_flag_.store(true);
      RCLCPP_INFO(this->get_logger(), "hina write return : %ld", write_return);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid daiza_clamp message length (must be 2, 1, 3) : %ld, %ld, %ld", msg->cylinder_states.size(), msg->motor_expand.size(), msg->motor_positions.size());
    }
  }

  void bonbori_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
    std::array<uint8_t, 2> send_data;
    send_data[0] = 0x04;
    send_data[1] = msg->data;
    auto encoded_data = cobs_encode(send_data);
    write(this->serial_fd, encoded_data.data(), encoded_data.size());
    auto write_return = write(this->serial_fd, encoded_data.data(), encoded_data.size());
    if(write_return < 0) reconnection_flag_.store(true);
    RCLCPP_INFO(this->get_logger(), "bonbori write return : %ld", write_return);
  }

  rclcpp::Publisher<nucleo_agent::msg::OdometerData>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscriber_;
  rclcpp::Subscription<nucleo_agent::msg::ActuatorCommands>::SharedPtr daiza_cmd_sub_;
  rclcpp::Publisher<nucleo_agent::msg::SensorStates>::SharedPtr daiza_sennsor_pub_;
  rclcpp::Subscription<nucleo_agent::msg::ActuatorCommands>::SharedPtr hina_cmd_sub_;
  rclcpp::Publisher<nucleo_agent::msg::SensorStates>::SharedPtr hina_sennsor_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bonbori_cmd_sub_;
  std::thread serial_thread_;
  std::thread reconection_thread_;
  mutable std::atomic<bool> reconnection_flag_ = false;
  int serial_fd;
  mutable std::array<uint8_t, 2> daiza_last_send_data;
  mutable std::array<uint8_t, 6> hina_last_send_data;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialPublisherNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
