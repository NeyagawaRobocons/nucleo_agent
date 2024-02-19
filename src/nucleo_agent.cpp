#include "rclcpp/rclcpp.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
#include "mecha_control/msg/actuator_commands.hpp"
#include "mecha_control/msg/mecha_state.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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

#include "bfcobs.hpp"

class SerialPublisherNode : public rclcpp::Node {
public:
  SerialPublisherNode() : Node("nucleo_agent") {
    // トピックの初期化
    publisher_ = create_publisher<nucleo_agent::msg::OdometerData>("motor_speed", 10);
    motor_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>("input_vel", 10, std::bind(&SerialPublisherNode::motor_4omni_callback, this, std::placeholders::_1));
    daiza_cmd_sub_ = create_subscription<mecha_control::msg::ActuatorCommands>("daiza_clamp", 10, std::bind(&SerialPublisherNode::daiza_cmd_callback, this, std::placeholders::_1));
    daiza_sennsor_pub_ = create_publisher<mecha_control::msg::SensorStates>("daiza_state", 10);
    hina_cmd_sub_ = create_subscription<mecha_control::msg::ActuatorCommands>("hina_dastpan", 10, std::bind(&SerialPublisherNode::hina_cmd_callback, this, std::placeholders::_1));
    hina_sennsor_pub_ = create_publisher<mecha_control::msg::SensorStates>("hina_state", 10);
    // パラメータの初期化
    this->declare_parameter("gain_motor_4omni", [this]() {
      std::vector<double> gain;
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      return gain;
    }()
    );
    RCLCPP_INFO(this->get_logger(), "nucleo_agent Node started");

    std::vector <std::string> devices;
    int num_devices = 0;
    for(const std::filesystem::directory_entry &i : std::filesystem::recursive_directory_iterator("/dev")){
        if(i.path().filename().string().find("ttyNucleo") != std::string::npos){
            std::cout << "device : "<<i.path().filename().string() << std::endl;
            devices.push_back("/dev/" + i.path().filename().string());
            num_devices++;
        }
    }
    if(num_devices == 0){
        std::cout << "no device" << std::endl;
        return ;
    }
    serial_fd = open(devices[0].c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cout << "open error" << std::endl;
        return ;
    }
    std::cout << "open success" << std::endl;

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


    // シリアルポートの読み取りを開始
    startReadingSerial();
    RCLCPP_INFO(this->get_logger(), "Serial port started");
  }
  private:
  void startReadingSerial() {
    // 非同期にシリアルポートの読み取りを行う
    serial_thread_ = std::thread([this]() {
      int count = 0;
      bfcobs<256> cobs;
      // clear read buffer
      tcflush(serial_fd, TCIFLUSH);
      while (rclcpp::ok()) {
        char buffer[256];
        ssize_t len = read(this->serial_fd, buffer, sizeof(buffer));
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
              if(size == 25)if(data[0] == 0x01){
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
                message.header.frame_id = "odom_omni_3wheel";
                publisher_->publish(message);
                // RCLCPP_INFO(this->get_logger(), "OdometerData rotation : %f, %f, %f", message.rotation[0], message.rotation[1], message.rotation[2]);
                // RCLCPP_INFO(this->get_logger(), "OdometerData angular_vel : %f, %f, %f", message.angular_vel[0], message.angular_vel[1], message.angular_vel[2]);
              }
              if(size == 3)if(data[0] == 0x02){
                // RCLCPP_INFO(this->get_logger(), "daiza_state received");
                auto message = mecha_control::msg::SensorStates();
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
              if(size == 6)if(data[0] == 0x03){
                // RCLCPP_INFO(this->get_logger(), "hina_state received");
                auto message = mecha_control::msg::SensorStates();
                message.limit_switch_states.resize(5, false);
                message.potentiometer_angles.resize(1, 0.0);
                for (size_t i = 0; i < 5; i++)
                {
                  message.limit_switch_states[i] = (data[1] >> i) & 0x01;
                }
                for (size_t i = 0; i < 1; i++)
                {
                  float potentiometer_angles;
                  memcpy(&potentiometer_angles, &data[2 + i * 4], 4);
                  message.potentiometer_angles[i] = potentiometer_angles;
                }
                // message.header.stamp = this->now();
                // message.header.frame_id = "hina_state";
                hina_sennsor_pub_->publish(message);
              }
              if(size == 6) if(data[0] == 0xff){
                RCLCPP_INFO(this->get_logger(), "debug cyl states: %d, %d, %d, %d    debug num data: %d", data[1], data[2], data[3], data[4], data[5]);
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

      // RCLCPP_INFO(this->get_logger(), "motor write return : %d", write(this->serial_fd, encoded_data.data(), encoded_data.size()));

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

      // RCLCPP_INFO(this->get_logger(), "motor write return : %d", write(this->serial_fd, encoded_data.data(), encoded_data.size()));

      // RCLCPP_INFO(this->get_logger(), "motor_3omni message received : %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid motor_3omni message length (must be 4) : %ld", msg->data.size());
    }
  }
  void daiza_cmd_callback(const mecha_control::msg::ActuatorCommands::SharedPtr msg) const {
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
          std::cout << "data change at" << i << std::endl;
          is_data_change = true;
        }
      }
      if(is_data_change == false) { return;}

      std::cout << "send_data : ";
      // for (size_t i = 0; i < send_data.size(); i++)
      // {
      //   std::cout << std::hex << (int)send_data[i] << ", ";
      // }

      for (size_t i = 0; i < 4; i++)
      {
        std::cout << std::hex << (int)((send_data[1] >> i) & 0x01) << ", ";
      }
      std::cout << std::endl;
      
      auto encoded_data = cobs_encode(send_data);

      RCLCPP_INFO(this->get_logger(), "daiza write return : %d", write(this->serial_fd, encoded_data.data(), encoded_data.size()));
      RCLCPP_INFO(this->get_logger(), "daiza write return : %d", write(this->serial_fd, encoded_data.data(), encoded_data.size()));
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid daiza_clamp message length (must be 4, 0, 0) : %ld, %ld, %ld", msg->cylinder_states.size(), msg->motor_expand.size(), msg->motor_positions.size());
    }
  }
  void hina_cmd_callback(const mecha_control::msg::ActuatorCommands::SharedPtr msg) const {
    if(msg->cylinder_states.size()==2 && msg->motor_expand.size()==1 && msg->motor_positions.size()==3){
      std::array<uint8_t, 15> send_data;
      send_data[0] = 0x03;
      send_data[1] = 0x00;
      for (size_t i = 0; i < 1; i++)
      {
        send_data[1] = send_data[1] & ((uint8_t)(msg->motor_expand[i]) << i);
      }
      send_data[2] = 0x00;
      for (size_t i = 0; i < 2; i++)
      {
        send_data[2] = send_data[2] & ((uint8_t)(msg->motor_expand[i]) << i);
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
      RCLCPP_INFO(this->get_logger(), "hina : %d, %d, %d, %d    changed : %d", send_data[0], send_data[1], send_data[2], send_data[3], is_data_change);
      if(is_data_change == false) return;
      
      auto encoded_data = cobs_encode(send_data);

      RCLCPP_INFO(this->get_logger(), "hina write return : %d", write(this->serial_fd, encoded_data.data(), encoded_data.size()));
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid daiza_clamp message length (must be 2, 1, 3) : %ld, %ld, %ld", msg->cylinder_states.size(), msg->motor_expand.size(), msg->motor_positions.size());
    }
  }

  rclcpp::Publisher<nucleo_agent::msg::OdometerData>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscriber_;
  rclcpp::Subscription<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_cmd_sub_;
  rclcpp::Publisher<mecha_control::msg::SensorStates>::SharedPtr daiza_sennsor_pub_;
  rclcpp::Subscription<mecha_control::msg::ActuatorCommands>::SharedPtr hina_cmd_sub_;
  rclcpp::Publisher<mecha_control::msg::SensorStates>::SharedPtr hina_sennsor_pub_;
  std::thread serial_thread_;
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
