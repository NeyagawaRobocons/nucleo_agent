#include "rclcpp/rclcpp.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
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

#include "bfcobs.hpp"

class SerialPublisherNode : public rclcpp::Node {
public:
  SerialPublisherNode() : Node("nucleo_agent") {
    // トピックの初期化
    publisher_ = create_publisher<nucleo_agent::msg::OdometerData>("odometer_3wheel", 10);
    motor_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>("motor_3omini", 10, std::bind(&SerialPublisherNode::motor_3omni_callback, this, std::placeholders::_1));
    // パラメータの初期化
    this->declare_parameter("gain_motor_3omni", [this]() {
      std::vector<double> gain;
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
              // for (size_t i = 0; i < size; i++)
              // {
              //   std::cout << std::hex << (int)data[i] << " ";
              // }
              // std::cout << std::endl;
              // RCLCPP_INFO(this->get_logger(), "data len : %ld", size);
              // 読み取ったデータをトピックにパブリッシュ
              if(size == 0 ){
                continue;
              }
              if(data[0] == 0x01 && size == 19){
                auto message = nucleo_agent::msg::OdometerData();
                message.set__rotation({
                  (double)((data[1] << 0) | (data[2] << 8) | (data[3] << 16) | (data[4] << 24)) / 400 * 2 * M_PI,
                  (double)((data[5] << 0) | (data[6] << 8) | (data[7] << 16) | (data[8] << 24)) / 400 * 2 * M_PI,
                  (double)((data[9] << 0) | (data[10] << 8) | (data[11] << 16) | (data[12] << 24)) / 400 * 2 * M_PI
                });
                message.set__angular_vel({
                  (double)(int16_t)((data[13] << 0) | (data[14] << 8)) / 400 * 2 * M_PI,
                  (double)(int16_t)((data[15] << 0) | (data[16] << 8)) / 400 * 2 * M_PI,
                  (double)(int16_t)((data[17] << 0) | (data[18] << 8)) / 400 * 2 * M_PI
                });
                message.header.stamp = this->now();
                message.header.frame_id = "odom_omni_3wheel";
                publisher_->publish(message);
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
      std::array<uint8_t, 7> send_data;
      send_data[0] = 0x01;
      for (size_t i = 0; i < 3; i++)
      {
        int16_t pwm = (int16_t)(msg->data[i] * 0.95 * INT16_MAX / this->get_parameter("gain_motor_3omni").as_double_array()[i]); 
        send_data[i*2 +1] = pwm & 0xff;
        send_data[i*2 +2] = pwm & 0xff;
      }
      
      auto encoded_data = cobs_encode(send_data);

      write(this->serial_fd, encoded_data.data(), encoded_data.size());

      RCLCPP_INFO(this->get_logger(), "motor_3omni message received : %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
    }else{
      RCLCPP_INFO(this->get_logger(), "invalid motor_3omni message length (must be 3) : %ld", msg->data.size());
    }
  }

  rclcpp::Publisher<nucleo_agent::msg::OdometerData>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_subscriber_;
  std::thread serial_thread_;
  int serial_fd;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialPublisherNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
