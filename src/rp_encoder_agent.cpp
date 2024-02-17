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
#include <math.h>

#include "bfcobs.hpp"

class SerialPublisherNode : public rclcpp::Node {
public:
  SerialPublisherNode() : Node("rp_encoder_agent") {
    // トピックの初期化
    publisher_ = create_publisher<nucleo_agent::msg::OdometerData>("odometer_3wheel", 10);
    // パラメータの初期化
    this->declare_parameter("gain_motor_3omni", [this]() {
      std::vector<double> gain;
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      gain.push_back(160.15962547712672);
      return gain;
    }()
    );
    RCLCPP_INFO(this->get_logger(), "rp encoder agent node started");

    std::vector <std::string> devices;
    int num_devices = 0;
    for(const std::filesystem::directory_entry &i : std::filesystem::recursive_directory_iterator("/dev")){
        if(i.path().filename().string().find("ttyrppico") != std::string::npos){
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
                RCLCPP_INFO(this->get_logger(), "OdometerData received");
                auto message = nucleo_agent::msg::OdometerData();
                for (size_t i = 0; i < 3; i++)
                {
                  int32_t rotation;
                  memcpy(&rotation, &data[1 + i * 4], 4);
                  message.rotation[i] = (double)rotation / 400 * M_PI*2;
                }
                for (size_t i = 0; i < 3; i++)
                {
                  int32_t angular_vel;
                  memcpy(&angular_vel, &data[13 + i * 4], 4);
                  message.angular_vel[i] = (double)angular_vel / 0x100 / 400 * M_PI*2;
                }
                message.header.stamp = this->now();
                message.header.frame_id = "odom_omni_3wheel";
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "OdometerData rotation : %f, %f, %f", message.rotation[0], message.rotation[1], message.rotation[2]);
                RCLCPP_INFO(this->get_logger(), "OdometerData angular_vel : %f, %f, %f", message.angular_vel[0], message.angular_vel[1], message.angular_vel[2]);
              }
            }
          }
        } else if (len < 0) {
            // RCLCPP_ERROR(this->get_logger(), "Error reading from serial port : error = %d", len);
        }
      }
    });
  }

  rclcpp::Publisher<nucleo_agent::msg::OdometerData>::SharedPtr publisher_;
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
