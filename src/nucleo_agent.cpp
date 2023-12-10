#include "rclcpp/rclcpp.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <filesystem>
#include <thread>
#include <vector>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "bfcobs.hpp"

class SerialPublisherNode : public rclcpp::Node {
public:
  SerialPublisherNode() : Node("serial_publisher_node") {
    std::vector <std::string> devices;
    for(const std::filesystem::directory_entry &i : std::filesystem::recursive_directory_iterator("/dev")){
        if(i.path().filename().string().find("ttyNucleo") != std::string::npos){
            std::cout << "device : "<<i.path().filename().string() << std::endl;
            devices.push_back("/dev/" + i.path().filename().string());
        }
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

    // トピックの初期化
    publisher_ = create_publisher<std_msgs::msg::String>("serial_data", 10);

    // シリアルポートの読み取りを開始
    startReadingSerial();

    RCLCPP_INFO(this->get_logger(), "SerialPublisherNode started");
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
              for (size_t i = 0; i < size; i++)
              {
                std::cout << std::hex << (int)data[i] << " ";
              }
              std::cout << std::endl;
              RCLCPP_INFO(this->get_logger(), "data len : %d", size);
              // 読み取ったデータをトピックにパブリッシュ
              auto message = std_msgs::msg::String();
              message.data = std::to_string(size);
              publisher_->publish(message);
            }
          }
        } else if (len < 0) {
            // RCLCPP_ERROR(this->get_logger(), "Error reading from serial port : error = %d", len);
        }
      }
    });
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::thread serial_thread_;
  int serial_fd;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
