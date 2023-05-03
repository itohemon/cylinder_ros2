#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "cylinder_ros2/cylinder_eyes.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CylinderEyes::CylinderEyes() : Node("cylinder_eyes")
{
  pi_ = pigpio_setup();
  if (pi_ < 0) {
    std::cout << "Failed to connect to Pigpio Daemon. Is it running?" << std::endl;
  }

  spi_ = eyes_setup();
  if (spi_ != 0) {
    std::cout << "Failed to set up SPI." << std::endl;
  }

  wheelstate_sub_ 
    = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      std::bind(&CylinderEyes::wheelStateCb, this, _1));

  timer_ = this->create_wall_timer(
    500ms, std::bind(&CylinderEyes::timerCb, this));
}

int CylinderEyes::pigpio_setup()
{
  char *addrStr = NULL;
  char *portStr = NULL;

  const int pi = pigpio_start(addrStr, portStr);

  return pi;
}

int CylinderEyes::eyes_setup()
{
  int rcode = bb_spi_open(pi_, SPI_CE, SPI_MISO, SPI_MOSI, SPI_SCLK, 10 * 1000, 0);
  if (rcode != 0) {
    return rcode;
  }

  char buf[4];
  char inBuf[4];
  unsigned int size = 4;

  buf[1] = 0x00;
  buf[3] = 0x00;

  buf[0] = REG_DIGIT0;
  buf[2] = REG_DIGIT0;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT1;
  buf[2] = REG_DIGIT1;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT2;
  buf[2] = REG_DIGIT2;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT3;
  buf[2] = REG_DIGIT3;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT4;
  buf[2] = REG_DIGIT4;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT5;
  buf[2] = REG_DIGIT5;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT6;
  buf[2] = REG_DIGIT6;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DIGIT7;
  buf[2] = REG_DIGIT7;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DECODE_MODE;
  buf[2] = REG_DECODE_MODE;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_INTENSITY;
  buf[2] = REG_INTENSITY;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_SCAN_LIMIT;
  buf[1] = 0x07;
  buf[2] = REG_SCAN_LIMIT;
  buf[3] = 0x07;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_SHUTDOWN;
  buf[1] = 0x01;
  buf[2] = REG_SHUTDOWN;
  buf[3] = 0x01;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  buf[0] = REG_DISPLAY_TEST;
  buf[1] = 0x00;
  buf[2] = REG_DISPLAY_TEST;
  buf[3] = 0x00;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  return 0;
}

void CylinderEyes::eyes_shutdown()
{
  char buf[4];
  char inBuf[4];
  int size = 4;
  
  buf[0] = REG_SHUTDOWN;
  buf[1] = 0x00;
  buf[2] = REG_SHUTDOWN;
  buf[3] = 0x00;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, size);

  bb_spi_close(pi_, SPI_CE);
}

void CylinderEyes::drawEyes(const uint8_t* pattern)
{
  char buf[6];
  char inBuf[4];

  for (int i = 0; i < 8; i++) {
    buf[0] = i + 1;
    buf[1] = pattern[i];
    buf[2] = i + 1;
    buf[3] = pattern[i];
    bb_spi_xfer(pi_, SPI_CE, buf, inBuf, 4);
  }

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  bb_spi_xfer(pi_, SPI_CE, buf, inBuf, 6);
}

void CylinderEyes::wheelStateCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vth_ = msg->angular.z; // Yaw角速度[rad/s]
}

void CylinderEyes::timerCb(void)
{
  double vth = vth_;
  double rate = vth / MAX_VTH * 100.0;
  if (rate < -70.0) {
    drawEyes(eyes_LL);
  } else if ((rate >= -70.0) && (rate < -30.0)) {
    drawEyes(eyes_L);
  } else if ((rate >= -30.0) && (rate < -10.0)) {
    drawEyes(eyes_CL);
  } else if ((rate >= -10.0) && (rate < 10.0)) {
    drawEyes(eyes_C);
  } else if ((rate >=  10.0) && (rate < 30.0)) {
    drawEyes(eyes_CR);
  } else if ((rate >=  30.0) && (rate < 70.0)) {
    drawEyes(eyes_R);
  } else {
    drawEyes(eyes_RR);
  }    
}

CylinderEyes::~CylinderEyes()
{
  eyes_shutdown();
  pigpio_stop(pi_);
}

int main(int argc,  char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderEyes>());
  rclcpp::shutdown();

  return 0;
}
