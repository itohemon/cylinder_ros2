#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <pigpiod_if2.h>

class CylinderEyes : public rclcpp::Node
{
public:
  CylinderEyes();
  ~CylinderEyes();
  
  const double MAX_VTH = 2.0;

  const uint8_t REG_DIGIT0       = 0x01;
  const uint8_t REG_DIGIT1       = 0x02;
  const uint8_t REG_DIGIT2       = 0x03;
  const uint8_t REG_DIGIT3       = 0x04;
  const uint8_t REG_DIGIT4       = 0x05;
  const uint8_t REG_DIGIT5       = 0x06;
  const uint8_t REG_DIGIT6       = 0x07;
  const uint8_t REG_DIGIT7       = 0x08;
  const uint8_t REG_DECODE_MODE  = 0x09;
  const uint8_t REG_INTENSITY    = 0x0A;
  const uint8_t REG_SCAN_LIMIT   = 0x0B;
  const uint8_t REG_SHUTDOWN     = 0x0C;
  const uint8_t REG_DISPLAY_TEST = 0x0F;

  const unsigned int SPI_CE = 17;
  const unsigned int SPI_MISO = 19;        // 使わない
  const unsigned int SPI_MOSI = 27;
  const unsigned int SPI_SCLK = 22;
  
  const uint8_t eyes_RR[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b10000011,
    0b10000011,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_R[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b10000111,
    0b10000111,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_CR[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b10001101,
    0b10001101,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_C[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b10011001,
    0b10011001,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_CL[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b10110001,
    0b10110001,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_L[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b11100001,
    0b11100001,
    0b10000001,
    0b01000010,
    0b00111100
  };

  const uint8_t eyes_LL[8] = 
  {
    //01234567
    //--------
    0b00111100,
    0b01000010,
    0b10000001,
    0b11000001,
    0b11000001,
    0b10000001,
    0b01000010,
    0b00111100
  };
  
private:
  void wheelStateCb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCb();

  int pigpio_setup();
  int eyes_setup();
  void eyes_shutdown();
  void drawEyes(const uint8_t* pattern);
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheelstate_sub_;

  double vth_;
  int pi_;
  int spi_;

};
