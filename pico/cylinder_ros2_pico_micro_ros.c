#include <stdio.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico_uart_transports.h"
#include "pico/multicore.h"

#include "hardware/pwm.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include <pico7219/pico7219.h> // The library's header

const uint PWMA  = 0;           /* L側車輪 */
const uint AI2   = 1;
const uint AI1   = 2;
const uint STBYn = 3;           /* モータドライバへのSTANDBY信号 */
const uint BI1   = 4;           /* R側車輪 */
const uint BI2   = 5;
const uint PWMB  = 6;

const uint ACHA = 8;            /* L側エンコーダ割込みピン */
const uint ACHB = 9;
const uint BCHA = 10;           /* R側エンコーダ割込みピン */
const uint BCHB = 11;

const uint MOSI = 19;           /* Eye LED SPI */
const uint SCK  = 18;
const uint CS   = 17;

const uint LED_PIN = 25;

const double WHEEL_RAD = 0.035;   /* ホイール半径[m] */
const double WHEEL_SEP = 0.186;   /* トレッド[m] */

const int INTR_HZ = 10;         /* タイマー割り込み周期 */
const int PPR = 840; /* 7[パルス/回転] * 2[Up/Down] * 60[ギア比] = 840 */

const int DUTY_CLK = 2500;      /* PWM周波数 */

#define PI 3.14159265358979323846

/* PID制御のパラメータ値 */
#define KP  -0.80
#define KI  -5.33
#define KD   0.08

/* EyePattern */
#define CHAIN_LEN 2             /* 8x8 LED個数 */
#define SPI_CHAN  0             /* PicoのSPIのチャネル番号 */

extern uint8_t eyes_sleep[];
extern uint8_t eyes_open[];
extern uint8_t eyes_close[];
extern uint8_t eyes_RR[];
extern uint8_t eyes_R[];
extern uint8_t eyes_CR[];
extern uint8_t eyes_C[];
extern uint8_t eyes_CL[];
extern uint8_t eyes_L[];
extern uint8_t eyes_LL[];

typedef struct Pico7219 Pico7219;
Pico7219 *pico7219;

/*
 *  A：左車輪
 */
int target_valA;            /* 目標の１秒あたりのパルス数 */
int last_countA;            /* 一回前の割込みまでの累積パルス数*/
int total_countA;           /* 累積パルス数 */
float errorA[2];            /* 0:現在のエラー値、1:一つ前のエラー値 */
float integralA;
uint sliceNumA;
uint chanA;
float omegaA;

/*
 * B：右車輪
 */
int target_valB;            /* 目標の１秒あたりのパルス数 */
int last_countB;            /* 一回前の割込みまでの累積パルス数 */
int total_countB;           /* 累積パルス数 */
float errorB[2];            /* 0:現在のエラー値、1:一つ前のエラー値 */
float integralB;
uint sliceNumB;
uint chanB;
float omegaB;

/*
 * 車輪の状態をpublishするための変数
 * 0:左角速度[rad/s]、 1:左角度[rad]、 2:右角速度[rad/s]、 3:右角度[rad]
 */
static float_t wheelState[4];
std_msgs__msg__Float32MultiArray present_wheelState;

/* Publisher object */
rcl_publisher_t publisher;

/*
 * 走行距離をpublishするための変数
 */
static float odometer;
std_msgs__msg__Float64 presentOdometer;
rcl_publisher_t pub_odometer;

geometry_msgs__msg__Twist cmd_vel;

bool led;

int state;                      /* 内部状態 */

void setMotorA(float);
void setMotorB(float);

void timer1000_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  led = !led;
  gpio_put(LED_PIN, led);
}

void timer100_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  float valueA;
  float valueB;
  int tcountA = total_countA;
  int tcountB = total_countB;
  int feedback_valA = tcountA - last_countA;
  int feedback_valB = tcountB - last_countB;
  int targetA = target_valA;
  int targetB = target_valB;
  float vel;

  errorA[0] = errorA[1];
  errorB[0] = errorB[1];

  errorA[1] = feedback_valA * INTR_HZ - targetA;
  errorB[1] = feedback_valB * INTR_HZ - targetB;

  integralA += (errorA[1] + errorA[0]) / 2.0 / INTR_HZ;
  integralB += (errorB[1] + errorB[0]) / 2.0 / INTR_HZ;

  float pA = KP * errorA[1];
  float pB = KP * errorB[1];
  float iA = KI * integralA;
  float iB = KI * integralB;
  float dA = KD * (errorA[1] - errorA[0]) * INTR_HZ;
  float dB = KD * (errorB[1] - errorB[0]) * INTR_HZ;

  valueA = pA + iA + dA;
  valueB = pB + iB + dB;
  //valueA = targetA * DUTY_CLK / PPR;
  //valueB = targetB * DUTY_CLK / PPR;

  /* publish message */
  omegaA = feedback_valA * 2.0 * PI * INTR_HZ / PPR;
  omegaB = feedback_valB * 2.0 * PI * INTR_HZ / PPR;
  present_wheelState.data.data[0] = omegaA;
  present_wheelState.data.data[1] = tcountA * 2.0 * PI / PPR;
  present_wheelState.data.data[2] = omegaB;
  present_wheelState.data.data[3] = tcountB * 2.0 * PI / PPR;
  present_wheelState.data.size = 4;

  vel = WHEEL_RAD * (omegaA + omegaB) / 2.0;
  float abs_vel = vel;
  if (abs_vel < 0)
    abs_vel *= -1.0;
  odometer += abs_vel / INTR_HZ;
  presentOdometer.data = odometer;

  rcl_ret_t ret;
  ret = rcl_publish(&publisher, &present_wheelState, NULL);
  ret = rcl_publish(&pub_odometer, &presentOdometer, NULL);
  
  setMotorA(valueA);
  setMotorB(valueB);

  last_countA = tcountA;
  last_countB = tcountB;
}

void cmd_vel_Cb(const void * msgin)
{
  const geometry_msgs__msg__Twist * cmdVelMsg 
    = (const geometry_msgs__msg__Twist *) msgin;

  double cmdV = cmdVelMsg->linear.x;  /* 車体の目標速度[m/s] */
  double cmdW = cmdVelMsg->angular.z; /* 車体の目標角速度[rad/s] */
  double target_wR, target_wL;  /* モータの目標回転角速度[rad/s] */

  target_wR = cmdV / WHEEL_RAD + WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;
  target_wL = cmdV / WHEEL_RAD - WHEEL_SEP * cmdW / 2.0 / WHEEL_RAD;

  target_valA = target_wL * PPR / PI / 2.0; /* Left */
  target_valB = target_wR * PPR / PI / 2.0; /* Right */
}

void setMotorA(float value)
{
  if (value > 0)
  {
    gpio_put(AI1, 1);
    gpio_put(AI2, 0);
    pwm_set_chan_level(sliceNumA, chanA, value);
  }
  else if (value < 0)
  {
    gpio_put(AI1, 0);
    gpio_put(AI2, 1);
    pwm_set_chan_level(sliceNumA, chanA, value * (-1.0));
  }
  else
  {
    gpio_put(AI1, 0);
    gpio_put(AI2, 0);
    pwm_set_chan_level(sliceNumA, chanA, 0);
  }
}

void setMotorB(float value)
{
  if (value > 0)
  {
    gpio_put(BI1, 0);
    gpio_put(BI2, 1);
    pwm_set_chan_level(sliceNumB, chanB, value);
  }
  else if (value < 0)
  {
    gpio_put(BI1, 1);
    gpio_put(BI2, 0);
    pwm_set_chan_level(sliceNumB, chanB, value * (-1.0));
  }
  else
  {
    gpio_put(BI1, 0);
    gpio_put(BI2, 0);
    pwm_set_chan_level(sliceNumB, chanB, 0);
  }
}

void readEncoder(uint gpio, uint32_t events)
{
  if (gpio == ACHA) {
    bool aa = gpio_get(ACHA);
    bool ab = gpio_get(ACHB);
    if (aa == ab)
    {
      total_countA++;
    }
    else
    {
      total_countA--;
    }
    return;
  } 

  if (gpio == BCHA) {
    bool ba = gpio_get(BCHA);
    bool bb = gpio_get(BCHB);
    if (ba != bb)
    {
      total_countB++;
    }
    else
    {
      total_countB--;
    }
    return;
  }
}

void eyeOpen()
{
  for (int pattern = 0; pattern < 4; pattern++) {
    for (int i = 0; i < 8; i++) // row
    {
      uint8_t v = eyes_open[i + 8 * pattern];
      for (int j = 0; j < 8; j++) // column
      {
        int sel = 1 << j;
        if (sel & v) {
          pico7219_switch_on (pico7219, i, j, FALSE);
          pico7219_switch_on (pico7219, i, j + 8, FALSE);
        }
      }
    }
    
    pico7219_flush (pico7219);
    sleep_ms(100);
    pico7219_switch_off_all (pico7219, FALSE);
  }
}

void eyeClose()
{
  for (int pattern = 0; pattern < 4; pattern++) {
    for (int i = 0; i < 8; i++) // row
    {
      uint8_t v = eyes_close[i + 8 * pattern];
      for (int j = 0; j < 8; j++) // column
      {
        int sel = 1 << j;
        if (sel & v) {
          pico7219_switch_on (pico7219, i, j, FALSE);
          pico7219_switch_on (pico7219, i, j + 8, FALSE);
        }
      }
    }
    
    pico7219_flush (pico7219);
    sleep_ms(100);
    pico7219_switch_off_all (pico7219, FALSE);
  }
}

void eyeBlink()
{
  eyeOpen();
  eyeClose();
}

void eyeOnePattern(uint8_t *pattern)
{
  for (int i = 0; i < 8; i++) // row
  {
    uint8_t v = pattern[i];
    for (int j = 0; j < 8; j++) // column
    {
      int sel = 1 << j;
      if (sel & v) {
        pico7219_switch_on (pico7219, i, j, FALSE);
        pico7219_switch_on (pico7219, i, j + 8, FALSE);
      }
    }
  }
  pico7219_flush (pico7219);
}

void eyePattern()
{
  float diffOmega;

  while (1) // Forever
  {
    switch (state)
    {
      case 0:
        eyeOnePattern(eyes_sleep);
        break;
      case 1:
        eyeOpen();
        state = 2;
        break;
      case 2:
        diffOmega = omegaA - omegaB;

        if (diffOmega > 4.0) {
          eyeOnePattern(eyes_LL);
        }
        else if (diffOmega > 3.0) {
          eyeOnePattern(eyes_L);
        }
        else if (diffOmega > 1.5) {
          eyeOnePattern(eyes_CL);
        }
        else if (diffOmega > -1.5) {
          eyeOnePattern(eyes_C);
        }
        else if (diffOmega > -3.0) {
          eyeOnePattern(eyes_CR);
        }
        else if (diffOmega > -4.0) {
          eyeOnePattern(eyes_R);
        }
        else {
          eyeOnePattern(eyes_RR);
        }

        if ((rand() % 20000) == 0) {
          eyeBlink();
        }
        break;
      case 3:
        eyeClose();
        break;
    }

    pico7219_switch_off_all (pico7219, FALSE);
  }
  
}

int main()
{
  /*
   * variables initialize
   */
  total_countA = 0;
  total_countB = 0;
  last_countA = 0;
  last_countB = 0;
  omegaA = 0;
  omegaB = 0;
  state = 0;
  odometer = 0.0;

  rmw_uros_set_custom_transport(
    true,
    NULL,
    pico_serial_transport_open,
    pico_serial_transport_close,
    pico_serial_transport_write,
    pico_serial_transport_read
    );

  /*
   * Pico搭載LED
   */
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  led = true;

  /*
   * モーター関連
   */
  gpio_set_dir(ACHA, GPIO_IN);
  gpio_set_dir(ACHB, GPIO_IN);
  gpio_set_dir(BCHA, GPIO_IN);
  gpio_set_dir(BCHB, GPIO_IN);
  gpio_disable_pulls(ACHA);
  gpio_disable_pulls(ACHB);
  gpio_disable_pulls(BCHA);
  gpio_disable_pulls(BCHB);
  gpio_set_irq_enabled_with_callback(ACHA, 0x4u | 0x8u, false, &readEncoder);
  gpio_set_irq_enabled(ACHA, 0x4u | 0x8u, true);
  gpio_set_irq_enabled(BCHA, 0x4u | 0x8u, true);
    
  gpio_init(STBYn);
  gpio_set_dir(STBYn, GPIO_OUT);
  gpio_put(STBYn, 1);

  /* Pico -> モータードライバA(右側） */
  gpio_init(AI1);
  gpio_init(AI2);
  gpio_set_dir(AI1, GPIO_OUT);
  gpio_set_dir(AI2, GPIO_OUT);
  gpio_set_function(PWMA, GPIO_FUNC_PWM);
  
  /* Pico -> モータードライバB(左側） */
  gpio_init(BI1);
  gpio_init(BI2);
  gpio_set_dir(BI1, GPIO_OUT);
  gpio_set_dir(BI2, GPIO_OUT);
  gpio_set_function(PWMB, GPIO_FUNC_PWM);


  /*
   * PWM出力設定
   */
  /* スライス番号取得 */
  sliceNumA = pwm_gpio_to_slice_num(PWMA);
  sliceNumB = pwm_gpio_to_slice_num(PWMB);
  /* ラップアラウンド値設定 */
  /* 0からDUTY_CLK-1までカウントアップし、0に戻る */
  pwm_set_wrap(sliceNumA, DUTY_CLK - 1);
  pwm_set_wrap(sliceNumB, DUTY_CLK - 1);
  /* チャネル番号取得 */
  chanA = pwm_gpio_to_channel(PWMA);
  chanB = pwm_gpio_to_channel(PWMB);
  /* PWMデューティを設定。起動時はデューティ0% にする */
  pwm_set_chan_level(sliceNumA, chanA, 0);
  pwm_set_chan_level(sliceNumB, chanB, 0);
  /* PWM有効化 */
  pwm_set_mask_enabled(0b00001001);

  /*
   * EyePatternライブラリ初期化
   */
  pico7219 = pico7219_create (SPI_CHAN, 1500 * 1000,
    MOSI, SCK, CS, CHAIN_LEN, FALSE);
  pico7219_set_intensity(pico7219, 1);       /* 輝度設定 */
  pico7219_switch_off_all (pico7219, FALSE); /* LED前消灯 */
  multicore_launch_core1(eyePattern);        /* EyePatternはcore1で走らせる */

  rcl_timer_t timer1000;        /* Create timer object */
  rcl_timer_t timer100;         /* Create timer object */
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  // Initialize micro-ROS allocator
  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 255 seconds.
  const int timeout_ms = 1000; 
  const uint8_t attempts = 255;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK)
  {
    // Unreachable agent, exiting program.
    return ret;
  }

  state = 1;

  // Initialize support object
  rclc_support_init(&support, 0, NULL, &allocator);
  // Create node object
  rclc_node_init_default(&node, "pico", "", &support);

  // Subscription object
  rcl_subscription_t subscriber;
  
  // Initialize a reliable subscriber
  rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
    );

  present_wheelState.data.capacity = 4;
  present_wheelState.data.data = wheelState;
  present_wheelState.data.size = 0;

  // Create a reliable rcl publisher
  rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_state");

  // Create a reliable rcl publisher
  rclc_publisher_init_default(&pub_odometer, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "odometer");

  // Initialize timer object
  rclc_timer_init_default(&timer1000, &support,
    RCL_MS_TO_NS(1000),         /* Timer period on nanoseconds */
    timer1000_callback);

  // Initialize timer object
  rclc_timer_init_default(&timer100, &support,
    RCL_MS_TO_NS(1000 / INTR_HZ), /* Timer period on nanoseconds */
    timer100_callback);

  // 3つめの引数は通信オブジェクトの数(Timerとsubscriptionの総数)
  // このプログラムはTimer2つsubscriber1つで通信オブジェクトは3
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  // Add to the executor
  rclc_executor_add_timer(&executor, &timer1000);
  rclc_executor_add_timer(&executor, &timer100);
  rclc_executor_add_subscription(&executor, &subscriber,
    &cmd_vel, &cmd_vel_Cb, ON_NEW_DATA);
  
  gpio_put(LED_PIN, led);

  while (true)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
  state = 4;

  led = false;
  gpio_put(LED_PIN, led);

  rclc_executor_fini(&executor);
  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&pub_odometer, &node);
  rcl_timer_fini(&timer1000);
  rcl_timer_fini(&timer100);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  std_msgs__msg__Float32MultiArray__fini(&present_wheelState);
  std_msgs__msg__Float64__fini(&presentOdometer);
  geometry_msgs__msg__Twist__fini(&cmd_vel);
  
  state = 0;
  gpio_put(STBYn, 0);

  return 0;
}
