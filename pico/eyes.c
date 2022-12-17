#include <hardware/spi.h>
#include "eyes.h"
#include "common.h"

static inline void cs_select() {
  asm volatile("nop \n nop \n nop");
  gpio_put(CS, 0);
  asm volatile("nop \n nop \n nop");
  // Active low
}

static inline void cs_deselect() {
  asm volatile("nop \n nop \n nop");
  gpio_put(CS, 1);
  asm volatile("nop \n nop \n nop");
}

static void max7219_write(uint8_t reg, uint8_t data) {
  int i;
  uint8_t buf[2];

  buf[0] = 0x00 | reg;
  buf[1] = data;

  cs_select();
  for (i = 0; i < CHAIN_LEN; i++) {
    spi_write_blocking(SPI_PORT, buf, 2);
  }
  cs_deselect();
}

/*
 * EyePattern関連初期化
 */
void eyes_init(void)
{
  gpio_init(CS);
  gpio_set_dir(CS, GPIO_OUT);
  gpio_put(CS, 1);

  spi_init(SPI_PORT, 1000 * 1000);

  spi_set_format(SPI_PORT,
    8,
    1,
    1,
    SPI_MSB_FIRST);

  gpio_set_function(SCK, GPIO_FUNC_SPI);
  gpio_set_function(MOSI, GPIO_FUNC_SPI);
  
  sleep_ms(10);

  max7219_write(REG_DIGIT0, 0x00);
  max7219_write(REG_DIGIT1, 0x00);
  max7219_write(REG_DIGIT2, 0x00);
  max7219_write(REG_DIGIT3, 0x00);
  max7219_write(REG_DIGIT4, 0x00);
  max7219_write(REG_DIGIT5, 0x00);
  max7219_write(REG_DIGIT6, 0x00);
  max7219_write(REG_DIGIT7, 0x00);

  max7219_write(REG_DECODE_MODE,  0x00);
  max7219_write(REG_INTENSITY,    0x00);
  max7219_write(REG_SCAN_LIMIT,   0x07);
  max7219_write(REG_SHUTDOWN,     0x01);
  max7219_write(REG_DISPLAY_TEST, 0x00);

  sleep_ms(10);
}

void eyeOnePattern(const uint8_t *pattern, int size)
{
  for (int i = 0; i < size / 8; i++) {
    uint8_t addr = 0x01;
    for (int j = 0; j < 8; j++) {
      max7219_write(addr, pattern[i * 8 + j]);
      addr++;
    }
    sleep_ms(10);
  }
}

void eyePattern()
{
  float diffOmega;

  while (1) // Forever
  {
    switch (state)
    {
      case 0:
        eyeOnePattern(eyes_sleep, sizeof(eyes_sleep));
        break;
      case 1:
        eyeOnePattern(eyes_open, sizeof(eyes_open));
        sleep_ms(100);
        state = 2;
        break;
      case 2:
        diffOmega = omegaA - omegaB;

        if (diffOmega > 4.0) {
          eyeOnePattern(eyes_LL, sizeof(eyes_LL));
        }
        else if (diffOmega > 3.0) {
          eyeOnePattern(eyes_L, sizeof(eyes_L));
        }
        else if (diffOmega > 1.5) {
          eyeOnePattern(eyes_CL, sizeof(eyes_CL));
        }
        else if (diffOmega > -1.5) {
          eyeOnePattern(eyes_C, sizeof(eyes_C));
        }
        else if (diffOmega > -3.0) {
          eyeOnePattern(eyes_CR, sizeof(eyes_CR));
        }
        else if (diffOmega > -4.0) {
          eyeOnePattern(eyes_R, sizeof(eyes_R));
        }
        else {
          eyeOnePattern(eyes_RR, sizeof(eyes_RR));
        }

        if ((rand() % 300) == 0) {
          eyeOnePattern(eyes_close, sizeof(eyes_close));
          sleep_ms(100);
          eyeOnePattern(eyes_sleep, sizeof(eyes_sleep));
          sleep_ms(100);
          eyeOnePattern(eyes_open, sizeof(eyes_open));
          sleep_ms(100);
        }
        break;
      case 3:
        eyeOnePattern(eyes_close, sizeof(eyes_close));
        sleep_ms(100);
        break;
    }
  }
}

