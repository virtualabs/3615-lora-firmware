/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2018 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include "lora/subghz.h"

#define LED_RED_PORT GPIOB
#define LED_RED_PIN GPIO11

#define USART_CONSOLE USART1  /* PB6/7 , af7 */

int _write(int file, char *ptr, int len);


static struct state_t state;

struct state_t {
	bool falling;
	int last_hold;
	//int tickcount;
};

/* Setup APB1 frequency to 24MHz */
static void clock_setup_bis(void)
{
  struct rcc_clock_scale pll_config = {
    .pllm = 4,
    .plln = 12,
    .pllp = 0,
    .pllq = 0,
    .pllr = RCC_PLLCFGR_PLLR_DIV2,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSI16,
    //.pll_source = RCC_PLLCFGR_PLLSRC_HSE,
    .hpre = 0,
    .ppre1 = 0,
    .ppre2 = 0,
    .ahb_frequency=24e6,
    .apb1_frequency=24e6,
    .apb2_frequency=24e6
  };
  rcc_clock_setup_pll(&pll_config);

  RCC_CR |= RCC_CR_HSEBYP;
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);

  /* Setup PCLK3 prescaler to 2 (24/2 = 12MHz) */
  //RCC_EXTCFGR = 0x08;
  //RCC_EXTCFGR |= (1<<16);

	/* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART1);
}


static void clock_setup(void)
{
	/* FIXME - this should eventually become a clock struct helper setup */
	rcc_osc_on(RCC_HSE);

	/* 16MHz / 4 = > 4 * 12 = 48MHz VCO => 24MHz main pll  */
	rcc_set_main_pll(
    RCC_PLLCFGR_PLLSRC_HSE,
    4,
    12,
		0,
    0,
    RCC_PLLCFGR_PLLR_DIV2
  );
	
  rcc_osc_on(RCC_PLL);
	/* either rcc_wait_for_osc_ready() or do other things */

	/* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART1);

  /* TODO: figure out what's going on with our PLL ?!$@ */

	rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* careful with the param here! */
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* 48MHz */
	rcc_ahb_frequency = 48e6;
	rcc_apb1_frequency = 48e6;
	rcc_apb2_frequency = 48e6;
}

static void usart_setup(void)
{
	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO6);

  /*
	usart_set_baudrate(USART_CONSOLE, 4800);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_EVEN);
  */
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void print_reg_hex(char *prefix, uint8_t value)
{
  int i;

  char digits[] = "0123456789abcdef";
  char digit[3];
  digit[0] = digits[(value & 0xf0)>>4];
  digit[1] = digits[(value & 0x0f)];
  digit[2] = '\0';
  for (i=0; i<strlen(prefix); i++)
    usart_send_blocking(USART_CONSOLE, prefix[i]);
  usart_send_blocking(USART_CONSOLE, '=');
  usart_send_blocking(USART_CONSOLE, digit[0]);
  usart_send_blocking(USART_CONSOLE, digit[1]);
  usart_send_blocking(USART_CONSOLE, '\r');
  usart_send_blocking(USART_CONSOLE, '\n');
}

void print_reg16_hex(char *prefix, uint16_t value)
{
  int i;

  char digits[] = "0123456789abcdef";
  char digit[4];
  digit[0] = digits[(value & 0xf000) >> 12];
  digit[1] = digits[(value & 0x0f00) >> 8];
  digit[2] = digits[(value & 0x00f0) >> 4];
  digit[3] = digits[(value & 0x000f)];
  for (i=0; i<strlen(prefix); i++)
    usart_send_blocking(USART_CONSOLE, prefix[i]);
  usart_send_blocking(USART_CONSOLE, '=');
  usart_send_blocking(USART_CONSOLE, digit[0]);
  usart_send_blocking(USART_CONSOLE, digit[1]);
  usart_send_blocking(USART_CONSOLE, digit[2]);
  usart_send_blocking(USART_CONSOLE, digit[3]);
  usart_send_blocking(USART_CONSOLE, '\r');
  usart_send_blocking(USART_CONSOLE, '\n');
}

int main(void)
{
  uint32_t count = 0xffffffff;
  uint8_t buf[256];
  uint8_t reg, status;
  uint16_t error = 0;
  uint32_t rx_params = 0;
	int j = 0;
  subghz_result_t res;
	

  /* Setup clock & UART */
  clock_setup_bis();
	usart_setup();

  /* Configure our debug SPI */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO4);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO5);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO6);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO7);

  /* Configure our debug RF_BUSY */
	//gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	//gpio_set_af(GPIOA, GPIO_AF6, GPIO12);

  /* Initialize SUBGHZ. */
  subghz_init();
	subghz_check_device_ready();
  pwr_subghzspi_unselect();

  for (int i = 0; i < 40000; i++) { /* Wait a bit. */
    __asm__("NOP");
  }

  subghz_set_tcxo_mode(SUBGHZ_TCXO_TRIM_1V7, 10 << 6);
  res = subghz_calibrate(SUBGHZ_CALIB_ALL);
  print_reg16_hex("calibrate_res", res);
  if (SUBGHZ_CMD_SUCCESS(res))
  {
    printf("Calibration OK\n");
  }

  subghz_set_standby_mode(SUBGHZ_STDBY_HSE32);

  status = subghz_get_status();
  print_reg16_hex("status mode", SUBGHZ_STATUS_MODE(status));


#if 0
  /* Read a register from SUBGHZ. */
  status = subghz_get_error(&error);
  print_reg16_hex("error", error);
  print_reg_hex("status", status);

  /* Clear error. */
  printf("Clear error\n");
  reg = 0;
  subghz_write_command(0x07, (uint8_t *)&reg, 1);

  /* Read a register from SUBGHZ. */
  status = subghz_get_error(&error);
  print_reg16_hex("error", error);
  print_reg_hex("status", status);

  subghz_write_reg(0x6C7, 0x42);
  status = subghz_read_reg(0x6C7, &reg);
  //status = subghz_get_status();
  print_reg_hex("status", status);
  print_reg_hex("0x6C7", reg);

  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO5);

	/* red led for buttons */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,LED_RED_PIN);

#endif

	/* red led for buttons */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_RED_PIN);
  gpio_set(LED_RED_PORT, LED_RED_PIN);

  test_lora_tx();

	while (1) {
    //gpio_toggle(LED_RED_PORT, LED_RED_PIN);
		for (int i = 0; i < 400000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}

	return 0;
}
