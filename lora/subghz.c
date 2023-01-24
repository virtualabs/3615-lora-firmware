#include <stdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>

#include "subghz.h"

extern void print_reg_hex(char *prefix, uint8_t value);
extern void print_reg16_hex(char *prefix, uint16_t value);

#define SUBGHZ_NSS_LOOP_TIME ((24000000*24U)>>16U)

bool g_deepsleep_enable = true;

int subghz_spi_init(int baudrate_prescaler);
int subghz_wait_on_busy(void);

static  uint16_t rx[4];

int subghz_init(void)
{
  /* Enable SUBGHZSPI clock. */
  rcc_periph_clock_enable(RCC_SUBGHZ);

  /* Configure NVIC to enable RADIO IRQ. */
  nvic_set_priority(NVIC_RADIO_IRQ, 0);
  nvic_enable_irq(NVIC_RADIO_IRQ);

  /* Disable SUBGHZ reset through RCC. */
  rcc_subghz_reset_release();

  /* Wait for SUBGHZ to be reset. */
  while(rcc_subghz_running() != 0);

  /* Select SUBGHZSPI NSS. */
  pwr_subghzspi_select();

  /***
   * TODO:
   * 
   * - EXTI44 management + RF busy trigger => EXTI needs a complete rework on STM32WLE5 ...
   * - SUBGHZSPI dedicated bus (same as SPI1 but at address SUBGHZSPI_BASE), compatible with default SPI driver =)
   **/


  exti_enable_request(EXTI44);
  pwr_enable_rfbusy_wakeup();

  /* Clear pending flag for EXTI45 (RFBUSY) */
  exti_reset_request(EXTI45);

  /* Initialize SUBGHZ SPI bus. */
  return subghz_spi_init(4);
}

/***********************************************
 * SPI core routines
 **********************************************/

/**
 * @brief Initialize SUBGHZ SPI bus.
 * 
 * @return 0 on success, 1 otherwise.
 **/

int subghz_spi_init(int baudrate_prescaler)
{
  /* Disable SUBGHZ peripheral. */
  spi_disable(SUBGHZSPI_BASE);


  /***** Set SUBGHZ CR1 register ******/
  SPI_CR1(SUBGHZSPI_BASE) = (SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM);
  SPI_CR1(SUBGHZSPI_BASE) |= (baudrate_prescaler << 3);

  /***** Set SUBGHZ CR2 register ******/
  SPI_CR2(SUBGHZSPI_BASE) = SPI_CR2_FRXTH | (7 << 8);

  /* Re-enable SUBGHZ peripheral. */
  spi_enable(SUBGHZSPI_BASE);
}

/**
 * @brief Start SPI transaction
 * 
 * This routine ensures the transceiver is ready to accept
 * an SPI transaction and asserts NSS.
*/

void spi_start_transaction(void)
{
  /* Ensure SUBGHZ is ready. */
  subghz_check_device_ready();

  /* Assert NSS. */
  pwr_subghzspi_select();
}


/**
 * @brief End SPI transaction.
 * 
 * This routine deasserts NSS and wait until the transceiver
 * is no more busy.
*/

void  spi_end_transaction(void)
{
  /* Deassert NSS. */
  pwr_subghzspi_unselect();

  /* Wait until SUBGHZ is no more busy. */
  subghz_wait_on_busy();
}


uint8_t spi_transmit(uint8_t data)
{
  spi_send8(SUBGHZSPI_BASE, data);
  return spi_read8(SUBGHZSPI_BASE);
}


uint16_t spi_read_byte(void)
{
  return spi_transmit(0x42);
}

/*******************************************/

/**
  * @brief  Read data registers at an address in the peripheral
  * @param  address register to configurate
  * @param  p_buffer pointer to a data buffer
  * @param  size    amount of data to be sent
  * @retval HAL status
  */

int subghz_read_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
  uint8_t status, data;
  uint8_t rxbuf[4];
  int i;

  /* Start transaction. */
  spi_start_transaction();

  rxbuf[0] = spi_transmit(SUBGHZ_READ_REGISTER);
  rxbuf[1] = spi_transmit((address & 0xF00) >> 8);
  rxbuf[2] = spi_transmit((address & 0x00FF));
  status = spi_read_byte();
  rxbuf[3] = status;

  for (i=0; i<size; i++)
  {
    *(p_buffer++) = spi_read_byte();
  }
  
  /* End transaction. */
  spi_end_transaction();

  /* Success. */
  return status;
}

/**
  * @brief  Write data registers at an address in the peripheral
  * @param  address register to configurate
  * @param  p_buffer pointer to a data buffer
  * @param  size    amount of data to be sent
  * @retval HAL status
  */
int subghz_write_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
  /* Start SPI transaction. */
  spi_start_transaction();

  spi_transmit(SUBGHZ_RADIO_WRITE_REGISTER);
  spi_transmit((address & 0xFF00U) >> 8U);
  spi_transmit((address & 0x00FFU));

  for (uint16_t i = 0U; i < size; i++)
  {
      spi_transmit(p_buffer[i]);
  }

  /* End of transaction. */
  spi_end_transaction();

  /* Success. */
  return 0;
}

/**
  * @brief  Read data registers at an address in the peripheral
  * @param  address register to configurate
  * @retval HAL status
  */
int subghz_read_reg(uint16_t address, uint8_t *p_reg)
{
  return subghz_read_regs(address, p_reg, 1);
}

/**
  * @brief  Write data registers at an address in the peripheral
  * @param  address register to configurate
  * @retval HAL status
  */
void subghz_write_reg(uint16_t address, uint8_t value)
{
  subghz_write_regs(address, &value, 1);
}

uint8_t subghz_get_status(void)
{
  uint8_t status;

  spi_start_transaction();

  spi_transmit(SUBGHZ_GET_STATUS);
  status = spi_transmit(0x00);
  
  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_get_error(uint16_t *error)
{
  uint8_t status = 0;
  uint16_t err = 0;

  spi_start_transaction();

  rx[0] = spi_transmit(SUBGHZ_GET_ERROR);
  status = spi_read_byte();
  rx[1] = status;
  err = spi_read_byte() << 8;
  rx[2] = err>>8;
  err |= (spi_read_byte() & 0xff);
  rx[3] = err&0xff;
  *error = err;

  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_set_tcxo_mode(uint8_t trim, uint32_t timeout)
{
  uint8_t status = 0;

  spi_start_transaction();
  
  status = spi_transmit(SUBGHZ_SET_TCXO_MODE);
  spi_transmit(trim);
  spi_transmit((timeout >> 16) & 0xFF);
  spi_transmit((timeout >> 8) & 0xFF);
  spi_transmit(timeout & 0xFF);

  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_calibrate(uint8_t calib_cfg)
{
  uint8_t status = 0;

  spi_start_transaction();
  
  status = spi_transmit(SUBGHZ_CALIBRATE);
  spi_transmit(calib_cfg);

  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_set_standby_mode(uint8_t mode)
{
  uint8_t status = 0;

  spi_start_transaction();
  
  status = spi_transmit(SUBGHZ_SET_STANDBY);
  spi_transmit(mode);

  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_set_regulator_mode(uint8_t mode)
{
  uint8_t status = 0;

  spi_start_transaction();
  
  status = spi_transmit(SUBGHZ_SET_REGULATOR_MODE);
  spi_transmit(mode);

  spi_end_transaction();

  /* Success. */
  return status;
}

uint8_t subghz_write_command(uint8_t command, uint8_t *p_parameters, int params_size)
{
  uint8_t status = 0;
  int i;

  spi_start_transaction();

  spi_transmit(command);
  for (i=0; i<params_size; i++)
  {
    spi_transmit(p_parameters[i]);
  }

  spi_end_transaction();

  /* Success. */
  return status;

}

/**
 * @brief Check SUBGHZ device.
 * 
 * @return 0 on success, 1 otherwise.
 **/

int subghz_check_device_ready(void)
{
  uint32_t count;
  
  #if 0
  /* TODO: add support for Deep Sleep (wakeup Radio if deep sleep mode is enabled) */
  if (g_deepsleep_enable)
  {
    printf("Waking up SubGHz ...\n");

    /* Set NSS to 0 (active) for 24us. */
    pwr_subghzspi_select();
    
    /* Wait Radio wakeup */
    count = SUBGHZ_NSS_LOOP_TIME;
    do
    {
      count--;
    } while (count != 0UL);

    pwr_subghzspi_unselect();
  }
  #endif 
  
  return subghz_wait_on_busy();
}

int subghz_wait_on_busy(void)
{
  int status = 0;
  uint32_t count;
  uint32_t mask;

  count  = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_RFBUSY_LOOP_TIME;
  //printf("Wait until RFBUSY signal is unset ... \r\n");

  #if 0
  /* Wait until Busy signal is set */
  do
  {
    mask = pwr_is_rfbusyms();
    if (count == 0U)
    {
      status  = 1;
      printf("timeout\r\n");
      break;
    }
    count--;
  } while ((pwr_is_rfbusys() & mask) == 1UL);
  #endif
  
  while (pwr_is_rfbusys() == 1);
  
  return status;
}