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


subghz_result_t spi_transmit(uint8_t data)
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

subghz_result_t subghz_read_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
  subghz_result_t status;
  uint8_t rxbuf[4], data;
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
subghz_result_t subghz_write_regs(uint16_t address, uint8_t *p_buffer, uint16_t size)
{
  subghz_result_t status;

  /* Start SPI transaction. */
  spi_start_transaction();

  status = spi_transmit(SUBGHZ_RADIO_WRITE_REGISTER);
  spi_transmit((address & 0xFF00U) >> 8U);
  spi_transmit((address & 0x00FFU));

  for (uint16_t i = 0U; i < size; i++)
  {
      spi_transmit(p_buffer[i]);
  }

  /* End of transaction. */
  spi_end_transaction();

  /* Status. */
  return status;
}

/**
  * @brief  Read data registers at an address in the peripheral
  * @param  address register to configurate
  * @retval HAL status
  */
subghz_result_t subghz_read_reg(uint16_t address, uint8_t *p_reg)
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


/**
 * @brief Send a command to the SUBGHZ transceiver.
 * 
 * @param command       Command to send
 * @param p_parameters  Pointer to an array containing the different parameters to pass to
 *                      the command
 * @param params_size   Size of the parameters array (number of parameters)
 * @return HAL status
*/

subghz_result_t subghz_write_command(uint8_t command, uint8_t *p_parameters, int params_size)
{
  subghz_result_t status = 0;
  int i;

  spi_start_transaction();

  status = spi_transmit(command);
  for (i=0; i<params_size; i++)
  {
    p_parameters[i] = spi_transmit(p_parameters[i]);
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
  return subghz_wait_on_busy();
}

int subghz_wait_on_busy(void)
{
  while (pwr_is_rfbusys() == 1);

  return 0;
}

/************************************************
 * SUBGHZ Commands API
 ***********************************************/

/**
 * Retrieve the status of the SubGHz transceiver.
 */
subghz_result_t subghz_get_status(void)
{
  subghz_result_t result;
  uint8_t params[] = {0x00};

  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_STATUS, params, 1);
  result = params[0];

  /* Success. */
  return result;
}


/**
 * @brief Retrieve the current error byte from transceiver.
 * 
 * @param error: pointer to a uint16_t error code.
 * @return current status
 */

subghz_result_t subghz_get_error(subghz_err_t *error)
{
  subghz_result_t status = 0;
  uint8_t params[] = {0x00, 0x00, 0x00};

  status = subghz_write_command(SUBGHZ_GET_ERROR, params, 3);
  *error = (params[1] << 8) | params[2];
  
  /* Success. */
  return status;
}


/**
 * @brief Set Temperature-Compensated Crystal Oscillator (TCXO) parameters
 * 
 * @param   trim    trimming value
 * @param   timeout timeout in ms after which the oscillator should be considered unstable
 * @return  status
 */
subghz_result_t subghz_set_tcxo_mode(subghz_tcxo_trim_t trim, uint32_t timeout)
{
  subghz_result_t status;
  uint8_t params[] = {
    trim,
    (timeout >> 16) & 0xFF,
    (timeout >> 8) & 0xFF,
    timeout & 0xFF
  };

  status = subghz_write_command(SUBGHZ_SET_TCXO_MODE, params, 4);

  /* Success. */
  return status;
}


/**
 * @brief Calibrate oscillators and PLLs
 * 
 * @param   calib_cfg  Calibration configuration byte 
 * @return  status
 */

subghz_result_t subghz_calibrate(uint8_t calib_cfg)
{
  subghz_result_t status = 0;
  uint8_t params[] = {calib_cfg};

  status = subghz_write_command(SUBGHZ_SET_TCXO_MODE, params, 1);

  /* Success. */
  return status;
}


/**
 * @brief Put transceiver in standby mode
 * 
 * @param mode specify the oscillator to use in standby mode (RC 13MHz or HSE32)
 * @return HAL status
 */

subghz_result_t subghz_set_standby_mode(subghz_standby_mode_t mode)
{
  subghz_result_t status = 0;
  uint8_t params[] = {mode};

  status = subghz_write_command(SUBGHZ_SET_STANDBY, params, 1);

  /* Success. */
  return status;
}


/**
 * @brief Set transceiver regulator mode
 * 
 * @param mode Transceiver regulator mode
 */

subghz_result_t subghz_set_regulator_mode(subghz_regulator_mode_t mode)
{
  subghz_result_t status = 0;
  uint8_t params[] = {mode};

  status = subghz_write_command(SUBGHZ_SET_REGULATOR_MODE, params, 1);

  /* Success. */
  return status;
}


/**
 * @brief Set transceiver in FS mode
 * 
 * @return HAL status
 */

subghz_result_t subghz_set_fs_mode(void)
{
  return subghz_write_command(SUBGHZ_SET_FS, NULL, 0);
}

/**
 * @brief Set transceiver in TX mode
 * 
 * @param   timeout     TX Timeout 
 * @return  HAL status
 */

subghz_result_t subghz_set_tx_mode(uint32_t timeout)
{
  uint8_t params[] = {
    (timeout >> 16) & 0xFF,
    (timeout >> 8) & 0xFF,
    timeout & 0xFF
  };

  return subghz_write_command(SUBGHZ_SET_TX, params, 3);
}


/**
 * @brief Set transceiver in RX mode
 * 
 * @param   timeout     RX Timeout 
 * @return  HAL status
 */

subghz_result_t subghz_set_rx_mode(uint32_t timeout)
{
  uint8_t params[] = {
    (timeout >> 16) & 0xFF,
    (timeout >> 8) & 0xFF,
    timeout & 0xFF
  };

  return subghz_write_command(SUBGHZ_SET_RX, params, 3);
}


/**
 * @brief Set stop RX timer on preamble
 * 
 * @param   config  RX timer stop condition 
 * @return  HAL status
 */

subghz_result_t subghz_set_stop_rxtimer_on_preamble(subghz_rxtimer_stop_t config)
{
  uint8_t params[] = {config};

  return subghz_write_command(SUBGHZ_SET_STOP_RX_TIMER, params, 1);
}


/**
 * @brief Set RX duty cycle (see section 5.8.3 from RM0453)
 * 
 * @param   rx_period    RX period 
 * @param   sleep_period Sleep period
 * @return  HAL status
*/

subghz_result_t subghz_set_duty_cycle(uint32_t rx_period, uint32_t sleep_period)
{
  uint8_t params[] = {
    (rx_period >> 16) & 0xff,
    (rx_period >> 8) & 0xff,
    (rx_period >> 0) & 0xff,
    (sleep_period >> 16) & 0xff,
    (sleep_period >> 8) & 0xff,
    (sleep_period >> 0) & 0xff,
  };

  return subghz_write_command(SUBGHZ_SET_RX, params, 6);
}


/**
 * @brief Enable Channel Activity Detection (only in LoRa mode)
 * 
 * @return  HAL status
 */

subghz_result_t subghz_set_cad(void)
{
  return subghz_write_command(SUBGHZ_SET_CAD, NULL, 0);
}


/**
 * @brief Set TX in continuous wave mode
 * 
 * @return  HAL status
 */

subghz_result_t subghz_set_tx_continuous_wave(void)
{
  return subghz_write_command(SUBGHZ_SET_TX_CONT_WAVE, NULL, 0);
}


/**
 * @brief Set TX in continuous preamble mode
 * 
 * @return  HAL status
 */

subghz_result_t subghz_set_tx_continuous_preamble(void)
{
  return subghz_write_command(SUBGHZ_SET_TX_CONT_PREAMBLE, NULL, 0);
}


/**
 * @brief Set packet type
 * 
 * @param   packet_type   Packet type to set
 * @return  HAL status
 */

subghz_result_t subghz_set_packet_type(subghz_packet_type_t packet_type)
{
  uint8_t params[] = {packet_type};
  return subghz_write_command(SUBGHZ_SET_PACKET_TYPE, params, 1);
}


/**
 * @brief Get current packet type
 * 
 * @param   p_packet_type   pointer to the current packet type
 * @return  HAL status
 */

subghz_result_t subghz_get_packet_type(subghz_packet_type_t *packet_type)
{
  subghz_result_t status;
  uint8_t params[] = {0x00, 0x00};
  
  subghz_write_command(SUBGHZ_GET_PACKET_TYPE, params, 2);
  status = params[0];
  *packet_type = params[1];

  return status;
}


/**
 * @brief Set RF frequency 
 * 
 * @param   freq        RF frequency parameter (see section 5.8.4 from RM0453
 *                      for frequency computation)
 * @return  HAL status
 */

subghz_result_t subghz_set_rf_freq(uint32_t freq)
{
  uint8_t params[] = {
    (freq >> 24) & 0xff,
    (freq >> 16) & 0xff,
    (freq >> 8) & 0xff,
    (freq >> 0) & 0xff
  };
  
  return subghz_write_command(SUBGHZ_SET_RF_FREQ, params, 4);
}


/**
 * @brief Set TX parameters
 * 
 * @param   power     Set transmit power in DB
 * @param   ramp_time Set PA Ramp-up time
 * @return  HAL status
*/

subghz_result_t subghz_set_tx_params(int8_t power, subghz_txparams_ramptime_t ramp_time)
{
  uint8_t params[] = {
    power,
    ramp_time
  };
  
  return subghz_write_command(SUBGHZ_SET_TX_PARAMS, params, 2);
}


/**
 * @brief Set power amplifier configuration
 * 
 * @param    duty_cycle   Set PA duty cycle (Caution, see section 5.8.4)
 * @param    hp_max       Set HP PA maximum value
 * @param    pa_sel       HP/LP selection (default, HP:1)
*/

subghz_result_t subghz_set_pa_config(uint8_t duty_cycle, uint8_t hp_max, uint8_t pa_sel)
{
  uint8_t params[] = {
    duty_cycle,
    hp_max,
    pa_sel,
    0x01
  };
  
  return subghz_write_command(SUBGHZ_SET_PA_CONFIG, params, 4);
}


/**
 * @brief   Set TX/RX fallback mode
 * 
 * @param   mode   Mode to enter when a packet is succesfully sent/received
 * @return  HAL status
*/

subghz_result_t subghz_set_tx_rx_fallback_mode(subghz_fallback_mode_t mode)
{
  uint8_t params[] = {mode};
  
  return subghz_write_command(SUBGHZ_SET_TXRX_FALLBACK, params, 1);
}


/**
 * @brief   Set CAD parameters
 * 
 * @param   nb_symbols    Number of CAD symbols
 * @param   det_peak      Detection peak value
 * @param   det_min       Detection min value
 * @param   exit_mode     Exit mode
 * @param   timeout       CAD timeout
 * @return  HAL status
*/

subghz_result_t subghz_set_cad_params(subghz_cad_symbols_t nb_symbols, uint8_t det_peak,
                                      uint8_t det_min, subghz_cad_exit_mode_t exit_mode,
                                      uint32_t timeout)
{
  uint8_t params[] = {
    nb_symbols,
    det_peak,
    det_min,
    exit_mode,
    (timeout >> 16) & 0xFF,
    (timeout >> 8) & 0xFF,
    timeout & 0xFF
  };
  
  return subghz_write_command(SUBGHZ_SET_CAD_PARAMS, params, 7);
}


/**
 * @brief Set buffer base address for RX and TX packets
 * 
 * @param   tx_base_addr    TX base address offset relative to SUBGHZ RAM base address
 * @param   rx_base_addr    RX base address offset relative to SUBGHZ RAM base address
 * @return  HAL status
*/

subghz_result_t subghz_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr)
{
  uint8_t params[] = {tx_base_addr, rx_base_addr};

  return subghz_write_command(SUBGHZ_SET_BUFFER_BA, params, 2);
}


/**
 * @brief Set FSK modulation parameters.
 * 
 * @param   bitrate     Modulation bitrate (bit/s)
 * @param   gaussian    Pulse shape (gaussian filter)
 * @param   bandwidth   RF bandwidth
 * @param   deviation   Frequency deviation
 * @return  HAL status
*/

subghz_result_t subghz_set_fsk_modulation_params(uint32_t bitrate, subghz_fsk_gaussian_t gaussian,
                                                 subghz_fsk_bandwidth_t bandwidth, uint32_t deviation)
{
  uint8_t params[] = {
    (bitrate >> 16) & 0xff,
    (bitrate >> 8) & 0xff,
    (bitrate >> 0) & 0xff,
    gaussian,
    bandwidth,
    (deviation >> 16) & 0xff,
    (deviation >> 8) & 0xff,
    (deviation >> 0) & 0xff,
  };

  return subghz_write_command(SUBGHZ_SET_MOD_PARAMS, params, 8);
}


/**
 * @brief Set LoRa modulation parameters.
 * 
 * @param   sf          Spreading factor
 * @param   bandwidth   RF bandwidth
 * @param   cr          Coding rate
 * @param   ldro        Low data rate optimization
 * @return  HAL status
*/

subghz_result_t subghz_set_lora_modulation_params(subghz_lora_sf_t sf, subghz_lora_bandwidth_t bandwidth,
                                                  subghz_lora_cr_t cr, subghz_lora_ldro_t ldro)
{
  uint8_t params[] = {
    sf,
    bandwidth,
    cr,
    ldro
  };

  return subghz_write_command(SUBGHZ_SET_MOD_PARAMS, params, 4);
}


/**
 * @brief Set generic packet parameters
 * 
 * @param   preamble_length     Size of preamble, in number of symbols
 * @param   det_length          Number of bit symbols used to detect preamble
 * @param   syncword_length     Size of synchronization word, in number of bit symbols
 * @param   addr_comp           Address comparison method
 * @param   packet_length       Type of packet (variable or fixed length)
 * @param   payload_length      Size of payload in bytes
 * @param   crc_type            CRC Size and status
 * @param   whitening           Enable/disable whitening
*/

subghz_result_t subghz_set_packet_params(uint16_t preamble_length, subghz_det_length_t det_length,
                                         uint8_t syncword_length, subghz_addr_comp_t addr_comp,
                                         subghz_packet_length_t packet_length, uint8_t payload_length,
                                         subghz_packet_crc_t crc_type, bool whitening)
{
  uint8_t params[] = {
    (preamble_length >> 8) & 0xff,
    (preamble_length >> 0) & 0xff,
    det_length,
    syncword_length,
    addr_comp,
    packet_length,
    payload_length,
    crc_type,
    whitening?1:0
  };

  return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 9);
}


/**
 * @brief Set LoRa packet parameters
 * 
 * @param   preamble_length     Preamble length in number of symbols
 * @param   header_type         variable/fixed length payload
 * @param   payload_length      Length of payload in bytes
 * @param   crc_enabled         CRC enabled if true, disabled otherwise
 * @param   invert_iq           standard IQ setup if false, inverted IQ if true
 * @return  HAL status
*/

subghz_result_t subghz_set_lora_packet_params(uint16_t preamble_length, subghz_lora_packet_hdr_t header_type,
                                              uint8_t payload_length, bool crc_enabled, bool invert_iq)
{
  uint8_t params[] = {
    (preamble_length >> 8) & 0xff,
    (preamble_length >> 0) & 0xff,
    header_type,
    payload_length,
    crc_enabled?1:0,
    invert_iq?1:0
  };

  return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 6);
}
