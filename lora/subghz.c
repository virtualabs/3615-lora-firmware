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
 * @retval 0 on success, 1 otherwise.
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
  uint8_t data;
  int i;

  /* Start transaction. */
  spi_start_transaction();

  spi_transmit(SUBGHZ_READ_REGISTER);
  spi_transmit((address & 0xF00) >> 8);
  spi_transmit((address & 0x00FF));
  status = spi_read_byte();
  
  for (i=0; i<size; i++)
  {
    *(p_buffer++) = spi_read_byte();
  }
  
  /* End transaction. */
  spi_end_transaction();

  /*??Success. */
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

  /*??Status. */
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
 * @brief Write TX buffer
 * 
 * @param   offset  Offset in the TX exchange memory (circular buffer)
 * @param   p_data  Pointer to the data to write
 * @param   length  Number of bytes to write
 * @retval  HAL status
*/

subghz_result_t subghz_write_buffer(uint8_t offset, uint8_t *p_data, int length)
{
  subghz_result_t status = 0;
  int i;

  spi_start_transaction();

  /* Send command. */
  status = spi_transmit(SUBGHZ_WRITE_BUFFER);

  /* Send offset. */
  spi_transmit(offset);

  /* Send buffer. */
  for (i=0; i<length; i++)
  {
    spi_transmit(p_data[i]);
  }

  spi_end_transaction();

  /*??Success. */
  return status;

}


/**
 * @brief Read RX buffer
 * 
 * @param   offset  Offset in the RX exchange memory (circular buffer)
 * @param   p_data  Pointer to the data to write
 * @param   length  Number of bytes to write
 * @retval  HAL status
*/

subghz_result_t subghz_read_buffer(uint8_t offset, uint8_t *p_data, int length)
{
  subghz_result_t status = 0;
  int i;

  spi_start_transaction();

  /* Send command. */
  spi_transmit(SUBGHZ_READ_BUFFER);

  /* Send offset. */
  spi_transmit(offset);

  /* Read status. */
  status = spi_transmit(0);

  /* Read buffer. */
  for (i=0; i<length; i++)
  {
    p_data[i] = spi_transmit(0);
  }

  spi_end_transaction();

  /*??Success. */
  return status;
}


/**
 * @brief Send a command to the SUBGHZ transceiver.
 * 
 * @param command       Command to send
 * @param p_parameters  Pointer to an array containing the different parameters to pass to
 *                      the command
 * @param params_size   Size of the parameters array (number of parameters)
 * @retval HAL status
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

  /*??Success. */
  return status;

}


/**
 * @brief Check SUBGHZ device.
 * 
 * @retval 0 on success, 1 otherwise.
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

  /*??Success. */
  return result;
}


/**
 * @brief Get RX buffer status (payload length and offset)
 * 
 * @param   p_rxbuf_status   pointer to a `subghz_rxbuf_status_t` structure that will
 *                           be filled by this function
 * @retval  HAL status
*/

subghz_result_t subghz_get_rxbuf_status(subghz_rxbuf_status_t *p_rxbuf_status)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00};
  
  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_RX_BUFFER_STATUS, params, 3);

  /* Copy response into structure. */
  result = params[0];
  p_rxbuf_status->payload_length = params[1];
  p_rxbuf_status->buffer_offset = params[2];

  /* Return status. */
  return result;
}


/**
 * @brief Retrieve the current error byte from transceiver.
 * 
 * @param  error pointer to a uint16_t error code.
 * @retval current status
 */

subghz_result_t subghz_get_error(subghz_err_t *error)
{
  subghz_result_t status = 0;
  uint8_t params[] = {0x00, 0x00, 0x00};

  status = subghz_write_command(SUBGHZ_GET_ERROR, params, 3);
  *error = (params[1] << 8) | params[2];
  
  /*??Success. */
  return status;
}


/**
 * @brief Set Temperature-Compensated Crystal Oscillator (TCXO) parameters
 * 
 * @param   trim    trimming value
 * @param   timeout timeout in ms after which the oscillator should be considered unstable
 * @retval  status
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

  /*??Success. */
  return status;
}


/**
 * @brief Calibrate oscillators and PLLs
 * 
 * @param   calib_cfg  Calibration configuration byte 
 * @retval  status
 */

subghz_result_t subghz_calibrate(uint8_t calib_cfg)
{
  subghz_result_t status = 0;
  uint8_t params[] = {calib_cfg};

  status = subghz_write_command(SUBGHZ_SET_TCXO_MODE, params, 1);

  /*??Success. */
  return status;
}


/**
 * @brief Set sleep mode
 * 
 * @param   sleep_cfg   Sleep mode configuration bits
 * @retval  HAL status
*/

subghz_result_t subghz_set_sleep(uint8_t sleep_cfg)
{
  subghz_result_t status = 0;
  uint8_t params[] = {sleep_cfg};

  status = subghz_write_command(SUBGHZ_SET_SLEEP, params, 1);

  /*??Success. */
  return status;
}


/**
 * @brief Put transceiver in standby mode
 * 
 * @param mode specify the oscillator to use in standby mode (RC 13MHz or HSE32)
 * @retval HAL status
 */

subghz_result_t subghz_set_standby_mode(subghz_standby_mode_t mode)
{
  subghz_result_t status = 0;
  uint8_t params[] = {mode};

  status = subghz_write_command(SUBGHZ_SET_STANDBY, params, 1);

  /*??Success. */
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

  /*??Success. */
  return status;
}


/**
 * @brief Set transceiver in FS mode
 * 
 * @retval HAL status
 */

subghz_result_t subghz_set_fs_mode(void)
{
  return subghz_write_command(SUBGHZ_SET_FS, NULL, 0);
}

/**
 * @brief Set transceiver in TX mode
 * 
 * @param   timeout     TX Timeout 
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
 */

subghz_result_t subghz_set_cad(void)
{
  return subghz_write_command(SUBGHZ_SET_CAD, NULL, 0);
}


/**
 * @brief Set TX in continuous wave mode
 * 
 * @retval  HAL status
 */

subghz_result_t subghz_set_tx_continuous_wave(void)
{
  return subghz_write_command(SUBGHZ_SET_TX_CONT_WAVE, NULL, 0);
}


/**
 * @brief Set TX in continuous preamble mode
 * 
 * @retval  HAL status
 */

subghz_result_t subghz_set_tx_continuous_preamble(void)
{
  return subghz_write_command(SUBGHZ_SET_TX_CONT_PREAMBLE, NULL, 0);
}


/**
 * @brief Set packet type
 * 
 * @param   packet_type   Packet type to set
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @retval  HAL status
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
 * @brief Set BSPK modulation parameters
 * 
 * @param   bitrate Bitrate
 * @param   gaussian Gaussian filter setting
 * @retval  HAL status
*/

subghz_result_t subghz_set_bpsk_modulation_params(uint32_t bitrate, subghz_bpsk_gaussian_t gaussian)
{
  uint8_t params[] = {
    (bitrate >> 16) & 0xff,
    (bitrate >> 8) & 0xff,
    (bitrate >> 0) & 0xff,
    gaussian
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
 * @retval  HAL status
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


/**
 * @brief Set BPSK packet parameters (payload length)
 * 
 * @param   payload_length maximum length for BSPK payload
 * @retval  HAL status
 */

subghz_result_t subghz_set_bpsk_packet_params(uint8_t payload_length)
{
  uint8_t params[] = {
    payload_length
  };

  return subghz_write_command(SUBGHZ_SET_PACKET_PARAMS, params, 1);
}


/**
 * @brief Set LoRa symbol timeout
 * 
 * Set number of LoRa symbols to be received before starting the reception of a
 * LoRa packet
 * 
 * @param   symb_num  Number of LoRa symbols (0-255)
 * @retval  HAL status
 */

subghz_result_t subghz_set_lora_symb_timeout(uint8_t symb_num)
{
  uint8_t params[] = {
    symb_num
  };

  return subghz_write_command(SUBGHZ_SET_LORA_SYMB_TO, params, 1);
}


/**
 * @brief Get RX buffer status (payload length and offset)
 * 
 * @param   p_rxbuf_status   pointer to a `subghz_rxbuf_status_t` structure that will
 *                              be filled by this function
 * @retval  HAL status
*/

subghz_result_t subghz_get_fsk_packet_status(subghz_fsk_packet_status_t *p_packet_status)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00, 0x00};
  
  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_PACKET_STATUS, params, 4);

  /* Copy response into structure. */
  result = params[0];
  p_packet_status->rx_status = params[1];
  p_packet_status->rssi_sync = params[2];
  p_packet_status->rssi_avg = params[3];

  /* Return status. */
  return result;
}


/**
 * @brief Get LoRa packet status.
 * 
 * @param   p_packet_status  pointer to a `subghz_lora_packet_status_t` structure that will be
 *                           filled by this function.
 * @retval  HAL status
 */

subghz_result_t subghz_get_lora_packet_status(subghz_lora_packet_status_t *p_packet_status)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00, 0x00};
  
  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_PACKET_STATUS, params, 4);

  /* Copy response into structure. */
  result = params[0];
  p_packet_status->rssi = params[1];
  p_packet_status->snr = params[2];
  p_packet_status->signal_rssi = params[3];

  /* Return status. */
  return result;
}


/**
 * @brief Get instantaneous RSSI
 * 
 * @param   p_rssi_inst pointer to a `uint8_t` that will receive the instantaneous RSSI value
 * @retval  HAL status
 */

subghz_result_t subghz_get_rssi_inst(uint8_t *p_rssi_inst)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00};
  
  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_RSSI_INST, params, 2);

  result = params[0];
  *p_rssi_inst = params[1];

  /* Return status. */
  return result; 
}


/**
 * @brief Get FSK status
 * 
 * @param   p_fsk_stats pointer to a `subghz_fsk_stats_t` struct that will be filled
 *                       by this function.
 * @retval  HAL status
*/

subghz_result_t subghz_get_fsk_stats(subghz_fsk_stats_t *p_fsk_stats)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_STATS, params, 7);

  /* Extract result and data. */
  result = params[0];
  p_fsk_stats->nb_packets_recvd = (params[1] << 8) | params[2];
  p_fsk_stats->nb_packets_crcerr = (params[3] << 8) | params[4];
  p_fsk_stats->nb_packets_lenerr = (params[5] << 8) | params[6];

  /* Return status. */
  return result; 
}



/**
 * @brief Get LoRa status
 * 
 * @param   p_lora_stats pointer to a `subghz_lora_stats_t` struct that will be filled
 *                       by this function.
 * @retval  HAL status
*/

subghz_result_t subghz_get_lora_stats(subghz_lora_stats_t *p_lora_stats)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Send command, receive response in params. */
  subghz_write_command(SUBGHZ_GET_STATS, params, 7);

  /* Extract result and data. */
  result = params[0];
  p_lora_stats->nb_packets_recvd = (params[1] << 8) | params[2];
  p_lora_stats->nb_packets_crcerr = (params[3] << 8) | params[4];
  p_lora_stats->nb_packets_headerr = (params[5] << 8) | params[6];

  /* Return status. */
  return result; 
}


/**
 * @brief Reset stats counters for FSK & LoRa
 * 
 * @retval  HAL status
*/

subghz_result_t subghz_reset_stats(void)
{
  uint8_t params[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Send command. */
  return subghz_write_command(SUBGHZ_RESET_STATS, params, 6);
}


/**
 * @brief Configure IRQ sources
 * 
 * @param   irq_mask  Global IRQ interrupt mask
 * @param   irq1_mask IRQ1 interrupt mask
 * @param   irq2_mask IRQ2 interrupt mask
 * @param   irq3_mask IRQ3 interrupt mask
 * @retval  HAL status
*/

subghz_result_t subghz_config_dio_irq(uint16_t irq_mask, uint16_t irq1_mask,
                                      uint16_t irq2_mask, uint16_t irq3_mask)
{
  uint8_t params[] = {
    (irq_mask >> 8) & 0xff,
    (irq_mask >> 0) & 0xff,
    (irq1_mask >> 8) & 0xff,
    (irq1_mask >> 0) & 0xff,
    (irq2_mask >> 8) & 0xff,
    (irq2_mask >> 0) & 0xff,
    (irq3_mask >> 8) & 0xff,
    (irq3_mask >> 0) & 0xff
  };

  /* Send command. */
  return subghz_write_command(SUBGHZ_CFG_DIO_IRQ, params, 8);
}


/**
 * @brief Get IRQ status
 * 
 * @param   p_irq_status  Pointer to a `subghz_irq_status_t` structure that
 *                        will be filled by this function.
 * @retval  HAL status
*/

subghz_result_t subghz_get_irq_status(subghz_irq_status_t *p_irq_status)
{
  subghz_result_t result;
  uint8_t params[] = {0x00, 0x00, 0x00};

  /* Send command. */
  subghz_write_command(SUBGHZ_GET_IRQ_STATUS, params, 3);

  result = params[0];
  p_irq_status->word = (params[1] << 8) | params[2];

  /* Return result. */
  return result;
}


/**
 * @brief Clear IRQ
 * 
 * @param   p_irq_status  Pointer to a `subghz_irq_status_t` structure that
 *                        will be filled by this function.
 * @retval  HAL status
*/

subghz_result_t subghz_clear_irq_status(subghz_irq_status_t *p_irq_status)
{
  subghz_result_t result;
  uint8_t params[] = {
    0x00,
    (p_irq_status->word >> 8) & 0xff,
    (p_irq_status->word >> 0) & 0xff,
  };

  /* Send command. */
  return subghz_write_command(SUBGHZ_CLR_IRQ_STATUS, params, 3);

  /* Return result. */
  return result;
}


/*********************************************
 * SubGHZ Driver API
 ********************************************/


/**
 * @brief Set TX payload
 * 
 * @param   payload   Pointer to the payload to send
 * @param   size      Payload size
 * @retval  HAL status
*/

subghz_result_t subghz_set_payload(uint8_t *payload, uint8_t size)
{
  /* Use offset 0 by default. */
  subghz_write_buffer(0x00, payload, size);
}


/********************************************
 * LoRa TX Test
 *******************************************/

void test_lora_tx(void)
{
  uint32_t chan = 0;
  uint8_t payload[] = "Hello world !";

  #define RF_SW_CTRL1_PIN                          GPIO4
  #define RF_SW_CTRL1_GPIO_PORT                    GPIOA
  #define RF_SW_CTRL2_PIN                          GPIO5
  #define RF_SW_CTRL2_GPIO_PORT                    GPIOA

  printf("Sending LoRa frame:\r\n");
  
  /* Tx procedure */
  if (SUBGHZ_CMD_FAILED(subghz_set_buffer_base_address(0, 0)))
  {
    printf("Cannot set buffer base address\r\n");
    return;
  }
  else
  {
    printf("Buffer base address set.\r\n");
  }
  
  if (SUBGHZ_CMD_FAILED(subghz_set_payload(payload, 13)))
  {
    printf("Cannot set payload\r\n");
    return;  
  }
  else
  {
    printf("Payload set.\r\n");
  }

  if (SUBGHZ_CMD_FAILED(subghz_set_packet_type(SUBGHZ_PACKET_LORA)))
  {
    printf("Cannot set packet type\r\n");
    return;
  }
  else
  {
    printf("Packet type set.\r\n");
  }

  if(SUBGHZ_CMD_FAILED(subghz_set_lora_packet_params(12, SUBGHZ_PKT_FIXED_LENGTH, 13, false, false)))
  {
    printf("Cannot set LoRa packet params\r\n");
    return;
  }
  else
  {
    printf("LoRa packet params set.\r\n");
  }

  subghz_write_reg(SUBGHZ_GSYNCR0, 0x12);
  subghz_write_reg(SUBGHZ_GSYNCR1, 0x34);
  subghz_write_reg(SUBGHZ_GSYNCR2, 0x56);
  subghz_write_reg(SUBGHZ_GSYNCR3, 0x78);

  SX_FREQ_TO_CHANNEL(chan, 865200000);
  if(SUBGHZ_CMD_FAILED(subghz_set_rf_freq(chan)))
  {
    printf("Cannot set RF freq\r\n");
    return;
  }
  else
  {
    printf("RF freq set.\r\n");
  }

  if(SUBGHZ_CMD_FAILED(subghz_set_pa_config(6, 0, 1)))
  {
    printf("Cannot set RF PA config\r\n");
    return;
  }
  else
  {
    printf("PA config set.\r\n");
  }

  if(SUBGHZ_CMD_FAILED(subghz_set_tx_params(0x0E, SUBGHZ_TXPARAMS_RAMPTIME_200US)))
  {
    printf("Cannot set TX params\r\n");
    return;
  }
  else
  {
    printf("TX params set.\r\n");
  }

  if(SUBGHZ_CMD_FAILED(subghz_set_lora_modulation_params(SUBGHZ_LORA_SF5, SUBGHZ_LORA_BW250,
                                    SUBGHZ_LORA_CR_48, SUBGHZ_LORA_LDRO_DISABLED)))
  {
    printf("Cannot set LoRa modulation params\r\n");
    return;
  }
  else
  {
    printf("LoRa modulation params set.\r\n");
  }

  if(SUBGHZ_CMD_FAILED(subghz_config_dio_irq(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                        IRQ_RADIO_NONE, IRQ_RADIO_NONE)))
  {
    printf("Cannot set DIO IRQ\r\n");
    return;    
  }
  else
  {
    printf("DIO IRQ set.\r\n");
  }
  
  /* TX mode, low power */
	gpio_mode_setup(RF_SW_CTRL1_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL1_PIN);
  gpio_mode_setup(RF_SW_CTRL2_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL2_PIN);
  gpio_set(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);
  gpio_set(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);

  /* Set TX */
  if(SUBGHZ_CMD_FAILED(subghz_set_tx_mode(0)))
  {
    printf("Cannot set TX mode\n");
    return;
  }
  else
  {
    printf("TX mode set.\r\n");
  }
}


/*********************************************
 * RADIO IRQ Handling
 ********************************************/

void radio_isr(void)
{
  subghz_irq_status_t irq_status;

  /* Get IRQ status */
  subghz_get_irq_status(&irq_status);

  if (irq_status.bits.tx_done)
  {
    printf("TX done!\r\n");
    subghz_clear_irq_status((1 << 0));
    subghz_set_sleep(4);
  }
}