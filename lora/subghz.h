#ifndef __INC_SUBGHZ_H
#define __INC_SUBGHZ_H

#include <libopencm3/stm32/spi.h>

#define SUBGHZ_DEFAULT_TIMEOUT    100U /* SUBGHZ default timeout: 100ms */
#define SUBGHZ_RFBUSY_LOOP_TIME   ((rcc_ahb_frequency*24U)>>20U)

#define SUBGHZ_RADIO_READ_REGISTER   0x1D
#define SUBGHZ_RADIO_WRITE_REGISTER  0x0D

/* Registers */
#define SUBGHZ_GBSYNCR               0x6AC
#define SUBGHZ_GPKTCTL1AR            0x6B8
#define SUBGHZ_GWHITEINIRL           0x6B9
#define SUBGHZ_GCRCINIRH             0x6BC
#define SUBGHZ_GCRCINIRL             0x6BD
#define SUBGHZ_GCRCPOLRH             0x6BE
#define SUBGHZ_GCRCPOLRL             0x6BF
#define SUBGHZ_GSYNCR0               0x6C0
#define SUBGHZ_GSYNCR1               0x6C1
#define SUBGHZ_GSYNCR2               0x6C2
#define SUBGHZ_GSYNCR3               0x6C3
#define SUBGHZ_GSYNCR4               0x6C4
#define SUBGHZ_GSYNCR5               0x6C5
#define SUBGHZ_GSYNCR6               0x6C6
#define SUBGHZ_GSYNCR7               0x6C7
#define SUBGHZ_LSYNCRH               0x740
#define SUBGHZ_LSYNCRL               0x741
#define SUBGHZ_RNGR3                 0x819
#define SUBGHZ_RNGR2                 0x81A
#define SUBGHZ_RNGR1                 0x81B
#define SUBGHZ_RNGR0                 0x81C
#define SUBGHZ_AGCRSSIC0R            0x89B
#define SUBGHZ_RXGAINCR              0x8AC
#define SUBGHZ_PAOCPR                0x8E7
#define SUBGHZ_HSEINTRIMR            0x911
#define SUBGHZ_HSEOUTTRIMR           0x912
#define SUBGHZ_SMPSC0R               0x916
#define SUBGHZ_PCR                   0x91A
#define SUBGHZ_REGDRVTSTR            0x91F
#define SUBGHZ_SMPSC2R               0x923

/* Commands */
#define SUBGHZ_CALIBRAT_IMAGE        0x98
#define SUBGHZ_CALIBRATE             0x89
#define SUBGHZ_CFG_DIO_IRQ           0x08
#define SUBGHZ_CLR_ERROR             0x07
#define SUBGHZ_CLR_IRQ_STATUS        0x02
#define SUBGHZ_GET_ERROR             0x17
#define SUBGHZ_GET_IRQ_STATUS        0x12
#define SUBGHZ_GET_PACKET_STATUS     0x14
#define SUBGHZ_GET_PACKET_TYPE       0x11
#define SUBGHZ_GET_RSSI_INST         0x15
#define SUBGHZ_GET_RX_BUFFER_STATUS  0x13
#define SUBGHZ_GET_STATS             0x10
#define SUBGHZ_GET_STATUS            0xC0
#define SUBGHZ_READ_BUFFER           0x1E
#define SUBGHZ_READ_REGISTER         0x1D
#define SUBGHZ_RESET_STATS           0x00
#define SUBGHZ_SET_BUFFER_BA         0x8F
#define SUBGHZ_SET_CAD               0xC5
#define SUBGHZ_SET_CAD_PARAMS        0x88
#define SUBGHZ_SET_FS                0xC1
#define SUBGHZ_SET_LORA_SYMB_TO      0xA0
#define SUBGHZ_SET_MOD_PARAMS        0x8B
#define SUBGHZ_SET_PA_CONFIG         0x95
#define SUBGHZ_SET_PACKET_PARAMS     0x8C
#define SUBGHZ_SET_PACKET_TYPE       0x8A
#define SUBGHZ_SET_REGULATOR_MODE    0x96
#define SUBGHZ_SET_RF_FREQ           0x86
#define SUBGHZ_SET_RX                0x82
#define SUBGHZ_SET_RX_DUTY_CYCLE     0x94
#define SUBGHZ_SET_TXRX_FALLBACK     0x93
#define SUBGHZ_SET_SLEEP             0x84
#define SUBGHZ_SET_STANDBY           0x80
#define SUBGHZ_SET_STOP_RX_TIMER     0x9F
#define SUBGHZ_SET_TCXO_MODE         0x97
#define SUBGHZ_SET_TX                0x83
#define SUBGHZ_SET_TX_CONT_WAVE      0xD1
#define SUBGHZ_SET_TX_CONT_PREAMBLE  0xD2
#define SUBGHZ_SET_TX_PARAMS         0x8E
#define SUBGHZ_WRITE_BUFFER          0x0E
#define SUBGHZ_WRITE_REGISTER        0x0D

/* Constants */
#define SUBGHZ_TCXO_1V6              0x00
#define SUBGHZ_TCXO_1V7              0x01
#define SUBGHZ_TCXO_1V8              0x02
#define SUBGHZ_TCXO_2V2              0x03
#define SUBGHZ_TCXO_2V4              0x04
#define SUBGHZ_TCXO_2V7              0x05
#define SUBGHZ_TCXO_3V0              0x06
#define SUBGHZ_TCXO_3V3              0x07

int subghz_init(void);
int subghz_check_device_ready(void);
int subghz_wait_on_busy(void);

uint8_t subghz_get_status(void);
uint8_t subghz_get_error(uint16_t *error);
uint8_t subghz_set_tcxo_mode(uint8_t trim, uint32_t timeout);
uint8_t subghz_calibrate(uint8_t calib_cfg);
uint8_t subghz_set_standby_mode(uint8_t mode);
uint8_t subghz_set_regulator_mode(uint8_t mode);

int subghz_read_reg(uint16_t address, uint8_t *p_reg);
void subghz_write_reg(uint16_t address, uint8_t value);
int subghz_read_regs(uint16_t address, uint8_t *p_buffer, uint16_t size);
int subghz_write_regs(uint16_t address, uint8_t *p_buffer, uint16_t size);
uint8_t subghz_write_command(uint8_t command, uint8_t *p_parameters, int params_size);


#endif /* __INC_SUBGHZ_H */