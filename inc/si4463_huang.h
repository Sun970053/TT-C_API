/*
 * si4463_huang.h
 *
 *  Created on: June 12, 2024
 *      Author: Ting-Shan, Huang
 */

#ifndef INC_SI4463_HUANG_H_
#define INC_SI4463_HUANG_H_

#include "radio_config_Si4463_GMSK_9600.h"
#include "radio_config_selection.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* Define debugging. If we need to enable printf, uncomment this line. */
#define DEBUG1

#ifdef DEBUG1
    #define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/* Boot Commands */
#define POWER_UP                        0x02

/* Common Commands */
#define NOP                             0x00
#define PART_INFO                       0x01
#define FUNC_INFO                       0x10
#define SET_PROPERTY                    0x11
#define GET_PROPERTY                    0x12
#define GPIO_PIN_CFG                    0x13
#define FIFO_INFO                       0x15
#define GET_INT_STATUS                  0x20
#define REQUEST_DEVICE_STATE            0x33
#define CHANGE_STATE                    0x34
#define READ_CMD_BUFFER                 0x44
#define CMD_FAST_RESPONSE_REG_A         0x50
#define CMD_FAST_RESPONSE_REG_B         0x51
#define CMD_FAST_RESPONSE_REG_C         0x53
#define CMD_FAST_RESPONSE_REG_D         0x57

/* IRCAL Commands */
#define IRCAL                           0x17
#define IRCAL_MANUAL                    0x1a

/* Transmission Commands */
#define START_TX                        0x31
#define WRITE_TX_FIFO                   0x66

/* Reception Commands */
#define GET_PACKET_INFO                 0x16
#define GET_MODEM_STATUS                0x22
#define START_RX                        0x32
#define RX_HOP                          0x36
#define READ_RX_FIFO                    0x77

/* Advanced Commands */
#define GET_ADC_READING                 0x14
#define GET_PH_STATUS                   0x21
#define GET_CHIP_STATUS                 0x23

/* Property Groups & Number */

/* Global */
#define PROP_GLOBAL_XO_TUNE             0x0000
#define PROP_GLOBAL_CONFIG              0x0003

/* Interrupt Control */
#define PROP_INT_CTL_ENABLE             0x0100
#define PROP_INT_CTL_PH_ENABLE          0x0101
#define PROP_INT_CTL_MODEM_ENABLE       0x0102
#define PROP_INT_CTL_CHIP_ENABLE        0x0103

/* Preamble */
#define PROP_PREAMBLE_TX_LENGTH         0x1000
#define PROP_PREAMBLE_CONFIG_STD_1      0x1001
#define PROP_PREAMBLE_CONFIG_NSTD       0x1002
#define PROP_PREAMBLE_CONFIG_STD_2      0x1003
#define PROP_PREAMBLE_CONFIG            0x1004
/* Sync Word */
#define PROP_SYNC_CONFIG                0x1100
#define PROP_SYNC_BITS                  0x1101
/* Packet */
#define PROP_PKT_CRC_CONFIG             0x1200
#define PROP_PKT_CONFIG_1               0x1206
#define PROP_PKT_LEN                    0x1208
#define PROP_PKT_LEN_FIELD_SOURCE       0x1209
#define PROP_PKT_FIELD_1_LENGTH         0x120D
#define PROP_PKT_FIELD_1_CONFIG         0x120F
#define PROP_PKT_FIELD_1_CRC_CONFIG     0x1210
#define PROP_PKT_FIELD_2_LENGTH         0x1211
#define PROP_PKT_FIELD_2_CONFIG         0x1213
#define PROP_PKT_FIELD_2_CRC_CONFIG     0x1214
#define PROP_PKT_FIELD_3_LENGTH         0x1215
#define PROP_PKT_FIELD_3_CONFIG         0x1217
#define PROP_PKT_FIELD_3_CRC_CONFIG     0x1218
#define PROP_PKT_FIELD_4_LENGTH         0x1219
#define PROP_PKT_FIELD_4_CONFIG         0x121B
#define PROP_PKT_FIELD_4_CRC_CONFIG     0x121C
#define PROP_PKT_FIELD_5_LENGTH         0x121D
#define PROP_PKT_FIELD_5_CONFIG         0x121F
#define PROP_PKT_FIELD_5_CRC_CONFIG     0x1220

/* Modem */
#define PROP_MODEM_MOD_TYPE             0x2000
#define PROP_MODEM_MAP_CONTROL          0x2001
#define PROP_MODEM_DSM_CTRL             0x2002
#define PROP_MODEM_DATA_RATE            0x2003
#define PROP_MODEM_TX_NCO_MODE          0x2006
#define PROP_MODEM_FREQ_DEV             0x200A
#define PROP_MODEM_FREQ_OFFSET          0x200D
#define PROP_MODEM_TX_FILTER_COEFF_8    0x200F
#define PROP_MODEM_TX_FILTER_COEFF_7    0x2010
#define PROP_MODEM_TX_FILTER_COEFF_6    0x2011
#define PROP_MODEM_TX_FILTER_COEFF_5    0x2012
#define PROP_MODEM_TX_FILTER_COEFF_4    0x2013
#define PROP_MODEM_TX_FILTER_COEFF_3    0x2014
#define PROP_MODEM_TX_FILTER_COEFF_2    0x2015
#define PROP_MODEM_TX_FILTER_COEFF_1    0x2016
#define PROP_MODEM_TX_FILTER_COEFF_0    0x2017
#define PROP_MODEM_TX_RAMP_DELAY        0x2018
#define PROP_MODEM_MDM_CTRL             0x2019
#define PROP_MODEM_IF_CONTROL           0x201A
#define PROP_MODEM_IF_FREQ              0x201B
#define PROP_MODEM_CLKGEN_BAND          0x2051

/* Power Ampliifier */
#define PROP_PA_MODE                    0x2200
#define PROP_PA_PWR_LVL                 0x2201
#define PROP_PA_BIAS_CLKDUTY            0x2202
#define PROP_PA_TC                      0x2203

/* Frequency Control */
#define PROP_FREQ_CONTROL_INTE          0x4000
#define PROP_FREQ_CONTROL_FRAC          0x4001
#define PROP_FREQ_CONTROL_CHANNEL_STEP_SIZE 0x4004
#define PROP_FREQ_CONTROL_W_SIZE        0x4006
#define PROP_FREQ_CONTROL_VCOCNT_RX_ADJ 0x4007

/* microsecond */
#define SI4463_SPI_TIMEOUT              2000
#define MAX_CTS_RETRY                   2000
#define SI4463_TRANSMIT_TIMEOUT         2000 /* in ms */
#define SI4463_RECEIVE_TIMEOUT          2000 /* in ms */

//#define GPIO_PIN_CFG                     0x13
typedef enum
{
    GPIO_NO_CHANGE =                0,
    GPIO_DISABLE =                  1,
    GPIO_OUPPUT_LOW =               2,
    GPIO_OUTPUT_HIGH =              3,
    GPIO_INPUT =                    4,
    GPIO_32_KHZ_CLOCK =             5,
    GPIO_DATA_OUT =                 11,
    GPIO_TX_STATE =                 32,
    GPIO_RX_STATE =                 33,
    GPIO_INT_SIGNAL =               39
} si4463_gpio_mode;

//#define CHANGE_STATE                     0x34
typedef enum
{
    STATE_NO_CHANGE,
	STATE_SLEEP,
	STATE_SPI_ACTIVE,
	STATE_READY,
	STATE_READY_2,
	STATE_TX_TUNE,
	STATE_RX_TUNE,
	STATE_TX,
	STATE_RX
} si4463_state;

//#define PROP_INT_CTL_ENABLE              0x0100
#define CHIP_INT_STATUS_EN               0x04
#define MODEM_INT_STATUS_EN              0x02
#define PH_INT_STATUS_EN                 0x01

//#define PROPERTY_INT_CTL_PH_ENABLE       0x0101
#define FILTER_MATCH_EN                  0x80
#define FILTER_MISS_EN                   0x40
#define PACKET_SENT_EN                   0x20
#define PACKET_RX_EN                     0x10
#define CRC_ERROR_EN                     0x08
#define TX_FIFO_ALMOST_EMPTY_EN          0x02
#define RX_FIFO_ALMOST_FULL_EN           0x01

//#define PROP_INT_CTL_MODEM_ENABLE        0x0102
#define INVALID_SYNC_EN                  0x20
#define RSSI_JUMP_EN                     0x10
#define RSSI_EN                          0x08
#define INVALID_PREAMBLE_EN              0x04
#define PREAMBLE_DETECT_EN               0x02
#define SYNC_DETECT_EN                   0x01

//#define PROP_INT_CTL_CHIP_ENABLE         0x0103
#define FIFO_UNDERFLOW_OVERFLOW_ERROR_EN 0x20
#define STATE_CHANGE_EN                  0x10
#define CMD_ERROR_EN                     0x08
#define CHIP_READY_EN                    0x04
#define LOW_BATT_EN                      0x02
#define WUT_EN                           0x01

//#define PROP_PKT_CRC_CONFIG             0x1200
typedef enum
{
    NO_CRC,
    ITU_T_CRC8,
    IEC_16,
    BAICHEVA_16,
    CRC_16_IBM,
    CCITT_16,
    KOOPMAN,
    IEEE_802_3,
    CASTAGNOLI,
    CRC_16_DNP
} si4463_crc_poly;

//#define PROP_MODEM_TX_NCO_MODE          0x2006
//Sets the oversampling ratio of the internal NCO clock signal used to synthesize the Gaussian filtered modulation waveform. This field is only effective in GFSK mode.
typedef enum
{
    OVERSAMPLING_RATIO_10,
    OVERSAMPLING_RATIO_40,
    OVERSAMPLING_RATIO_20,
} si4463_txosr;

//#define PROP_MODEM_CLKGEN_BAND          0x2051
typedef enum
{
    FVCO_DIV_4,
    FVCO_DIV_6,
    FVCO_DIV_8,
    FVCO_DIV_12,
    FVCO_DIV_16,
    FVCO_DIV_24,
    FVCO_DIV_24_2,
    FVCO_DIV_24_3
} si4463_fvco_div;

//#define PROP_PA_MODE                    0x2200
typedef enum
{
    MOD_CW,
	MOD_OOK,
	MOD_2FSK,
	MOD_2GFSK,
	MOD_4FSK,
	MOD_4GFSK
} si4463_mod_type;

/* Error codes. Negative number, because it can be distinguished by returning a value. */
#define SI4463_OK                        (0)
#define SI4463_ERR_INVALID_INPUT         (-1)
#define SI4463_ERR_INVALID_NOP           (-2)
#define SI4463_ERR_READ_REG              (-10)
#define SI4463_ERR_WRITE_REG             (-11)
#define SI4463_INIT_TIMEOUT              (-20)
#define SI4463_CTS_TIMEOUT               (-21)
#define SI4463_TX_TIMEOUT                (-30)
#define SI4463_ERR_OVER_TX_FIFO          (-31)
#define SI4463_RX_TIMEOUT                (-40)
#define SI4463_ERR_PREV_CMD_UNDONE       (-41)
#define SI4463_ERR_BAD_PROP_ID           (-42)
#define SI4463_ERR_READ_RXBUFF		     (-43)
#define SI4463_ERR_BAD_PARAM             (-50)
#define SI4463_ERR_INVALID_MOD           (-51)
#define SI4463_ERR_INVALID_DR            (-52)
#define SI4463_ERR_INVALID_TXOSR         (-53)
#define SI4463_ERR_CHIP_VERSION          (-127)

/* Data rate */
typedef enum
{
    DR_1200,
    DR_2400,
    DR_4800,
    DR_9600
} si4463_data_rate;

/* Tx power */
typedef enum
{
    PWR_0_dBm,
    PWR_4_dBm,
    PWR_10_dBm,
    PWR_14_dBm,
    PWR_20_dBm
} si4463_pa_pwr_lvl;

typedef struct
{
    /* PH */
    bool filterMatch;
    bool filterMiss;
    bool packetSent;
    bool packetRx;
    bool crcError;
    bool txFifoAlmostEmpty;
    bool rxFifoAlmostFull;

    /* Modem */
    bool postambleDetect;
    bool invalidSync;
    bool rssiJump;
    bool rssi;
    bool invalidPreamble;
    bool preambleDetect;
    bool syncDetect;

    /* Chip */
    bool cal;
    bool fifoUnderflowOverflowError;
    bool stateChange;
    bool cmdError;
    bool chipReady;
    bool lowBatt;
    bool wut;
} si4463_interrupt_handler_t;

/*
 * GPIO status handler
 * Please document the configuration registers for 
 * each Digital Input/Output (GPIO) pin and provide the 
 * current status, distinguishing between low and high states,
 * for each of these pins.
 */
typedef struct
{
    bool                        (*IRQ)();
    bool                        (*GPIO0)();
    bool                        (*GPIO1)();
    bool                        (*GPIO2)();
    bool                        (*GPIO3)();
    bool                        gpio_high;
    bool                        gpio_low;
} si4463_gpio_handler_t;

typedef struct
{
    uint8_t                     chipRev;
    uint16_t                    partNum;
    uint8_t                     partBuild;
    uint16_t                    Id;
    uint8_t                     CustomerId;
    uint8_t                     ROMId;
} si4463_part_info_t;

typedef struct
{
    uint8_t                     revExt;
    uint8_t                     revBranch;
    uint8_t                     revInt;
    uint16_t                    patch;
    uint8_t                     func;
} si4463_func_info_t;

/* Frequency config */
typedef struct
{
    uint8_t                     freq_inte;
    uint32_t                    freq_frac;
    uint8_t                     n_presc;
    uint8_t                     outdiv;
} si4463_freq_t;

/* Data rate config */
typedef struct
{
    uint32_t modemDataRate;
    uint32_t modemTxNCOMode;
    si4463_txosr TxOSR;
} si4463_data_rate_t;


typedef struct
{
    si4463_mod_type             txMod;
    si4463_mod_type             rxMod;
    si4463_data_rate            txDataRate;
    si4463_data_rate            rxDataRate;
    si4463_pa_pwr_lvl           power;
    uint32_t                    frequency;
    uint8_t                     preambleNum;
    uint8_t                     syncWords[4];
    si4463_crc_poly             crcPoly;
    bool                        crcSeed;

} si4463_setting_t;

typedef struct
{
    int8_t currentRSSI;
    int8_t latchRSSI;
} si4463_status_t;


typedef struct
{
    uint8_t                     (*SPI_Write)(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                     (*SPI_Read)(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                     (*SPI_WriteRead)(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                     (*SPI_CheckState)(void);
    uint8_t                     spi_state_ready;
    void                        (*DelayUs)(uint32_t delay);
    void                        (*NSEL)(bool val);
    void                        (*SDN)(bool val);
    void                        (*OOK)(bool val);
    si4463_interrupt_handler_t  interrupts;
    si4463_gpio_handler_t       gpios;
    si4463_part_info_t          partInfo;
    si4463_func_info_t          funInfo;
    si4463_freq_t               freq;
    si4463_data_rate_t          dr;
    si4463_setting_t            settings;
    si4463_status_t             status;
} si4463_t;

int8_t si4463_powerOnReset(si4463_t* si4463);
int8_t si4463_init(si4463_t* si4463);
int8_t si4463_checkNop(si4463_t* si4463);
int8_t si4463_getPartInfo(si4463_t* si4463);
int8_t si4463_getFuncInfo(si4463_t* si4463);
int16_t si4463_getTxFifoInfo(si4463_t* si4463);
int16_t si4463_getRxFifoInfo(si4463_t* si4463);
int8_t si4463_getCurrentRSSI(si4463_t* si4463);
int8_t si4463_getLatchRSSI(si4463_t* si4463);
int8_t si4463_clearTxFifo(si4463_t* si4463);
int8_t si4463_clearRxFifo(si4463_t* si4463);
int8_t si4463_clearInterrupts(si4463_t* si4463);
int8_t si4463_getInterrupts(si4463_t* si4463);
int8_t si4463_clearChipStatus(si4463_t* si4463);
int8_t si4463_transmit(si4463_t* si4463, uint8_t* txData, uint8_t txDataLen, si4463_state nextState);
int8_t si4463_initRx(si4463_t* si4463, uint16_t dataLen, si4463_state nextStateAfterTimeOut, si4463_state nextStateAfterValid, si4463_state nextStateAfterInvalid);
int8_t si4463_receive(si4463_t* si4463, uint8_t* rxData, uint8_t rxDataLen);
int8_t si4463_setTxPower(si4463_t* si4463, uint8_t power);
int8_t si4463_getTxPower(si4463_t* si4463);
int8_t si4463_setPreamble(si4463_t* si4463, uint8_t preambleLen);
int16_t si4463_getPreamble(si4463_t* si4463);
int8_t si4463_setSyncWords(si4463_t* si4463, uint8_t* syncdata, uint8_t syncLen);
int8_t si4463_getSyncWords(si4463_t* si4463);
int8_t si4463_setCRC(si4463_t* si4463, bool crcSeed, si4463_crc_poly crcPoly);
int8_t si4463_getCRC(si4463_t* si4463);
int8_t si4463_setFrequency(si4463_t* si4463, uint32_t freq);
int32_t si4463_getFrequency(si4463_t* si4463);
int8_t si4463_setTxModulation(si4463_t* si4463, si4463_mod_type mod);
int8_t si4463_setRxModulation(si4463_t* si4463, si4463_mod_type mod);
int8_t si4463_getModulation(si4463_t* si4463);
int8_t si4463_setTxDataRate(si4463_t* si4463, si4463_data_rate dataRate);
int8_t si4463_setRxDataRate(si4463_t* si4463, si4463_data_rate dataRate);
int16_t si4463_getDataRate(si4463_t* si4463);
int8_t si4463_enterStandbyMode(void);
int8_t si4463_startTx(si4463_t* si4463, uint16_t dataLen, si4463_state nextState);
void si4463_controlOOK(si4463_t* si4463, bool toneOn);

#endif
