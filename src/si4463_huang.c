/*
 * si4463_huang.c
 *
 *  Created on: June 13, 2024
 *      Author: Ting-Shan, Huang
 */

#include "si4463_huang.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

int8_t si4463_sendCommand(si4463_t* si4463, uint8_t* cmdData, uint8_t cmdLen);
int8_t si4463_waitforCTS(si4463_t* si4463);
int8_t si4463_getResponse(si4463_t* si4463, uint8_t* respData, uint8_t respLen);
int8_t si4463_writeTxFiFo(si4463_t* si4463, uint8_t* txFifoData, uint8_t txFifoLen);
int8_t si4463_readRxDataBuff(si4463_t* si4463, uint8_t* rxFifoData, uint8_t rxFifoLength);
// int8_t si4463_getFastResponseReg(uint8_t* regVal, uint8_t startRegs, uint8_t regsNum);
int8_t si4463_getFreqConfig(si4463_t* si4463);
int8_t si4463_getDataRateConfig(si4463_t* si4463);
int8_t si4463_configArray(si4463_t* si4463, uint8_t* configArray);
int8_t si4463_setProperties(si4463_t* si4463, uint8_t* setData, uint8_t setDataLen, uint16_t propNum);
int8_t si4463_getProperties(si4463_t* si4463, uint8_t* getData, uint8_t getDataLen, uint16_t propNum);
int8_t si4463_txInterrupt(si4463_t* si4463);
int8_t si4463_rxInterrupt(si4463_t* si4463);

uint8_t SI4463_CONFIGURATION_DATA[] = RADIO_CONFIGURATION_DATA_ARRAY;
uint8_t SI4463_CONFIGURATION_DATA_DIRRX[] = RADIO_CONFIGURATION_DATA_ARRAY_DIR_RX;

modembuffer_t modem_table[MOD_TYPE_COUNT][DR_COUNT] = {0};

uint8_t gcmd[SI4463_MAX_FIFO_SIZE + 1];

int8_t si4463_powerOnReset(si4463_t* si4463)
{
    si4463->SDN(true);
    si4463->DelayUs(10);
    si4463->SDN(false);

    // Wait for POR (Power on reset) on GPIO1. The delay threshold is 10 ms.
    int count = 0;
    while(si4463->gpios.GPIO1() == si4463->gpios.gpio_low)
    {
        if(count < 10000)
        {
            count++;
            si4463->DelayUs(1);
        }
        else
            return SI4463_INIT_TIMEOUT;
    }
    // This first SPI transaction has to take less than 4ms (NSEL LOW time).
    // If it cannot be guaranteed, send a shorter command (e.g. NOP) first,
    // check CTS, then send POWER_UP or patch.
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;
    return si4463_checkNop(si4463);
}

int8_t si4463_init(si4463_t* si4463)
{
    int res = si4463_powerOnReset(si4463);
    if(res != SI4463_OK) return res;
    // Start to configurate the radio
    res = si4463_configArray(si4463, SI4463_CONFIGURATION_DATA);
    if(res != SI4463_OK) return res;

    si4463->settings.txDataRate = DR_9600;
    si4463->settings.rxDataRate = DR_9600;
    si4463->settings.txMod = MOD_GMSK;
    si4463->settings.rxMod = MOD_GMSK;
    return SI4463_OK;
}

int8_t si4463_checkNop(si4463_t* si4463)
{
    uint8_t cmd = NOP;

    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
	if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

	return SI4463_OK;
}

int8_t si4463_gpioPinCfg(si4463_t* si4463, si4463_gpio_mode gpio0, si4463_gpio_mode gpio1, si4463_gpio_mode gpio2, si4463_gpio_mode gpio3)
{
    uint8_t cmd[8] = {0};
    cmd[0] = GPIO_PIN_CFG;
    cmd[1] = gpio0;
    cmd[2] = gpio1;
    cmd[3] = gpio2;
    cmd[4] = gpio3;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;

    if(!si4463_sendCommand(si4463, cmd, 8)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_getPartInfo(si4463_t* si4463)
{
    uint8_t cmd = PART_INFO;
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;
    DEBUG_PRINTF("rxbuff: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5], rxbuff[6], rxbuff[7]);
    if((uint16_t)((rxbuff[1] << 8) | rxbuff[2]) != 0x4463) return SI4463_ERR_CHIP_VERSION;
    si4463->partInfo.chipRev = rxbuff[0];
    si4463->partInfo.partNum = (rxbuff[1] << 8) | rxbuff[2];
    si4463->partInfo.partBuild = rxbuff[3];
    si4463->partInfo.Id = (rxbuff[4] << 8) | rxbuff[5];
    si4463->partInfo.CustomerId = rxbuff[6];
    si4463->partInfo.ROMId = rxbuff[7];
    return SI4463_OK;
}

int8_t si4463_getFuncInfo(si4463_t* si4463)
{
    uint8_t cmd = FUNC_INFO;
    uint8_t rxbuff[6] = {0};
    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 6)) return SI4463_ERR_READ_REG;
    DEBUG_PRINTF("rxbuff: %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5]);
    si4463->funInfo.revExt = rxbuff[0];
    si4463->funInfo.revBranch = rxbuff[1];
    si4463->funInfo.revInt = rxbuff[2];
    si4463->funInfo.patch = (rxbuff[3] << 8) | rxbuff[4];
    si4463->funInfo.func = rxbuff[5];
    return SI4463_OK;
}

int8_t si4463_getADCInfo(si4463_t* si4463)
{
    uint8_t cmd[3] = {GET_ADC_READING, 0x18, 0xC5};
    uint8_t rxbuff[6] = {0};
    if(!si4463_sendCommand(si4463, cmd, 3)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 6)) return SI4463_ERR_READ_REG;
    return SI4463_OK;
}

int16_t si4463_getTxFifoInfo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x00};
    uint8_t rxbuff[2] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    return rxbuff[1];
}

int16_t si4463_getRxFifoInfo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x00};
    uint8_t rxbuff[2] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    return rxbuff[0];
}

int16_t si4463_getPacketInfo(si4463_t* si4463)
{
    uint8_t cmd[6] = {GET_PACKET_INFO};
    uint8_t rxbuff[2] = {0};

    if(!si4463_sendCommand(si4463, cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    return rxbuff[1];
}

int8_t si4463_getCurrentRSSI(si4463_t* si4463)
{
	uint8_t cmd[2] = {GET_MODEM_STATUS, 0x00};
	uint8_t rxbuff[8] = {0};
	if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
	if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

	// si4468 RSSI calculation formula
    si4463->status.currentRSSI = (int16_t)((float)rxbuff[2]/2 - 134.0f);
	return SI4463_OK;
}

int8_t si4463_getLatchRSSI(si4463_t* si4463)
{
	uint8_t cmd[2] = {GET_MODEM_STATUS, 0x00};
	uint8_t rxbuff[8] = {0};
	if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
	if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

	// si4468 RSSI calculation formula
    si4463->status.latchRSSI = (int16_t)((float)rxbuff[3]/2 - 134.0f);
	return SI4463_OK;
}

int8_t si4463_getModemStatus(si4463_t* si4463)
{
    uint8_t cmd[2] = {GET_MODEM_STATUS, 0x00};
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

    // si4468 RSSI calculation formula
    si4463->status.modemPend = rxbuff[0];
    si4463->status.modemStatus = rxbuff[1];
    si4463->status.currentRSSI = (int16_t)((float)rxbuff[2]/2 - 134.0f);
    si4463->status.latchRSSI = (int16_t)((float)rxbuff[3]/2 - 134.0f);
    si4463->status.ant1RSSI = (int16_t)((float)rxbuff[4]/2 - 134.0f);
    si4463->status.ant2RSSI = (int16_t)((float)rxbuff[5]/2 - 134.0f);
    si4463->status.afcFreqOffset = (uint16_t)(rxbuff[6] << 8) | (uint16_t)(rxbuff[7]);
    return SI4463_OK;
}

int8_t si4463_clearTxFifo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x01};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_clearRxFifo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x02};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_resetInterrupts(si4463_t* si4463)
{
    uint8_t buff[4] = {0};
    buff[0] = 0x00;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = CHIP_READY_EN;
    return si4463_setProperties(si4463, buff, 4, PROP_INT_CTL_ENABLE);
}

int8_t si4463_clearInterrupts(si4463_t* si4463)
{
    uint8_t cmd[4] = {GET_INT_STATUS, 0x00, 0x00, 0x00};
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

    return SI4463_OK;
}

int8_t si4463_getInterrupts(si4463_t* si4463)
{
    uint8_t cmd[4] = {GET_INT_STATUS, 0xFF, 0xFF, 0xFF};
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

    /* Get pend bytes */
    uint8_t phPend = rxbuff[2];
    uint8_t modemPend = rxbuff[4];
    uint8_t chipPend = rxbuff[6];

    si4463->interrupts.filterMatch = (phPend >> 7) & 0x1;
    si4463->interrupts.filterMiss = (phPend >> 6) & 0x1;
    si4463->interrupts.packetSent = (phPend >> 5) & 0x1;
    si4463->interrupts.packetRx = (phPend >> 4) & 0x1;
    si4463->interrupts.crcError = (phPend >> 3) & 0x1;
    si4463->interrupts.txFifoAlmostEmpty = (phPend >> 1) & 0x1;
    si4463->interrupts.rxFifoAlmostFull = phPend & 0x1;

    si4463->interrupts.latchrssi = (modemPend >> 7) & 0x1;
    si4463->interrupts.postambleDetect = (modemPend >> 6) & 0x1;
    si4463->interrupts.invalidSync = (modemPend >> 5) & 0x1;
    si4463->interrupts.rssiJump = (modemPend >> 4) & 0x1;
    si4463->interrupts.rssi = (modemPend >> 3) & 0x1;
    si4463->interrupts.invalidPreamble = (modemPend >> 2) & 0x1;
    si4463->interrupts.preambleDetect = (modemPend >> 1) & 0x1;
    si4463->interrupts.packetSent = modemPend & 0x1;

    si4463->interrupts.cal = (chipPend >> 6) & 0x1;
    si4463->interrupts.fifoUnderflowOverflowError = (chipPend >> 5) & 0x1;
    si4463->interrupts.stateChange = (chipPend >> 4) & 0x1;
    si4463->interrupts.cmdError = (chipPend >> 3) & 0x1;
    si4463->interrupts.chipReady = (chipPend >> 2) & 0x1;
    si4463->interrupts.lowBatt = (chipPend >> 1) & 0x1;
    si4463->interrupts.wut = chipPend & 0x1;

    return SI4463_OK;
}

int8_t si4463_clearChipStatus(si4463_t* si4463)
{
    uint8_t cmd[2] = {GET_CHIP_STATUS, 0x00};
    uint8_t rxbuff[4] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 4)) return SI4463_ERR_READ_REG;

    return SI4463_OK;
}

int8_t si4463_transmit(si4463_t* si4463, uint8_t* txData, uint8_t txDataLen, si4463_state nextState)
{
    int16_t result = si4463_txInterrupt(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_clearInterrupts(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_clearTxFifo(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_getTxFifoInfo(si4463);
    if(result >= txDataLen)
    {
        result = si4463_writeTxFiFo(si4463, txData, txDataLen);
        if(result != SI4463_OK) return result;
        result = si4463_startTx(si4463, txDataLen + 1, nextState);
        if(result != SI4463_OK) return result;
        uint16_t counter = 0;
        // Check if IRQ pin is pulled down
        while(counter < SI4463_TRANSMIT_TIMEOUT)
        {
            if(si4463->gpios.IRQ() == si4463->gpios.gpio_low)
            {
                result = si4463_clearInterrupts(si4463);
                return result;
            }
            si4463->DelayUs(1000);
            counter++;
        }
    }
    else if(result >= 0)
        return SI4463_ERR_OVER_TX_FIFO;

    return result;
}

int8_t si4463_initRx(si4463_t* si4463, uint16_t dataLen, si4463_state nextStateAfterTimeOut, si4463_state nextStateAfterValid, si4463_state nextStateAfterInvalid)
{
    int result = si4463_clearRxFifo(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_rxInterrupt(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_clearInterrupts(si4463);
    if(result != SI4463_OK) return result;

    uint8_t cmd[8] = {START_RX, RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 0x00, ((dataLen >> 8) & 0x1F),  
            (dataLen & 0x00FF), nextStateAfterTimeOut, nextStateAfterValid, nextStateAfterInvalid};
    if(!si4463_sendCommand(si4463, cmd, 8)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_initDirectRx(si4463_t* si4463)
{
    int result = si4463_clearRxFifo(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_resetInterrupts(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_clearInterrupts(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_setDeviceState(si4463, STATE_RX);
    if(result != SI4463_OK) return result;

    result = si4463_configArray(si4463, SI4463_CONFIGURATION_DATA_DIRRX);
    if(result != SI4463_OK) return result;

//    uint8_t cmd[10] = {0};

    // Synchronous modem operation
//    cmd[0] = 0xC0;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_BCR_MISC1);
//    cmd[0] = 0x00;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_ONE_SHOT_AFC);
//    cmd[0] = 0x83;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_AFC_GAIN_1);

    // Asynchronous modem operation
//    cmd[0] = 0x0A;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_RAW_SEARCH2);
//    cmd[0] = 0x10;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_RAW_CONTROL);
//    cmd[0] = 0x03;
//    si4463_setProperties(si4463, cmd, 1, PROP_MODEM_OOK_MISC);


    return SI4463_OK;
    // Active GPIO2
//    return si4463_gpioPinCfg(si4463, GPIO_NO_CHANGE, GPIO_NO_CHANGE, GPIO_RX_DATA_CLK , GPIO_RX_DATA);
}

int8_t si4463_receive(si4463_t* si4463, uint8_t* rxData, uint8_t rxDataLen)
{
	int result = si4463_clearInterrupts(si4463);
	if(result != SI4463_OK) return result;
	if(!si4463_readRxDataBuff(si4463, rxData, rxDataLen)) return SI4463_ERR_READ_RXBUFF;
	return si4463_clearRxFifo(si4463);
}

int8_t si4463_setTxPower(si4463_t* si4463, uint8_t power)
{
    // Range: 0-127
    if(power > 127 || power < 5) return SI4463_ERR_BAD_PARAM;
    uint8_t cmd = power;

    return si4463_setProperties(si4463, &cmd, 1, PROP_PA_PWR_LVL);
}

int8_t si4463_getTxPower(si4463_t* si4463)
{
    uint16_t propNum = PROP_PA_PWR_LVL;
    uint8_t rxbuff = 0;
    int res = si4463_getProperties(si4463, &rxbuff, 1, propNum);
    if(res == SI4463_OK)
    {
        if(rxbuff >= 80)
            si4463->settings.power = PWR_20_dBm;
        else if(rxbuff >= 25)
            si4463->settings.power = PWR_14_dBm;
        else if(rxbuff >= 17)
            si4463->settings.power = PWR_10_dBm;
        else if(rxbuff >= 9)
            si4463->settings.power = PWR_4_dBm;
        else if(rxbuff >= 5)
            si4463->settings.power = PWR_0_dBm;
        return rxbuff;
    }
    else
        return res;
}

int8_t si4463_setPreamble(si4463_t* si4463, uint8_t preambleLen)
{
    if(preambleLen < 5) return SI4463_ERR_BAD_PARAM;
    uint8_t cmd = preambleLen;

    return si4463_setProperties(si4463, &cmd, 1, PROP_PREAMBLE_TX_LENGTH);
}

int16_t si4463_getPreamble(si4463_t* si4463)
{
    uint16_t propNum = PROP_PREAMBLE_TX_LENGTH;
    uint8_t rxbuff = 0;
    int res = si4463_getProperties(si4463, &rxbuff, 1, propNum);
    if(res == SI4463_OK)
    {
        si4463->settings.preambleNum = rxbuff;
        return si4463->settings.preambleNum;
    }
    else
        return res;
}

int8_t si4463_setSyncWords(si4463_t* si4463, uint8_t* syncdata, uint8_t syncLen)
{
    if(syncLen < 1 || syncLen > 4) return SI4463_ERR_BAD_PARAM;
    uint8_t* cmd = (uint8_t*)malloc((1 + syncLen)*sizeof(uint8_t));
    cmd[0] = syncLen-1;
    memcpy(&cmd[1], syncdata, syncLen);

    int res = si4463_setProperties(si4463, cmd, 1 + syncLen, PROP_SYNC_CONFIG);

    free(cmd);
    return res;
}

int8_t si4463_getSyncWords(si4463_t* si4463)
{
    uint16_t propNum = PROP_SYNC_CONFIG;
    uint8_t rxbuff[5] = {0};
    int res = si4463_getProperties(si4463, rxbuff, 5, propNum);
    if(res == SI4463_OK)
    {
        uint8_t syncLen = (0x03 & rxbuff[0]) + 1;
        for(int i = 0; i < syncLen; i++)
            si4463->settings.syncWords[i] = rxbuff[i+1];
        
        return syncLen;
    }
    else
        return res;
}

int8_t si4463_setCRC(si4463_t* si4463, bool crcSeed, si4463_crc_poly crcPoly)
{
	if((0x0F & crcPoly) > 8) return 0;
	uint8_t cmd = (0x80 & (crcSeed << 3))  | (0x0F & crcPoly);
 
    return si4463_setProperties(si4463, &cmd, 1, PROP_PKT_CRC_CONFIG);
}

int8_t si4463_getCRC(si4463_t* si4463)
{
    uint16_t propNum = PROP_PKT_CRC_CONFIG;
    uint8_t rxbuff = 0;
    int res = si4463_getProperties(si4463, &rxbuff, 1, propNum);
    if(res == SI4463_OK)
    {
        si4463->settings.crcSeed = (0x80 & rxbuff) >> 7;
        si4463->settings.crcPoly = (si4463_crc_poly)(0x0F & rxbuff);
        return SI4463_OK;
    }
    else
        return res;
}

int8_t si4463_getFreqConfig(si4463_t* si4463)
{
    uint16_t propNum = PROP_FREQ_CONTROL_INTE;
    uint8_t rxbuff[4] = {0};

    int res = si4463_getProperties(si4463, rxbuff, 4, propNum);
    if(res == SI4463_OK)
    {
        si4463->freq.freq_inte = rxbuff[0];
        si4463->freq.freq_frac = ((0x0F & (uint32_t)rxbuff[1]) << 16) | ((uint32_t)rxbuff[2] << 8) | (uint32_t)rxbuff[3];
    }
    propNum = PROP_MODEM_CLKGEN_BAND;
    memset(rxbuff, '\0', 4);
    res = si4463_getProperties(si4463, rxbuff, 1, propNum);
    if(res == SI4463_OK)
    {
        // ENUM_1 or ENUM 0
        if(0x08 & rxbuff[0])
            si4463->freq.n_presc = 2;
        else
            si4463->freq.n_presc = 4;
        switch(0x07 & rxbuff[0])
        {
            case FVCO_DIV_4:
                si4463->freq.outdiv = 4;
                break;
            case FVCO_DIV_6:
                si4463->freq.outdiv = 6;
                break;
            case FVCO_DIV_8:
                si4463->freq.outdiv = 8;
                break;
            case FVCO_DIV_12:
                si4463->freq.outdiv = 12;
                break;
            case FVCO_DIV_16:
                si4463->freq.outdiv = 16;
                break;
            case FVCO_DIV_24:
                si4463->freq.outdiv = 24;
                break;
            case FVCO_DIV_24_2:
                si4463->freq.outdiv = 24;
                break;
            case FVCO_DIV_24_3:
                si4463->freq.outdiv = 24;
                break;
        }
    }

    return res;
}

int8_t si4463_setFrequency(si4463_t* si4463, uint32_t freq)
{
    int res = si4463_getFreqConfig(si4463);
    if(res == SI4463_OK)
    {
        uint8_t cmd[3] = {0};
        float temp = (float)freq / ((si4463->freq.n_presc * (float)RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ)/ si4463->freq.outdiv);
        uint32_t fc_frac = (temp - si4463->freq.freq_inte)*pow(2, 19);
        cmd[0] = 0x0F & (fc_frac >> 16);
        cmd[1] = 0xFF & fc_frac >> 8;
        cmd[2] = 0xFF & fc_frac;
        return si4463_setProperties(si4463, cmd, 3, PROP_FREQ_CONTROL_FRAC);
    }
    else
        return res;
}

int32_t si4463_getFrequency(si4463_t* si4463)
{
    int res = si4463_getFreqConfig(si4463);
    if(res == SI4463_OK)
    {
        float fc = ((float)si4463->freq.freq_inte + (float)si4463->freq.freq_frac/524288.0);
        float np = (si4463->freq.n_presc*(float)RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ/si4463->freq.outdiv);
        int32_t rounded_freq = (int32_t)(fc * np * 0.001 + 0.5)*1000;
        si4463->settings.frequency = rounded_freq;
        return SI4463_OK;
    }
    else
        return res;
}

int8_t si4463_setTxModulation(si4463_t* si4463, si4463_mod_type mod)
{
    int8_t res = SI4463_OK;
    // OOK for transmission/beacons
    if(mod == MOD_OOK)
    {
        res = si4463_configArray(si4463, modem_table[mod][DR_1200].tx);
        si4463->settings.txMod = mod;
        return res;
    }

    // Check whether the variable "mod" is reasonable.
    if(mod > MOD_GMSK || mod < MOD_2FSK) return SI4463_ERR_INVALID_MOD;
    // FSK, 2GFSK and GMSK for telemetry/echo
    res = si4463_configArray(si4463, modem_table[mod][si4463->settings.txDataRate].tx);

    if(res == SI4463_OK)
        si4463->settings.txMod = mod;
    return res;
}

int8_t si4463_setRxModulation(si4463_t* si4463, si4463_mod_type mod)
{
    // Check whether the variable "mod" is reasonable.
    if(mod > MOD_GMSK || mod < MOD_2FSK) return SI4463_ERR_INVALID_MOD;

    int8_t res = si4463_configArray(si4463, modem_table[mod][si4463->settings.rxDataRate].rx);

    if(res == SI4463_OK)
        si4463->settings.rxMod = mod;
    return res;
}

int8_t si4463_getModulation(si4463_t* si4463)
{
    const uint16_t propNum = PROP_MODEM_MOD_TYPE;
    uint8_t rxbuff = 0;
    int res = si4463_getProperties(si4463, &rxbuff, 1, propNum);
    if(res == SI4463_OK)
    {
        uint8_t modType = 0x07 & rxbuff;
        return modType;
    }
    else
        return res;
}

int8_t si4463_setTxDataRate(si4463_t* si4463, si4463_data_rate datarate)
{
    // Check whether the variable "datarate" is reasonable.
    if(datarate > DR_9600) return SI4463_ERR_INVALID_DR;

    int8_t res = si4463_configArray(si4463, modem_table[si4463->settings.txMod][datarate].tx);

    if(res == SI4463_OK)
        si4463->settings.txDataRate = datarate;
    return res;
}

int8_t si4463_setRxDataRate(si4463_t* si4463, si4463_data_rate datarate)
{
    // Check whether the variable "datarate" is reasonable.
    if(datarate > DR_9600) return SI4463_ERR_INVALID_DR;

    int8_t res = si4463_configArray(si4463, modem_table[si4463->settings.rxMod][datarate].rx);

    if(res == SI4463_OK)
        si4463->settings.rxDataRate = datarate;
    return res;
}

int16_t si4463_getDataRate(si4463_t* si4463)
{
    int res = si4463_getDataRateConfig(si4463);
    if(res == SI4463_OK)
    {
        uint8_t TXOSR = 0;
        switch(si4463->dr.TxOSR)
        {
        case OVERSAMPLING_RATIO_10:
            TXOSR = 10;
            break;
        case OVERSAMPLING_RATIO_40:
            TXOSR = 40;
            break;
        case OVERSAMPLING_RATIO_20:
            TXOSR = 20;
            break;
        default:
            return SI4463_ERR_INVALID_TXOSR;
        }
        float datarate = (si4463->dr.modemDataRate * (float)RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ)/(float)(si4463->dr.modemTxNCOMode)/TXOSR;
        return (int16_t)datarate;
    }
    else
        return res;
}

int8_t si4463_getDeviceState(si4463_t* si4463)
{
    uint8_t cmd = REQUEST_DEVICE_STATE;
    uint8_t rxbuff[2] = {0};

    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    si4463->state.currState = (si4463_state)rxbuff[0];
    si4463->state.currChannel = rxbuff[1];
    return SI4463_OK;
}

int8_t si4463_setDeviceState(si4463_t* si4463, si4463_state state)
{
    uint8_t cmd[2] = {0};
    cmd[0] = CHANGE_STATE;
    cmd[1] = (uint8_t)state;

    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    si4463->state.currState = state;
    return SI4463_OK;
}

int8_t si4463_getDataRateConfig(si4463_t* si4463)
{
    uint16_t propNum = PROP_MODEM_DATA_RATE;
    uint8_t rxbuff[7] = {0};
    int res = si4463_getProperties(si4463, rxbuff, 7, propNum);
    if(res == SI4463_OK)
    {
        si4463->dr.modemDataRate = ((uint32_t)rxbuff[0] << 16) | ((uint32_t)rxbuff[1] << 8) | (uint32_t)rxbuff[2];
        si4463->dr.modemTxNCOMode = ((uint32_t)(rxbuff[3] & 0x03) << 24) | ((uint32_t)rxbuff[4] << 16) | ((uint32_t)rxbuff[5] << 8) | (uint32_t)rxbuff[6];
        si4463->dr.TxOSR = (si4463_txosr)((rxbuff[3] & 0x0C) >> 2);
        return res;
    }
    else
        return res;
}

int8_t si4463_sendCommand(si4463_t* si4463, uint8_t* cmdData, uint8_t cmdLen)
{
    si4463->NSEL(false);
    si4463->SPI_Write(cmdData, cmdLen, SI4463_SPI_TIMEOUT);
    si4463->NSEL(true);
    if(si4463->SPI_CheckState() != si4463->spi_state_ready)
        return 0;
    else
        return 1;
}

int8_t si4463_waitforCTS(si4463_t* si4463)
{
    uint8_t output_address[2] = {0};
    uint8_t ctsValue[2] = {0};
    uint16_t errCnt = 0;

    while(ctsValue[1] != 0xFF)
    {
        if(++errCnt > MAX_CTS_RETRY)
            return 0;
        output_address[0] = READ_CMD_BUFFER;
        si4463->NSEL(false);
        si4463->SPI_WriteRead(&output_address[0], &ctsValue[0], 2, SI4463_SPI_TIMEOUT);
        si4463->NSEL(true);
    }
    return 1;
}

int8_t si4463_getResponse(si4463_t* si4463, uint8_t* respData, uint8_t respLen)
{
	uint8_t output_address[2] = {0};
	uint8_t ctsValue[2] = {0};
	uint16_t errCnt = 0;

	while(ctsValue[1] != 0xFF)
	{
		if(++errCnt > MAX_CTS_RETRY)
			return 0;
		output_address[0] = READ_CMD_BUFFER;
		si4463->NSEL(false);
		si4463->SPI_WriteRead(&output_address[0], &ctsValue[0], 2, SI4463_SPI_TIMEOUT);
		if(ctsValue[1] == 0xFF)
			si4463->SPI_Read(respData, respLen, SI4463_SPI_TIMEOUT);
		si4463->NSEL(true);
	}
    if(si4463->SPI_CheckState() != si4463->spi_state_ready)
        return 0;
    else
        return 1;
}

int8_t si4463_writeTxFiFo(si4463_t* si4463, uint8_t* txFifoData, uint8_t txFifoLen)
{
    int res = SI4463_OK;
//    uint8_t* cmd = (uint8_t*)malloc((txFifoLen + 2)*sizeof(uint8_t));
    memset(gcmd, '\0', SI4463_MAX_FIFO_SIZE + 1);
//    memset(cmd, '\0', 100);
    gcmd[0] = WRITE_TX_FIFO;
    gcmd[1] = txFifoLen;
    memcpy(&gcmd[2], txFifoData, txFifoLen);
    if(!si4463_sendCommand(si4463, gcmd, txFifoLen + 2)) res = SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) res = SI4463_CTS_TIMEOUT;

//    free(cmd);
    return res;
}

int8_t si4463_startTx(si4463_t* si4463, uint16_t dataLen, si4463_state nextState)
{
    uint8_t cmd[5] = {START_TX, RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
            (nextState << 4), ((dataLen >> 8) & 0x001F), (dataLen & 0x00FF)};
    if(!si4463_sendCommand(si4463, cmd, sizeof(cmd))) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_readRxDataBuff(si4463_t* si4463, uint8_t* rxFifoData, uint8_t rxFifoLength)
{
	uint8_t cmd = READ_RX_FIFO;

	si4463->NSEL(false);
	si4463->SPI_Write(&cmd, 1, SI4463_SPI_TIMEOUT);
	si4463->SPI_Read(rxFifoData, rxFifoLength, SI4463_SPI_TIMEOUT);
	si4463->NSEL(true);

	if(si4463->SPI_CheckState() != si4463->spi_state_ready)
		return 0;
	else
		return 1;
}

int8_t si4463_configArray(si4463_t* si4463, uint8_t* configArray)
{
    uint16_t index = 0, currentNum = 0;
    while(configArray[index])
    {
        currentNum = configArray[index];
        if(!si4463_sendCommand(si4463, &configArray[index + 1], currentNum))
            return SI4463_ERR_WRITE_REG;
        if(!si4463_waitforCTS(si4463))
            return SI4463_CTS_TIMEOUT;

        if(configArray[index + 1] == 0x11)
            DEBUG_PRINTF("Property number: 0x%04x\r\n", (configArray[index + 2] << 8) | configArray[index + 4]);
		else
            DEBUG_PRINTF("Command number: 0x%02x\r\n", configArray[index + 1]);
		index = index + currentNum + 1;
    }
    return SI4463_OK;
}

int8_t si4463_setProperties(si4463_t* si4463, uint8_t* setData, uint8_t setDataLen, uint16_t propNum)
{
    int8_t res = SI4463_OK;
//    uint8_t* cmd = (uint8_t*)malloc((setDataLen + 4)*sizeof(uint8_t));

    memset(gcmd, '\0', SI4463_MAX_FIFO_SIZE + 1);
    gcmd[0] = SET_PROPERTY;
    gcmd[1] = (uint8_t)(propNum >> 8);
    gcmd[2] = setDataLen;
    gcmd[3] = (uint8_t)(propNum & 0x00FF);
    memcpy(&gcmd[4], setData, setDataLen);
    if(!si4463_sendCommand(si4463, gcmd, setDataLen + 4)) res = SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) res = SI4463_CTS_TIMEOUT;

//    free(cmd);
    return res;
}

int8_t si4463_getProperties(si4463_t* si4463, uint8_t* getData, uint8_t getDataLen, uint16_t propNum)
{
    uint8_t cmd[4] = {GET_PROPERTY, (uint8_t)(propNum >> 8), getDataLen, (uint8_t)(propNum & 0x00FF)};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, getData, getDataLen)) return SI4463_ERR_READ_REG;

    DEBUG_PRINTF("--------------------------\r\n");
		DEBUG_PRINTF("Get property data: ");
		for(int i = 0; i < getDataLen;i++)
		{
			DEBUG_PRINTF("0x%02x ", getData[i]);
		}
    DEBUG_PRINTF("\r\n");

    return SI4463_OK;
}

int8_t si4463_txInterrupt(si4463_t* si4463)
{
	uint8_t buff[4] = {0};
	buff[0] = PH_INT_STATUS_EN;
	buff[1] = PACKET_SENT_EN;
	buff[2] = 0x00;
	buff[3] = CHIP_READY_EN;
	return si4463_setProperties(si4463, buff, 4, PROP_INT_CTL_ENABLE);
}

int8_t si4463_rxInterrupt(si4463_t* si4463)
{
	uint8_t buff[4] = {0};
	buff[0] = MODEM_INT_STATUS_EN | PH_INT_STATUS_EN;
	buff[1] = PACKET_RX_EN | CRC_ERROR_EN;
	buff[2] = 0x00;
	buff[3] = CHIP_READY_EN;
	return si4463_setProperties(si4463, buff, 4, PROP_INT_CTL_ENABLE);
}

void si4463_controlOOK(si4463_t* si4463, bool toneOn)
{
    if(toneOn)
        si4463->OOK(si4463->gpios.gpio_high);
    else
        si4463->OOK(si4463->gpios.gpio_low);
}

int8_t si4463_setPARamp(si4463_t* si4463, uint8_t tc)
{
    if(tc > 15) return SI4463_ERR_BAD_PARAM;
    return si4463_setProperties(si4463, &tc, 1, PROP_PA_RAMP_EX);
}

