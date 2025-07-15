# TT-C_API
The TT-C API is a transceiver interface driver compatible with the Si446x series, designed for reliable communication in challenging environments. It supports both the Si4463 and Si4468 transceivers and offers built-in support for the AX.25 protocol and Morse code. The API provides a wide range of functions to help developers operate the Si446x series transceivers and configure their settings efficiently. Additionally, Si446x series feature automatic frequency control (AFC) to mitigate the effects of Doppler shift.

## Features

### Modulation
- GFSK, FSK, GMSK, MSK
- OOK

### Data rate
- From 1200 bps ~ 9600 bps

### Packet Length
- TX and RX 64 byte FIFOs
- 129 bytes dedicated Tx or Rx FIFO

## Serial Peripheral Interface (SPI)

The Si446x communicates with the host MCU over a standard 4-wire serial peripheral interface (SPI): SCLK, SDI,
SDO, and nSEL. The SPI interface is designed to operate at a maximum of 10 MHz.