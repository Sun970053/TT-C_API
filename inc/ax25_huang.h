/*
 * ax25_huang.h
 *
 *  Created on: June 16, 2024
 *      Author: Ting-Shan, Huang
 */

#ifndef INC_AX25_HUANG_H_
#define INC_AX25_HUANG_H_

#include <stdint.h>
#include <stdbool.h>

// macros to access bits in byte array, from http://www.mathcs.emory.edu/~cheung/Courses/255/Syllabus/1-C-intro/bit-array.html
#define SET_BIT_IN_ARRAY(A, k)                                  ( A[(k/8)] |= (1 << (k%8)) )
#define CLEAR_BIT_IN_ARRAY(A, k)                                ( A[(k/8)] &= ~(1 << (k%8)) )
#define TEST_BIT_IN_ARRAY(A, k)                                 ( A[(k/8)] & (1 << (k%8)) )
#define GET_BIT_IN_ARRAY(A, k)                                  ( (A[(k/8)] & (1 << (k%8))) ? 1 : 0 )

// maximum callsign length in bytes
#define RADIOLIB_AX25_MAX_CALLSIGN_LEN                          6

// flag field                                                                 MSB   LSB   DESCRIPTION
#define RADIOLIB_AX25_FLAG                                      0b01111110  //  7     0     AX.25 frame start/end flag

// address field
#define RADIOLIB_AX25_SSID_COMMAND_DEST                         0b10000000  //  7     7     frame type: command (set in destination SSID)
#define RADIOLIB_AX25_SSID_COMMAND_SOURCE                       0b00000000  //  7     7                 command (set in source SSID)
#define RADIOLIB_AX25_SSID_RESPONSE_DEST                        0b00000000  //  7     7                 response (set in destination SSID)
#define RADIOLIB_AX25_SSID_RESPONSE_SOURCE                      0b10000000  //  7     7                 response (set in source SSID)
#define RADIOLIB_AX25_SSID_HAS_NOT_BEEN_REPEATED                0b00000000  //  7     7                 not repeated yet (set in repeater SSID)
#define RADIOLIB_AX25_SSID_HAS_BEEN_REPEATED                    0b10000000  //  7     7                 repeated (set in repeater SSID)
#define RADIOLIB_AX25_SSID_RESERVED_BITS                        0b01100000  //  6     5     reserved bits in SSID
#define RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE              0b00000000  //  0     0     HDLC extension bit: next octet contains more address information
#define RADIOLIB_AX25_SSID_HDLC_EXTENSION_END                   0b00000001  //  0     0                         address field end

// control field
#define RADIOLIB_AX25_CONTROL_U_SET_ASYNC_BAL_MODE              0b01101100  //  7     2     U frame type: set asynchronous balanced mode (connect request)
#define RADIOLIB_AX25_CONTROL_U_SET_ASYNC_BAL_MODE_EXT          0b00101100  //  7     2                   set asynchronous balanced mode extended (connect request with module 128)
#define RADIOLIB_AX25_CONTROL_U_DISCONNECT                      0b01000000  //  7     2                   disconnect request
#define RADIOLIB_AX25_CONTROL_U_DISCONNECT_MODE                 0b00001100  //  7     2                   disconnect mode (system busy or disconnected)
#define RADIOLIB_AX25_CONTROL_U_UNNUMBERED_ACK                  0b01100000  //  7     2                   unnumbered acknowledge
#define RADIOLIB_AX25_CONTROL_U_FRAME_REJECT                    0b10000100  //  7     2                   frame reject
#define RADIOLIB_AX25_CONTROL_U_UNNUMBERED_INFORMATION          0b00000000  //  7     2                   unnumbered information
#define RADIOLIB_AX25_CONTROL_U_EXHANGE_IDENTIFICATION          0b10101100  //  7     2                   exchange ID
#define RADIOLIB_AX25_CONTROL_U_TEST                            0b11100000  //  7     2                   test
#define RADIOLIB_AX25_CONTROL_POLL_FINAL_ENABLED                0b00010000  //  4     4     control field poll/final bit: enabled
#define RADIOLIB_AX25_CONTROL_POLL_FINAL_DISABLED               0b00000000  //  4     4                                   disabled
#define RADIOLIB_AX25_CONTROL_S_RECEIVE_READY                   0b00000000  //  3     2     S frame type: receive ready (system ready to receive)
#define RADIOLIB_AX25_CONTROL_S_RECEIVE_NOT_READY               0b00000100  //  3     2                   receive not ready (TNC buffer full)
#define RADIOLIB_AX25_CONTROL_S_REJECT                          0b00001000  //  3     2                   reject (out of sequence or duplicate)
#define RADIOLIB_AX25_CONTROL_S_SELECTIVE_REJECT                0b00001100  //  3     2                   selective reject (single frame repeat request)
#define RADIOLIB_AX25_CONTROL_INFORMATION_FRAME                 0b00000000  //  0     0     frame type: information (I frame)
#define RADIOLIB_AX25_CONTROL_SUPERVISORY_FRAME                 0b00000001  //  1     0                 supervisory (S frame)
#define RADIOLIB_AX25_CONTROL_UNNUMBERED_FRAME                  0b00000011  //  1     0                 unnumbered (U frame)

// protocol identifier field
#define RADIOLIB_AX25_PID_ISO_8208                              0x01
#define RADIOLIB_AX25_PID_TCP_IP_COMPRESSED                     0x06
#define RADIOLIB_AX25_PID_TCP_IP_UNCOMPRESSED                   0x07
#define RADIOLIB_AX25_PID_SEGMENTATION_FRAGMENT                 0x08
#define RADIOLIB_AX25_PID_TEXNET_DATAGRAM_PROTOCOL              0xC3
#define RADIOLIB_AX25_PID_LINK_QUALITY_PROTOCOL                 0xC4
#define RADIOLIB_AX25_PID_APPLETALK                             0xCA
#define RADIOLIB_AX25_PID_APPLETALK_ARP                         0xCB
#define RADIOLIB_AX25_PID_ARPA_INTERNET_PROTOCOL                0xCC
#define RADIOLIB_AX25_PID_ARPA_ADDRESS_RESOLUTION               0xCD
#define RADIOLIB_AX25_PID_FLEXNET                               0xCE
#define RADIOLIB_AX25_PID_NET_ROM                               0xCF
#define RADIOLIB_AX25_PID_NO_LAYER_3                            0xF0
#define RADIOLIB_AX25_PID_ESCAPE_CHARACTER                      0xFF

/*!
  \brief No error, method executed successfully.
*/
#define RADIOLIB_ERR_NONE                                       (0)

/*!
  \brief Timed out waiting for transmission finish.
*/
#define RADIOLIB_ERR_TX_TIMEOUT                                 (-5)

/*!
  \brief The provided callsign is invalid.

  The specified callsign is longer than 6 ASCII characters.
*/
#define RADIOLIB_ERR_INVALID_CALLSIGN                           (-801)

/*!
  \brief The provided repeater configuration is invalid.

  The specified number of repeaters does not match number of repeater IDs or their callsigns.
*/
#define RADIOLIB_ERR_INVALID_NUM_REPEATERS                      (-802)

/*!
  \brief One of the provided repeater callsigns is invalid.

  The specified callsign is longer than 6 ASCII characters.
*/
#define RADIOLIB_ERR_INVALID_REPEATER_CALLSIGN                  (-803)

/*!
  \brief The receiving CRC checksum is wrong.

  The length of CRC is 2 bytes.
*/
#define RADIOLIB_ERR_RX_CRC_CHECKSUM                            (-808)

// CCITT CRC properties (used by AX.25)
#define RADIOLIB_CRC_CCITT_POLY                                 (0x1021)
#define RADIOLIB_CRC_CCITT_INIT                                 (0xFFFF)
#define RADIOLIB_CRC_CCITT_OUT                                  (0xFFFF)

typedef struct
{
    uint8_t size;
    uint32_t poly;
    uint32_t init;
    uint32_t out;
    bool refIn;
    bool refOut;
}crc_t;

typedef struct
{
  char destCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];
  uint8_t destSSID;
  char srcCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];
  uint8_t srcSSID;
  uint8_t control;
  uint8_t protocolID;
  uint16_t infoLen;
  uint8_t rcvSeqNumber;
  uint16_t sendSeqNumber;
  uint8_t* info;
  uint16_t preambleLen;
}ax25sendframe_t;

typedef struct
{
  char destCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];
  uint8_t destSSID;
  char srcCallsign[RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1];
  uint8_t srcSSID;
  uint8_t control;
  uint8_t protocolID;
  uint16_t infoLen;
  uint8_t rcvSeqNumber;
  uint16_t sendSeqNumber;
  uint8_t* info;
  uint16_t preambleLen;
  bool isCrcOk;
}ax25receiveframe_t;

typedef struct
{
  ax25sendframe_t* ax25SendFrame;
  ax25receiveframe_t* ax25RcvFrame;
  crc_t crc;
}ax25frame_t;

ax25sendframe_t* createAX25SendFrame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID, uint8_t* info, uint16_t infoLen, uint16_t preLen);
ax25receiveframe_t* createAX25ReceiveFrame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID,  uint16_t preLen);
void deleteAX25SendFrame(ax25frame_t* ax25frame);
void deleteAX25ReceiveFrame(ax25frame_t* ax25frame);
void initCRC(ax25frame_t* ax25frame);
uint16_t AX25Frame_HDLC_Generator(ax25frame_t* ax25frame, uint8_t** pStuffedFrame, uint16_t* stuffedFrameLen);
uint16_t AX25Frame_HDLC_Parser(ax25frame_t* ax25frame , uint8_t* stuffedFrame, uint16_t stuffedFrameLen);
void ax25_nrzi_encode(uint8_t* input, uint8_t* output, uint16_t len);
void ax25_nrzi_decode(uint8_t* input, uint8_t* output, uint16_t len);
void ax25_g3ruh_scrambler_init(uint32_t tap_mask);
void ax25_g3ruh_scrambler(uint8_t* unscrambled, uint8_t* scrambled, uint16_t len);
void ax25_g3ruh_descrambler(uint8_t* scrambled, uint8_t* unscrambled, uint16_t len);

#endif
