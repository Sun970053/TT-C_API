/*
 * ax25_huang.c
 *
 *  Created on: June 16, 2024
 *      Author: Ting-Shan, Huang
 */

#include "ax25_huang.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

uint32_t reflect(uint32_t in, uint8_t bits);
uint32_t checksum(ax25frame_t* ax25frame, uint8_t* buff, size_t len);

uint32_t d_shift_register = 0;
uint32_t d_taps[32];
uint32_t d_tap_count;


ax25sendframe_t* createAX25SendFrame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID, uint8_t* info, uint16_t infoLen, uint16_t preLen)
{
  ax25sendframe_t* ax25frame = (ax25sendframe_t*)malloc(sizeof(ax25sendframe_t));

  // destination callsign/SSID
  memcpy(ax25frame->destCallsign, destCallsign, strlen(destCallsign));
  ax25frame->destCallsign[strlen(destCallsign)] = '\0';
  ax25frame->destSSID = destSSID;

  // source callsign/SSID
  memcpy(ax25frame->srcCallsign, srcCallsign, strlen(srcCallsign));
  ax25frame->srcCallsign[strlen(srcCallsign)] = '\0';
  ax25frame->srcSSID = srcSSID;

  // control field
  ax25frame->control = control;

  // sequence numbers
  ax25frame->rcvSeqNumber = 0;
  ax25frame->sendSeqNumber = 0;

  // PID field
  ax25frame->protocolID = protocolID;

  // info field
  ax25frame->infoLen = infoLen;
  if(infoLen > 0) 
  {
      ax25frame->info = (uint8_t*)malloc(infoLen*sizeof(uint8_t));
      memcpy(ax25frame->info, info, infoLen);
  }

  // save preamble length
  ax25frame->preambleLen = preLen;

  return ax25frame;
}

ax25receiveframe_t* createAX25ReceiveFrame(const char* destCallsign, uint8_t destSSID, const char* srcCallsign, uint8_t srcSSID, uint8_t control, uint8_t protocolID,  uint16_t preLen)
{
  ax25receiveframe_t* ax25frame = (ax25receiveframe_t*)malloc(sizeof(ax25receiveframe_t));

  // destination callsign/SSID
  memcpy(ax25frame->destCallsign, destCallsign, strlen(destCallsign));
  ax25frame->destCallsign[strlen(destCallsign)] = '\0';
  ax25frame->destSSID = destSSID;

  // source callsign/SSID
  memcpy(ax25frame->srcCallsign, srcCallsign, strlen(srcCallsign));
  ax25frame->srcCallsign[strlen(srcCallsign)] = '\0';
  ax25frame->srcSSID = srcSSID;

  // control field
  ax25frame->control = control;

  // sequence numbers
  ax25frame->rcvSeqNumber = 0;
  ax25frame->sendSeqNumber = 0;

  // PID field
  ax25frame->protocolID = protocolID;

  // save preamble length
  ax25frame->preambleLen = preLen;

  return ax25frame;
}

void deleteAX25SendFrame(ax25frame_t* ax25frame)
{
  // deallocate info field
  if(ax25frame->ax25SendFrame->infoLen > 0)
  {
      free(ax25frame->ax25SendFrame->info);
      ax25frame->ax25SendFrame->info = NULL;
  }
  free(ax25frame->ax25SendFrame);
  ax25frame->ax25SendFrame = NULL;
}

void deleteAX25ReceiveFrame(ax25frame_t* ax25frame)
{
  // deallocate info field
  if(ax25frame->ax25RcvFrame->infoLen > 0)
  {
      free(ax25frame->ax25SendFrame->info);
      ax25frame->ax25SendFrame->info = NULL;
  }
  free(ax25frame->ax25SendFrame);
  ax25frame->ax25SendFrame = NULL;
}

void initCRC(ax25frame_t* ax25frame)
{
  ax25frame->crc.size = 16;
  ax25frame->crc.poly = RADIOLIB_CRC_CCITT_POLY;
  ax25frame->crc.init = RADIOLIB_CRC_CCITT_INIT;
  ax25frame->crc.out = RADIOLIB_CRC_CCITT_OUT;
  ax25frame->crc.refIn = false;
  ax25frame->crc.refOut = false;
}

uint16_t AX25Frame_HDLC_Generator(ax25frame_t* ax25frame, uint8_t** pStuffedFrame, uint16_t* stuffedFrameLen)
{
  // check destination callsign length (6 characters max)
  if(strlen(ax25frame->ax25SendFrame->destCallsign) > RADIOLIB_AX25_MAX_CALLSIGN_LEN) {
      return(RADIOLIB_ERR_INVALID_CALLSIGN);
  }

  // calculate frame length without FCS (destination address, source address, repeater addresses, control, PID, info)
  size_t frameBuffLen = (2*(RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1)) + 1 + 1 + ax25frame->ax25SendFrame->infoLen;
  // create frame buffer without preamble, start or stop flags
  uint8_t* frameBuff = (uint8_t*)malloc((frameBuffLen + 2)*sizeof(uint8_t));
  uint8_t* frameBuffPtr = frameBuff;

  // set destination callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
  for(size_t i = 0; i < strlen(ax25frame->ax25SendFrame->destCallsign); i++)
  {
      *(frameBuffPtr + i) = ax25frame->ax25SendFrame->destCallsign[i] << 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // set destination SSID
  *(frameBuffPtr++) = RADIOLIB_AX25_SSID_RESPONSE_DEST | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->ax25SendFrame->destSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

  // set source callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  memset(frameBuffPtr, ' ' << 1, RADIOLIB_AX25_MAX_CALLSIGN_LEN);
  for(size_t i = 0; i < strlen(ax25frame->ax25SendFrame->srcCallsign); i++)
  {
      *(frameBuffPtr + i) = ax25frame->ax25SendFrame->srcCallsign[i] << 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // set source SSID
  *(frameBuffPtr++) = RADIOLIB_AX25_SSID_COMMAND_SOURCE | RADIOLIB_AX25_SSID_RESERVED_BITS | (ax25frame->ax25SendFrame->srcSSID & 0x0F) << 1 | RADIOLIB_AX25_SSID_HDLC_EXTENSION_CONTINUE;

  // set HDLC extension end bit
  *(frameBuffPtr - 1) |= RADIOLIB_AX25_SSID_HDLC_EXTENSION_END;

  // set sequence numbers of the frames that have it
  uint8_t controlField = ax25frame->ax25SendFrame->control;
  if((ax25frame->ax25SendFrame->control & 0x01) == 0) {
      // information frame, set both sequence numbers
      controlField |= ax25frame->ax25SendFrame->rcvSeqNumber << 5;
      controlField |= ax25frame->ax25SendFrame->sendSeqNumber << 1;
  } else if((ax25frame->ax25SendFrame->control & 0x02) == 0) {
      // supervisory frame, set only receive sequence number
      controlField |= ax25frame->ax25SendFrame->rcvSeqNumber << 5;
  }

  // set control field
  *(frameBuffPtr++) = controlField;

  // set PID field of the frames that have it
  if(ax25frame->ax25SendFrame->protocolID != 0x00) {
      *(frameBuffPtr++) = ax25frame->ax25SendFrame->protocolID;
  }

  // set info field of the frames that have it
  if(ax25frame->ax25SendFrame->infoLen > 0) {
      memcpy(frameBuffPtr, ax25frame->ax25SendFrame->info, ax25frame->ax25SendFrame->infoLen);
      frameBuffPtr += ax25frame->ax25SendFrame->infoLen;
  }

  // flip bit order
  for(size_t i = 0; i < frameBuffLen; i++) 
  {
      frameBuff[i] = reflect(frameBuff[i], 8);
  }

  ax25frame->crc.size = 16;
  ax25frame->crc.poly = RADIOLIB_CRC_CCITT_POLY;
  ax25frame->crc.init = RADIOLIB_CRC_CCITT_INIT;
  ax25frame->crc.out = RADIOLIB_CRC_CCITT_OUT;
  ax25frame->crc.refIn = false;
  ax25frame->crc.refOut = false;

  uint16_t fcs = checksum(ax25frame, frameBuff, frameBuffLen);
  *(frameBuffPtr++) = (uint8_t)((fcs >> 8) & 0xFF);
  *(frameBuffPtr++) = (uint8_t)(fcs & 0xFF);

  printf("AX.25 frame\r\n");
	printf("packet: ");
	for(int i = 0; i <  frameBuffLen + 2; i++)
		printf("0x%02x ", frameBuff[i]);
	printf("\r\n");

  // prepare buffer for the final frame (stuffed, with added preamble + flags and NRZI-encoded)
  // worst-case scenario: sequence of 1s, will have 120% of the original length, stuffed frame also includes both flags
  uint8_t* stuffedFrameBuff = (uint8_t*)malloc((ax25frame->ax25SendFrame->preambleLen + 1 + (6*frameBuffLen)/5 + 2)*sizeof(uint8_t));

  // initialize buffer to all zeros
  memset(stuffedFrameBuff, 0x00, ax25frame->ax25SendFrame->preambleLen + 1 + (6*frameBuffLen)/5 + 2);

  // stuff bits (skip preamble and both flags)
  uint16_t stuffedFrameBuffLenBits = 8*(ax25frame->ax25SendFrame->preambleLen + 1);
  uint8_t count = 0;
  for(size_t i = 0; i < frameBuffLen + 2; i++) 
  {
    for(int8_t shift = 7; shift >= 0; shift--) 
    {
      // for example, if preambleLen == 8, then stuffedFrameBuffPos -> [79 ~ 72] [87 ~ 80] [95 ~ 88] ...
      uint16_t stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);
      if((frameBuff[i] >> shift) & 0x01) 
      {
        // copy 1 and increment counter
        SET_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count++;

        // check 5 consecutive 1s
        if(count == 5) 
        {
          // get the new position in stuffed frame
          stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);

          // insert 0 and reset counter
          CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
          stuffedFrameBuffLenBits++;
          count = 0;
        }
      } 
      else
      {
        // copy 0 and reset counter
        CLEAR_BIT_IN_ARRAY(stuffedFrameBuff, stuffedFrameBuffPos);
        stuffedFrameBuffLenBits++;
        count = 0;
      }
    }
  }

    // deallocate memory
    free(frameBuff);

    // set preamble bytes and start flag field
    for(uint16_t i = 0; i < ax25frame->ax25SendFrame->preambleLen + 1; i++)
        stuffedFrameBuff[i] = RADIOLIB_AX25_FLAG;

    // get stuffed frame length in bytes
    size_t stuffedFrameBuffLen = stuffedFrameBuffLenBits/8 + 1;
    uint8_t trailingLen = stuffedFrameBuffLenBits % 8;

    // set end flag field (may be split into two bytes due to misalignment caused by extra stuffing bits)
    if(trailingLen != 0) 
    {
        stuffedFrameBuffLen++;
        stuffedFrameBuff[stuffedFrameBuffLen - 2] |= RADIOLIB_AX25_FLAG >> trailingLen;
        stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG << (8 - trailingLen);
    } 
    else 
    {
        stuffedFrameBuff[stuffedFrameBuffLen - 1] = RADIOLIB_AX25_FLAG;
    }

    *stuffedFrameLen = stuffedFrameBuffLen;
    *pStuffedFrame = stuffedFrameBuff;
    return RADIOLIB_ERR_NONE;
}

uint16_t AX25Frame_HDLC_Parser(ax25frame_t* ax25frame , uint8_t* stuffedFrame, uint16_t stuffedFrameLen)
{
  // prepare the true length of AX.25 frame
  size_t ax25frameBuffLen = 0;
  // calculate the unstuff bytes length without preamble and front flag 
  size_t frameBuffLen = stuffedFrameLen - ax25frame->ax25RcvFrame->preambleLen - 1;
  // prepare buffer for the unstuffed frame (only AX.25 frame)
  uint8_t* frameBuff = (uint8_t*)malloc(frameBuffLen * sizeof(uint8_t));
  
  // initialize buffer to all zeros
  memset(frameBuff, '\0', frameBuffLen);

  // stuff bits (skip preamble and front flag)
  uint16_t stuffedFrameBuffLenBits = 8*(ax25frame->ax25RcvFrame->preambleLen + 1);

  uint8_t count = 0;
  for(size_t i = 0; i < frameBuffLen * 8; i++)
  {
    uint16_t stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);
    // check if HDLC frame in certain position is 1
    if(GET_BIT_IN_ARRAY(stuffedFrame, stuffedFrameBuffPos))
    {
      SET_BIT_IN_ARRAY(frameBuff, (i + 7 - 2*(i%8)));
      stuffedFrameBuffLenBits++;
      count++;

      // check 5 consecutive 1s
      if(count == 5) 
      {
        // get the new position in stuffed frame
        stuffedFrameBuffPos = stuffedFrameBuffLenBits + 7 - 2*(stuffedFrameBuffLenBits%8);

        // check if HDLC frame in certain position is 0
        if(!GET_BIT_IN_ARRAY(stuffedFrame, stuffedFrameBuffPos))
        {
          stuffedFrameBuffLenBits++;
        }   
        // if HDLC frame in certain position is 1, then there are 6 consecutive 1s. 
        // That byte should be an end flag. 
        else
        {
          ax25frameBuffLen = i/8;
          break;
        }
          
        count = 0;
      }
    } 
    else 
    {
      // copy 0 and reset counter
      CLEAR_BIT_IN_ARRAY(frameBuff, (i + 7 - 2*(i%8)));
      stuffedFrameBuffLenBits++;
      count = 0;
    }
  }

  // Reallocate the AX.25 frame size
  uint8_t* ax25frameBuff = (uint8_t*)malloc(ax25frameBuffLen * sizeof(uint8_t));
  memcpy(ax25frameBuff, frameBuff, ax25frameBuffLen);
  free(frameBuff);

  printf("AX.25 frame\r\n");
	printf("packet: ");
	for(int i = 0; i <  ax25frameBuffLen; i++)
		printf("0x%02x ", ax25frameBuff[i]);
	printf("\r\n");

  // verify CRC result
  uint16_t verifyFcs = checksum(ax25frame, ax25frameBuff, ax25frameBuffLen - 2);
  uint16_t rcvFcs =  (uint16_t)((ax25frameBuff[ax25frameBuffLen-2] << 8) | ax25frameBuff[ax25frameBuffLen-1]);
  if(verifyFcs == rcvFcs)
    ax25frame->ax25RcvFrame->isCrcOk = true;
  else
    ax25frame->ax25RcvFrame->isCrcOk = false;

  // flip bit order
  for(size_t i = 0; i < ax25frameBuffLen; i++) 
  {
      ax25frameBuff[i] = reflect(ax25frameBuff[i], 8);
  }

  // Set pointer to easily track AX.25 frame
  uint8_t* frameBuffPtr = ax25frameBuff;

  // get destination callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  for(size_t i = 0; i < strlen(ax25frame->ax25RcvFrame->destCallsign); i++)
  {
    ax25frame->ax25RcvFrame->destCallsign[i]= *(frameBuffPtr + i) >> 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // get destination SSID
  ax25frame->ax25RcvFrame->destSSID = *(frameBuffPtr++) >> 1;

  // get source callsign - all address field bytes are shifted by one bit to make room for HDLC address extension bit
  for(size_t i = 0; i < strlen(ax25frame->ax25RcvFrame->srcCallsign); i++)
  {
    ax25frame->ax25RcvFrame->srcCallsign[i] = *(frameBuffPtr + i) >> 1;
  }
  frameBuffPtr += RADIOLIB_AX25_MAX_CALLSIGN_LEN;

  // get source SSID
  ax25frame->ax25RcvFrame->srcSSID = *(frameBuffPtr++) >> 1;

  // get sequence numbers of the frames that have it
  uint8_t controlField = *(frameBuffPtr++);
  if((controlField & 0x01) == 0) {
      // information frame, set both sequence numbers
      ax25frame->ax25RcvFrame->rcvSeqNumber = controlField >> 5;
      ax25frame->ax25RcvFrame->sendSeqNumber = controlField >> 1;
  } else if((controlField & 0x02) == 0) {
      // supervisory frame, set only receive sequence number
      ax25frame->ax25RcvFrame->rcvSeqNumber = controlField >> 5;
  }

  // get PID field of the frames that have it
  if(*frameBuffPtr != 0x00) {
      ax25frame->ax25RcvFrame->protocolID = *(frameBuffPtr++);
  }

  // get info field of the frames that have it
  uint8_t infoLen = ax25frameBuffLen - ((2*(RADIOLIB_AX25_MAX_CALLSIGN_LEN + 1)) + 1 + 1 + 2);
  uint8_t* info = (uint8_t*)malloc((infoLen + 1)*sizeof(uint8_t));
  memset(info, '\0', (infoLen + 1)*sizeof(uint8_t));
  memcpy(info, frameBuffPtr, infoLen);
  ax25frame->ax25RcvFrame->infoLen = infoLen;
  ax25frame->ax25RcvFrame->info = info;

  if(ax25frame->ax25RcvFrame->isCrcOk == true)
	  return RADIOLIB_ERR_NONE;
  else
	  return RADIOLIB_ERR_RX_CRC_CHECKSUM;
}

uint32_t reflect(uint32_t in, uint8_t bits) {
  uint32_t res = 0;
  for(uint8_t i = 0; i < bits; i++) {
    res |= (((in & ((uint32_t)1 << i)) >> i) << (bits - i - 1));
  }
  return(res);
}

uint32_t checksum(ax25frame_t* ax25frame, uint8_t* buff, size_t len) {
  uint32_t crc = ax25frame->crc.init;
  size_t pos = 0;
  for(size_t i = 0; i < 8*len; i++) {
    if(i % 8 == 0) {
      uint32_t in = buff[pos++];
      if(ax25frame->crc.refIn) {
        in = reflect(in, 8);
      }
      crc ^= (in << (ax25frame->crc.size - 8));
    }

    if(crc & ((uint32_t)1 << (ax25frame->crc.size - 1))) {
      crc <<= (uint32_t)1;
      crc ^= ax25frame->crc.poly;
    } else {
      crc <<= (uint32_t)1;
    }
  }

  crc ^= ax25frame->crc.out;
  if(ax25frame->crc.refOut) {
    crc = reflect(crc, ax25frame->crc.size);
  }
  crc &= (uint32_t)0xFFFFFFFF >> (32 - ax25frame->crc.size);
  return(crc);
}

void ax25_nrzi_encode(uint8_t* input, uint8_t* output, uint16_t len)
{
//	for(size_t i = preambleLen+1; i < len*8; i++) {
//		size_t currBitPos = i + 7 - 2*(i%8);
//		size_t prevBitPos = (i - 1) + 7 - 2*((i - 1)%8);
//		if(TEST_BIT_IN_ARRAY(input, currBitPos)) {
//			// bit is 1, no change, copy previous bit
//			if(TEST_BIT_IN_ARRAY(input, prevBitPos)) {
//				SET_BIT_IN_ARRAY(output, currBitPos);
//			} else {
//				CLEAR_BIT_IN_ARRAY(output, currBitPos);
//			}
//		} else {
//			// bit is 0, transition, copy inversion of the previous bit
//			if(TEST_BIT_IN_ARRAY(input, prevBitPos)) {
//				CLEAR_BIT_IN_ARRAY(output, currBitPos);
//			} else {
//				SET_BIT_IN_ARRAY(output, currBitPos);
//			}
//		}
//	}
	uint8_t prev_nrzi_bit = 0;
	uint8_t nrz_bit;
	uint8_t nrzi_bit;
	int16_t i, j;

	for (i=0; i<len; i++) {
		for (j=7; j>=0; j--) {
			nrz_bit = input[i]>>j & 0x01;

			if (nrz_bit == 0)
				nrzi_bit = prev_nrzi_bit ^ 1;
			else
				nrzi_bit = prev_nrzi_bit;

			if (nrzi_bit)
				output[i] |= (1 << j);
			else
				output[i] &= ~(1 << j);

			prev_nrzi_bit = nrzi_bit;
		}
	}
}

void ax25_nrzi_decode(uint8_t* input, uint8_t* output, uint16_t len)
{
//	for(size_t i = preambleLen+1; i < len*8; i++) {
//		size_t currBitPos = i + 7 - 2*(i%8);
//		size_t prevBitPos = (i - 1) + 7 - 2*((i - 1)%8);
//		// if current bit == previous bit, then output bit is 0.
//		if(TEST_BIT_IN_ARRAY(input, currBitPos) == TEST_BIT_IN_ARRAY(input, prevBitPos)) {
//			SET_BIT_IN_ARRAY(output, currBitPos);
//		// if current bit != previous bit, then output bit is 1.
//		} else {
//			CLEAR_BIT_IN_ARRAY(output, currBitPos);
//		}
//	}
    uint8_t prev_nrzi_bit = 0;
    uint8_t nrz_bit;
    uint8_t nrzi_bit;
    int16_t i, j;

    for (i=0; i<len; i++) {
        for (j=7; j>=0; j--) {
            nrzi_bit = input[i]>>j & 0x01;

            if (nrzi_bit != prev_nrzi_bit)
                nrz_bit = 0;
            else
                nrz_bit = 1;

            if (nrz_bit)
                output[i] |= (1 << j);
            else
                output[i] &= ~(1 << j);

            prev_nrzi_bit = nrzi_bit;
        }
    }
}

void ax25_g3ruh_scrambler_init(uint32_t tap_mask)
{
  uint8_t i;
  d_tap_count = 0;

  for (i=0; i<32; i++) 
  {
    if ((tap_mask & 0x01) == 1) 
    {
      d_taps[d_tap_count] = i;
      d_tap_count++;
    }
    tap_mask = tap_mask >> 1;
  }

  d_shift_register = 0;
}

void ax25_g3ruh_scrambler(uint8_t* unscrambled, uint8_t* scrambled, uint16_t len)
{
	uint8_t unscrambled_bit;
	uint8_t scrambled_bit;
	uint32_t tap_bit;
	int16_t i, j, t;

	for (i=0; i<len; i++)
	{
		for (j=7; j>=0; j--)
		{
			unscrambled_bit = unscrambled[i]>>j & 0x01;
			d_shift_register <<= 1;

			scrambled_bit = unscrambled_bit;
			for (t=0; t<d_tap_count; t++)
			{
				tap_bit = (d_shift_register >> d_taps[t]) & 0x01;
				scrambled_bit = scrambled_bit ^ tap_bit;
			}

			d_shift_register |= scrambled_bit;

			if (scrambled_bit)
				scrambled[i] |= (1 << j);
			else
				scrambled[i] &= ~(1 << j);
		}
	}
}

void ax25_g3ruh_descrambler(uint8_t* scrambled, uint8_t* unscrambled, uint16_t len)
{
  uint8_t unscrambled_bit;
  uint8_t scrambled_bit;
  uint32_t tap_bit;
  int16_t i, j, t;

  for (i=0; i<len; i++) 
  {
    for (j=7; j>=0; j--) 
    {
      scrambled_bit = scrambled[i]>>j & 0x01;
      d_shift_register <<= 1;

      unscrambled_bit = scrambled_bit;
      for (t=0; t<d_tap_count; t++) 
      {
        tap_bit = (d_shift_register >> d_taps[t]) & 0x01;
        unscrambled_bit = unscrambled_bit ^ tap_bit;
      }

      d_shift_register |= scrambled_bit;

      if (unscrambled_bit)
        unscrambled[i] |= (1 << j);
      else
        unscrambled[i] &= ~(1 << j);
    }
  }
}
