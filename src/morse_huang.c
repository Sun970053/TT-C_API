/*
 * morse_huang.c
 *
 *  Created on: August 12, 2024
 *      Author: Ting-Shan, Huang
 */

#include "morse_huang.h"

#define MCODE(LEN, PATTERN) ( (PATTERN << 3) | ( LEN & 0x7 ) )
#define M_LEN(X) ( X & 0x7 )
#define M_PATTERN(X) ( X >> 3 )

const unsigned char morseTable[] = { 
    '0', MCODE(5,  0b11111 ),
    '1', MCODE(5,  0b01111 ),
    '2', MCODE(5,  0b00111 ),
    '3', MCODE(5,  0b00011 ),
    '4', MCODE(5,  0b00001 ),
    '5', MCODE(5,  0b00000 ),
    '6', MCODE(5,  0b10000 ),
    '7', MCODE(5,  0b11000 ),
    '8', MCODE(5,  0b11100 ),
    '9', MCODE(5,  0b11110 ),
    '/', MCODE(5,  0b10010 ),
    'A', MCODE(2,  0b01 ),
    'B', MCODE(4,  0b1000 ),
    'C', MCODE(4,  0b1010 ),
    'D', MCODE(3,  0b100 ),
    'E', MCODE(1,  0b0 ),
    'F', MCODE(4,  0b0010 ),
    'G', MCODE(3,  0b110 ),
    'H', MCODE(4,  0b0000 ),
    'I', MCODE(2,  0b00 ),
    'J', MCODE(4,  0b0111 ),
    'K', MCODE(3,  0b101 ),
    'L', MCODE(4,  0b0100 ),
    'M', MCODE(2,  0b11 ),
    'N', MCODE(2,  0b10 ),
    'O', MCODE(3,  0b111 ),
    'P', MCODE(4,  0b0110 ),
    'Q', MCODE(4,  0b1101 ),
    'R', MCODE(3,  0b010 ),
    'S', MCODE(3,  0b000 ),
    'T', MCODE(1,  0b1 ),
    'U', MCODE(3,  0b001 ),
    'V', MCODE(4,  0b0001 ),
    'W', MCODE(3,  0b011 ),
    'X', MCODE(4,  0b1001 ),
    'Y', MCODE(4,  0b1011 ),
    'Z', MCODE(4,  0b1100 ),
    0,0
 };

void setMorseText(char* text)
{
    
}

morse_state prepareNextChar(void);
morse_state prepareNextDitDah(void);
