/*
 * morse_huang.h
 *
 *  Created on: August 12, 2024
 *      Author: Ting-Shan, Huang
 */

#include <stdint.h>

#define MORSE_MAX_TEXT_LENGTH (80)

static const unsigned char _morseTable[42*2];

typedef enum
{
    msStopped,
    msWarmUpPause,
    msSendingDit,
    msSendingDah,
    msSendingPauseBetweenDitDah,
    msSendingPauseBetweenLetters,
    msSendingSpace,
    msSendingLongCarrier,
    msEndOfText,
    msLongPauseBetweenRepeat
} morse_state;

typedef struct
{
    void                        (*DelayUs)(uint32_t delay);
    char                        text[MORSE_MAX_TEXT_LENGTH];
    uint16_t                    currentPosInText;
} morse_t;

void setMorseText(char* text);
morse_state prepareNextChar(void);
morse_state prepareNextDitDah(void);