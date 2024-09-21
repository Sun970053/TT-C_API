/*
 * morse_huang.c
 *
 *  Created on: August 12, 2024
 *      Author: Ting-Shan, Huang
 */

#include "morse_huang.h"
#include <string.h>

// configurable params
#define UNITSIZE_IN_MS                  (150)
#define WARMUP_PAUSE_IN_MS              (3000)
#define ENDOFTEXT_PAUSE_IN_MS           (3000)
// #define LONG_RETRY_PAUSE_IN_MS          (30000)
#define LONG_RETRY_PAUSE_IN_MS          (1000)
#define LONG_CARRIER_DURATION_IN_MS     (4000)

#define MCODE(LEN, PATTERN)             ( (PATTERN << 3) | ( LEN & 0x7 ) )
#define M_LEN(X)                        ( X & 0x7 )
#define M_PATTERN(X)                    ( X >> 3 )

static const unsigned char morseTable[] = { 
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

void morse_setText(morse_t* morse, char* text)
{
    size_t len = strlen(text);
    if( len > MORSE_MAX_TEXT_LENGTH -1 )
    {
        len = MORSE_MAX_TEXT_LENGTH - 1;
    }
    strncpy(morse->text, text, len + 1);
}

void morse_handleTimeout(morse_t* morse)
{
    if(morse->currentState == msStopped)
    {
        return;
    }

    if(morse->time.millisecondsElapsed(&(morse->time)) > morse->currentTimeoutInMS)
    {
        morse->time.resetToNow(&(morse->time));
        morse_changeStateByTimeout(morse);
    }
}

morse_state morse_prepareNextChar(morse_t* morse)
{
    char ch = morse->text[morse->currentPosInText];
    morse->currentPosInText++;

    if(ch == 0)
    {
        // reset current position in text.
        morse->currentPosInText = 0;
        return msEndOfText;
    }
    else if(ch == ' ')
    {
        return msSendingSpace;
    }
    else if(ch == '_')
    {
        return msSendingLongCarrier;
    }
    else
    {
        int length = 0;
        unsigned char pattern = 0;
        if( morse_findCharInTable(ch, &length, &pattern) )
        {
            // align pattern
            pattern <<= (8 - length);

            morse->currentUnitPattern = pattern;
            morse->numUnitRest = length;
            return morse_prepareNextDitDah(morse);
        }
        else
        {
            // send space
            return msSendingSpace;
        }
    }
}

morse_state morse_prepareNextDitDah(morse_t* morse)
{
    if(morse->numUnitRest == 0 )
    {
        return msSendingPauseBetweenLetters;
    }

    morse_state st = (morse->currentUnitPattern & 0x80)? msSendingDah : msSendingDit;
    morse->currentUnitPattern <<= 1;
    morse->numUnitRest -= 1;
    return st;
}

bool morse_findCharInTable(char ch, int* outLength, uint8_t* outPattern)
{
    size_t idx=0;
    while( morseTable[idx] )
    {
        if( morseTable[idx] == ch )
        {
            unsigned char encodedPatter = morseTable[idx + 1];
            int len = M_LEN( encodedPatter );
            unsigned char pattern = M_PATTERN( encodedPatter );

            *outLength = len;
            *outPattern = pattern;
            return true;
        }

        idx += 2;
    }
    return false;
}

void morse_changeStateByTimeout(morse_t* morse)
{
	morse_state state;
    switch (morse->currentState)
    {
    case msStopped:
        // nothing to do. (should never happen)
        break;
    case msWarmUpPause:
        // start sending
        morse->currentPosInText = 0;
        state = morse_prepareNextChar(morse); // possible: dit, dah, space, longCarrier, endoftext
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msSendingDit:
    case msSendingDah:
        state = msSendingPauseBetweenDitDah;
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msSendingPauseBetweenDitDah:
        state = morse_prepareNextDitDah(morse); // possible: dit, dah, pause_letters
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msSendingPauseBetweenLetters:
        state = morse_prepareNextChar(morse);  // possible: dit, dah, space, longCarrier, endoftext
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msSendingSpace:
    case msSendingLongCarrier:
        state = morse_prepareNextChar(morse);  // possible: dit, dah, space, longCarrier, endoftext
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msEndOfText:
        state = msLongPauseBetweenRepeat;
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    case msLongPauseBetweenRepeat:
        state = msWarmUpPause;
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
        break;
    default:
        // never happen
        morse_state state = msWarmUpPause;
        morse->currentTimeoutInMS = morse_getTimeoutForState(state);
        morse->currentState = state;
    }
}

int32_t morse_getTimeoutForState(morse_state state)
{
    int32_t res = 0;

    // UNITSIZE_IN_MS: dit duration
    switch(state)
    {
    case msWarmUpPause:
        res = WARMUP_PAUSE_IN_MS;
        break;
    case msSendingDit:
        res = UNITSIZE_IN_MS;
        break;
    case msSendingDah:
        res = 3 * UNITSIZE_IN_MS;
        break;
    case msSendingPauseBetweenDitDah:
        res = UNITSIZE_IN_MS;
        break;
    case msSendingPauseBetweenLetters:
        res = 3 * UNITSIZE_IN_MS;
        break;
    case msSendingSpace:
        res = 7 * UNITSIZE_IN_MS;
        break;
    case msSendingLongCarrier:
        res = LONG_CARRIER_DURATION_IN_MS;
        break;
    case msEndOfText:
        res = ENDOFTEXT_PAUSE_IN_MS;
        break;
    case msLongPauseBetweenRepeat:
        res = LONG_RETRY_PAUSE_IN_MS;
        break;
    default:
        res = UNITSIZE_IN_MS;
    }
    return res;
}

bool morse_isToneActive(morse_t* morse)
{
	return (morse->currentState == msSendingDit)
			|| (morse->currentState == msSendingDah)
			|| (morse->currentState == msSendingLongCarrier);
}

void morse_start(morse_t* morse)
{
    morse_state state = msWarmUpPause;
    morse->currentTimeoutInMS = morse_getTimeoutForState(state);
    morse->currentState = state;
}

void morse_stop(morse_t* morse)
{
    morse->currentState = msStopped;
}
