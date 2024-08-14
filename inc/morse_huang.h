/*
 * morse_huang.h
 *
 *  Created on: August 12, 2024
 *      Author: Ting-Shan, Huang
 */

#include <stdint.h>
#include <stdbool.h>

#define MORSE_MAX_TEXT_LENGTH (80)

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

struct timerange;

typedef struct timerange
{
    uint32_t (*millisecondsElapsed)(struct timerange* time);
    uint32_t (*secondsElapsed)(struct timerange* time);
    void (*resetToNow)(struct timerange* time);
    uint32_t stamp;
} timerange_t;

typedef struct
{
    void (*DelayUs)(uint32_t delay);
    char text[MORSE_MAX_TEXT_LENGTH];
    uint16_t currentPosInText;
    morse_state currentState;

    timerange_t time;
    int32_t currentTimeoutInMS;

    uint8_t currentUnitPattern;
    int numUnitRest;
} morse_t;

void morse_setText(morse_t* morse, char* text);
void morse_handleTimeout(morse_t* morse);
morse_state morse_prepareNextChar(morse_t* morse);
morse_state morse_prepareNextDitDah(morse_t* morse);
bool morse_findCharInTable(char ch, int* length, uint8_t* pattern);
void morse_changeStateByTimeout(morse_t* morse);
int32_t morse_getTimeoutForState(morse_state state);
bool morse_isToneActive(morse_t* morse);
void morse_start(morse_t* morse);
void morse_stop(morse_t* morse);
