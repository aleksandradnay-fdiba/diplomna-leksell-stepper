/*
 * File:   millisecond_timers.c
 * Author: sENEV
 *
 * Created on April 7, 2025, 2:28 PM
 */


#include "xc.h"
#include "milliecond_timers.h"


volatile uint32_t milliseconds[TIMERS_MAX_COUNT];
volatile uint8_t activeTimersCount = TIMERS_MAX_COUNT;

volatile uint32_t millisecondsVar = 0;

void millisecondEventHandler(void){
    for (int j = 0; j < activeTimersCount; j++) {
        milliseconds[j]++;
    }
    
    ++millisecondsVar;
}

uint8_t setActiveTimersCount(uint8_t count)
{
    if(count <= TIMERS_MAX_COUNT) {
        activeTimersCount = count;
        return 1;
    }
    
    activeTimersCount = TIMERS_MAX_COUNT;
    return 0;
}

uint8_t timer(uint8_t timer, uint32_t period)
{
    return (milliseconds[timer - 1] >= period) ? 1 : 0;
}

void resetTimer(uint8_t timer) {
    milliseconds[timer - 1] = 0;
}

uint32_t readTimer(uint8_t timer) 
{
    return milliseconds[timer - 1];
}

void initTimers()
{
    for (int j = 0; j < TIMERS_MAX_COUNT; j++) {
        milliseconds[j] = 0;
    }
}

uint32_t millis()
{
    return millisecondsVar;
}
