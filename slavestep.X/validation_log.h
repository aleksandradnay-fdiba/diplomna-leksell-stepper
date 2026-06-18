/* 
 * File:   validation_log.h
 * Author: acer
 *
 * Created on June 18, 2026, 7:18 PM
 */

#ifndef VALIDATION_LOG_H
#define VALIDATION_LOG_H

#include <stdint.h>
#include <stdbool.h>

#define VALIDATION_LOG_SIZE 40u

typedef struct {
    uint8_t testId;
    uint8_t repetition;

    float targetMm;

    int32_t homeRawCount;
    float homeMm;

    int32_t finalRawCount;
    float finalMm;

    float errorMm;

} VALIDATION_LogEntry_t;

extern VALIDATION_LogEntry_t g_validationLog[VALIDATION_LOG_SIZE];
extern uint8_t g_validationLogCount;

void VALIDATION_LOG_Clear(void);

bool VALIDATION_LOG_Add(uint8_t testId,
                        uint8_t repetition,
                        float targetMm,
                        int32_t homeRawCount,
                        float homeMm,
                        int32_t finalRawCount,
                        float finalMm);

#endif