#include "validation_log.h"

VALIDATION_LogEntry_t g_validationLog[VALIDATION_LOG_SIZE];
uint8_t g_validationLogCount = 0u;

void VALIDATION_LOG_Clear(void) {
    g_validationLogCount = 0u;
}

bool VALIDATION_LOG_Add(uint8_t testId,
                        uint8_t repetition,
                        float targetMm,
                        int32_t homeRawCount,
                        float homeMm,
                        int32_t finalRawCount,
                        float finalMm) {
    
    if (g_validationLogCount >= VALIDATION_LOG_SIZE) return false;

    VALIDATION_LogEntry_t *entry = &g_validationLog[g_validationLogCount];

    entry->testId = testId;
    entry->repetition = repetition;
    entry->targetMm = targetMm;

    entry->homeRawCount = homeRawCount;
    entry->homeMm = homeMm;

    entry->finalRawCount = finalRawCount;
    entry->finalMm = finalMm;

    entry->errorMm = finalMm - targetMm;

    g_validationLogCount++;

    return true;
}
