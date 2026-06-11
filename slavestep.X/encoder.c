/*
 * File: encoder.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements encoder position processing for the axis-control system.
 * The module maintains the raw encoder count, zero offset, position
 * conversion, hardware count acquisition, and interrupt-based count
 * update logic.
 *
 * The encoder can be compiled either for hardware operation or for
 * software simulation used during validation.
 *
 * Created: March 15, 2026
 */

#include "encoder.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/interrupt_manager.h"
#include <stdint.h>
#include <stdbool.h>

#define ENCODER_DEFAULT_SCALE_MM_PER_COUNT (0.002f)
#define ENCODER_DEFAULT_POSITION_MM (0.0f)
#define ENCODER_DEFAULT_RAW_COUNT (0)

/* Enables software-only encoder simulation for validation builds. */
#define ENCODER_ENABLE_SIMULATION (0u)

#define ENCODER_MODE ENCODER_MODE_AB_QUADRATURE

/* Global encoder count updated from interrupt context */
static volatile int32_t g_encoderRawCount = 0;
static volatile uint8_t g_encoderPrevAB = 0;
static volatile bool g_encoderPrevABValid = false;

static bool ENCODER_IsScaleValid(const ENCODER_t *encoder);
static int32_t ENCODER_ConvertPositionToRawCount(const ENCODER_t *encoder, float positionMm);
static float ENCODER_ConvertRawCountToPositionMm(const ENCODER_t *encoder, int32_t rawCount);
static void ENCODER_UpdateDerivedPosition(ENCODER_t *encoder);
static bool ENCODER_ReadHardwareRawCount(const ENCODER_t *encoder, int32_t *rawCount);

static uint8_t ENCODER_ReadAB(void) {
    uint8_t a;
    a = IO_RB0_GetValue();
    uint8_t b;
    b = IO_RB1_GetValue();
    return (uint8_t)((a<<1) | b);
}

 /* A-only mode counts rising edges of channel A and uses channel B for direction. */
void ENCODER_ISR_Update(void) {
    uint8_t a;
    a = IO_RB0_GetValue();
    uint8_t b;
    b = IO_RB1_GetValue();
    
    if (!g_encoderPrevABValid) {
        g_encoderPrevAB = a;
        g_encoderPrevABValid = true;
        return;
    }

    if ((g_encoderPrevAB == 0u) && (a == 1u)) {
        if (b == 0u)  g_encoderRawCount++;
        else g_encoderRawCount--;
    }
    g_encoderPrevAB = a; 
}

/* Use following two interrupt handlers for AB quadrature mode */
void ENCODER_ISR_A(void) {
    if (IO_RB0_GetValue()) {
        if (IO_RB1_GetValue()) {
            --g_encoderRawCount;
        } else {
            ++g_encoderRawCount;
        }
    } else {
        if (IO_RB1_GetValue()) {
            ++g_encoderRawCount;
        } else {
            --g_encoderRawCount;
        }
    }
}

void ENCODER_ISR_B(void) {
    if (IO_RB1_GetValue()) {
        if (IO_RB0_GetValue()) {
            ++g_encoderRawCount;
        } else {
            --g_encoderRawCount;
        }
    } else {
        if (IO_RB0_GetValue()) {
            --g_encoderRawCount;
        } else {
            ++g_encoderRawCount;
        }
    }
}

void ENCODER_ResetHardwareCount(void) {
    INTERRUPT_GlobalInterruptHighDisable();
    g_encoderRawCount = 0;
    g_encoderPrevAB = ENCODER_ReadAB();
    g_encoderPrevABValid = true;
    INTERRUPT_GlobalInterruptHighEnable();
}

static bool ENCODER_PlatformReadRawCount(const ENCODER_t *encoder, int32_t *rawCount){
    (void)encoder;
    
    if (rawCount == 0) return false;
    
    INTERRUPT_GlobalInterruptHighDisable();
    *rawCount = g_encoderRawCount;
    INTERRUPT_GlobalInterruptHighEnable();
    
    return true;
}

static bool ENCODER_IsScaleValid(const ENCODER_t *encoder) {
    if (encoder == 0) return false;
    
    return(encoder->scaleMmPerCount > 0.0f);
}

static int32_t ENCODER_ConvertPositionToRawCount(const ENCODER_t *encoder, float positionMm) {
    float rawCountFloat;
    
    if ((encoder == 0) || !ENCODER_IsScaleValid(encoder)) return 0;
    
    rawCountFloat = positionMm / encoder->scaleMmPerCount;
    
    /* Round to the nearest integer raw count. */
    if (rawCountFloat >= 0.0f) rawCountFloat += 0.5f;
    else rawCountFloat -= 0.5f;
    
    return (int32_t)rawCountFloat;
}

static float ENCODER_ConvertRawCountToPositionMm (const ENCODER_t *encoder, int32_t rawCount) {
    if((encoder == 0) || !ENCODER_IsScaleValid(encoder)) return 0.0f;
    return ((float)rawCount) * encoder->scaleMmPerCount;
}

static void ENCODER_UpdateDerivedPosition(ENCODER_t *encoder) {
    int32_t relativeCount;
    
    if ((encoder == 0) || !ENCODER_IsScaleValid(encoder)) return;
    
    relativeCount = encoder->rawCount - encoder->zeroOffset;
    encoder->positionMm = ENCODER_ConvertRawCountToPositionMm(encoder, relativeCount);
}

/* Selects between simulated and hardware encoder count acquisition. */
static bool ENCODER_ReadHardwareRawCount(const ENCODER_t *encoder, int32_t *rawCount) {
    if((encoder == 0) || (rawCount == 0)) return false;
       
#if (ENCODER_ENABLE_SIMULATION == 1u)
    *rawCount = encoder->rawCount;
    return true;
#else 
    return ENCODER_PlatformReadRawCount(encoder, rawCount);
#endif
}

void ENCODER_Initialise(ENCODER_t *encoder) {
    if (encoder == 0) return;
    
    encoder->state = ENCODER_STATE_READY;
    encoder->rawCount = ENCODER_DEFAULT_RAW_COUNT;
    encoder->zeroOffset = ENCODER_DEFAULT_RAW_COUNT;
    encoder->scaleMmPerCount = ENCODER_DEFAULT_SCALE_MM_PER_COUNT; 
    encoder->valid = true;
    encoder->positionMm = ENCODER_DEFAULT_POSITION_MM;
    
    #if (ENCODER_ENABLE_SIMULATION == 0u)
    ENCODER_ResetHardwareCount();
    #endif
}

void ENCODER_Task(ENCODER_t *encoder) {
    if (encoder == 0) return;
}

ENCODER_Result_t ENCODER_Update(ENCODER_t *encoder){
    int32_t rawCount;
    
    if (encoder == 0) return ENCODER_ERROR_NULL;
    
    if (encoder->state == ENCODER_STATE_UNINITIALISED) {
        encoder->valid = false;
        return ENCODER_ERROR_NOT_INIT;
    }
    
    if (!ENCODER_IsScaleValid(encoder)) {
        encoder->valid = false;
        encoder->state = ENCODER_STATE_ERROR;
        return ENCODER_ERROR_INVALID;          
    }  
    
    
    if (!ENCODER_ReadHardwareRawCount(encoder, &rawCount)) {
        encoder->valid = false;
        encoder->state = ENCODER_STATE_ERROR;
        return ENCODER_ERROR_INVALID;
    }
    
    encoder->rawCount = rawCount;
    ENCODER_UpdateDerivedPosition(encoder);
    
    encoder->valid = true;
    encoder->state = ENCODER_STATE_READY;
    
    return ENCODER_OK;
}

int32_t ENCODER_GetRawCount(const ENCODER_t *encoder) {
    if(encoder == 0) return 0;
    return encoder->rawCount;
}

float ENCODER_GetPositionMm(const ENCODER_t *encoder) { 
    if(encoder == 0) return 0.0f;
    if(!ENCODER_IsScaleValid(encoder)) return 0.0f;
    return encoder->positionMm;  
}

void ENCODER_SetZero(ENCODER_t *encoder){
    
    int32_t rawCount;
    if(encoder == 0) return;
    
    encoder->zeroOffset = encoder->rawCount;
    ENCODER_UpdateDerivedPosition(encoder);
    if (ENCODER_ReadHardwareRawCount(encoder, &rawCount))
    {
        encoder->rawCount = rawCount;
        encoder->zeroOffset = rawCount;
        ENCODER_UpdateDerivedPosition(encoder);
    }
}

void ENCODER_SetZeroAtRawCount(ENCODER_t *encoder, int32_t rawCount) {
    
    if (encoder == 0) return;
    
    encoder->zeroOffset = rawCount;
    ENCODER_UpdateDerivedPosition(encoder);
}

bool ENCODER_IsValid(const ENCODER_t *encoder) {
    if (encoder == 0) return false;
    return encoder->valid;
}

ENCODER_State_t ENCODER_GetState(const ENCODER_t *encoder){
    if(encoder == 0) return ENCODER_STATE_ERROR;
    return encoder->state;
}

/*
 * Sets the simulated encoder position.
 * This function is used only when encoder simulation is enabled.
 */
void ENCODER_SetPositionMm(ENCODER_t *encoder, float positionMm) {
    int32_t relativeRawCount;
    if (encoder == 0) return;

    relativeRawCount = ENCODER_ConvertPositionToRawCount(encoder, positionMm);

    encoder->positionMm = positionMm;
    encoder->rawCount = encoder->zeroOffset + relativeRawCount;
}