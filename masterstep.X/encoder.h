/*
 * File: encoder.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the encoder module.
 * Defines encoder states, result codes, runtime data structure,
 * interrupt service functions, position acquisition functions,
 * zero-reference handling, and status feedback functions.
 *
 * Created: March 15, 2026
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ENCODER_STATE_UNINITIALISED = 0,
    ENCODER_STATE_READY,
    ENCODER_STATE_ERROR
} ENCODER_State_t;

typedef enum {
    ENCODER_OK = 0,
    ENCODER_ERROR_COMM,
    ENCODER_ERROR_NOT_INIT,
    ENCODER_ERROR_NULL,
    ENCODER_ERROR_INVALID,
    ENCODER_ERROR_INVALID_DATA
} ENCODER_Result_t;

typedef struct {
    ENCODER_State_t state;
    int32_t rawCount;
    int32_t zeroOffset;
    float scaleMmPerCount;
    bool valid;
    float positionMm;
} ENCODER_t;

/* Interrupt service functions */
void ENCODER_ISR_Update(void);
void ENCODER_ISR_A(void);
void ENCODER_ISR_B(void);
void ENCODER_ResetHardwareCount(void);

/* Initialization and periodic update */
void ENCODER_Initialise(ENCODER_t *encoder);
void ENCODER_Task (ENCODER_t *encoder);
ENCODER_Result_t ENCODER_Update(ENCODER_t *encoder);

/* Position feedback */
int32_t ENCODER_GetRawCount(const ENCODER_t *encoder);
float ENCODER_GetPositionMm(const ENCODER_t *encoder);

/* Zero-reference handling */
void ENCODER_SetZero(ENCODER_t *encoder);
void ENCODER_SetZeroAtRawCount (ENCODER_t *encoder, int32_t rawCount);

/* Status feedback */
bool ENCODER_IsValid(const ENCODER_t *encoder);
ENCODER_State_t ENCODER_GetState(const ENCODER_t *encoder);

/* Simulation support */
void ENCODER_SetPositionMm(ENCODER_t *encoder, float positionMm);

#endif	/* ENCODER_H */

