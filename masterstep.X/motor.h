/*
 * File: motor.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the stepper motor-control module.
 * Defines motor direction, state, step mode, runtime data structure,
 * and public functions for motor initialization, enabling, direction
 * control, speed configuration, start/stop control, and status feedback.
 *
 * Created: March 14, 2026
 */

#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* Motor rotation direction */
typedef enum {
    MOTOR_DIRECTION_POSITIVE = 0,
    MOTOR_DIRECTION_NEGATIVE
} MOTOR_Direction_t;

/* Motor operating states */
typedef enum {
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MOTOR_State_t;

/* Stepper motor excitation modes */
typedef enum {
    MOTOR_STEP_MODE_FULL = 0,
    MOTOR_STEP_MODE_HALF
} MOTOR_StepMode_t;

/* Runtime motor-control object */
typedef struct {
    MOTOR_State_t state;
    MOTOR_Direction_t direction;
    uint16_t stepFrequencyHz;
    bool enabled;
    bool running;
} MOTOR_t;

/* Initialization and timer service */
void MOTOR_Initialise(MOTOR_t *motor);
void MOTOR_TimerISR(void);

/* Enable control */
void MOTOR_Enable(MOTOR_t *motor);
void MOTOR_Disable(MOTOR_t *motor);
bool MOTOR_IsEnabled(const MOTOR_t *motor);

/* Step mode control */
void MOTOR_SetStepMode(MOTOR_t *motor, MOTOR_StepMode_t mode);

/* Direction control */
void MOTOR_SetDirection(MOTOR_t *motor, MOTOR_Direction_t direction);
MOTOR_Direction_t MOTOR_GetDirection(const MOTOR_t *motor);

/* Speed control */
void MOTOR_SetStepFrequencyHz(MOTOR_t *motor, uint16_t frequencyHz);
uint16_t MOTOR_GetStepFrequencyHz(const MOTOR_t *motor);

/* Motion control */
void MOTOR_Start(MOTOR_t *motor);
void MOTOR_Stop(MOTOR_t *motor);
bool MOTOR_IsRunning(const MOTOR_t *motor);

/* Status feedback */
MOTOR_State_t MOTOR_GetState(const MOTOR_t *motor);

/* Periodic motor state update */
void MOTOR_Task(MOTOR_t *motor);

#endif	/* MOTOR_H */

