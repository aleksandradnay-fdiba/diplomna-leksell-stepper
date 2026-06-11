/*
 * File: motor.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements stepper motor control for the axis-control system.
 * The module manages motor enable/disable control, direction selection,
 * step frequency configuration, full-step and half-step commutation,
 * and timer-interrupt-based step generation.
 *
 * Created: April 14, 2026
 */

#include "motor.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr2.h"
#include "mcc_generated_files/pwm5.h"
#include "mcc_generated_files/pwm6.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_DEFAULT_DIRECTION (MOTOR_DIRECTION_POSITIVE)
#define MOTOR_MIN_STEP_FREQUENCY_HZ (1u)
#define MOTOR_MAX_STEP_FREQUENCY_HZ (5000u)

#define MOTOR_ENABLE_SIMULATION (0u)

/* Timer interrupt frequency used for software step generation. */
#define MOTOR_TIMER_ISR_HZ (489u)

#define MOTOR_PWM_DUTY_OFF             (0u)
#define MOTOR_PWM_DUTY_ON              (511u)

#define PHASE_A_IN1_ON()               IO_RA0_SetHigh()
#define PHASE_A_IN1_OFF()              IO_RA0_SetLow()
#define PHASE_A_IN2_ON()               IO_RA1_SetHigh()
#define PHASE_A_IN2_OFF()              IO_RA1_SetLow()

#define PHASE_B_IN1_ON()               IO_RA2_SetHigh()
#define PHASE_B_IN1_OFF()              IO_RA2_SetLow()
#define PHASE_B_IN2_ON()               IO_RA3_SetHigh()
#define PHASE_B_IN2_OFF()              IO_RA3_SetLow()

#define PHASE_A_PWM_ON()               PWM5_LoadDutyValue(MOTOR_PWM_DUTY_ON)
#define PHASE_A_PWM_OFF()              PWM5_LoadDutyValue(MOTOR_PWM_DUTY_OFF)

#define PHASE_B_PWM_ON()               PWM6_LoadDutyValue(MOTOR_PWM_DUTY_ON)
#define PHASE_B_PWM_OFF()              PWM6_LoadDutyValue(MOTOR_PWM_DUTY_OFF)

static volatile bool g_motorStepEnabled = false;
static volatile uint16_t g_motorTicksPerStep = 1u;
static volatile uint16_t g_motorTickCounter = 0u;
static volatile uint8_t g_motorStepIndex = 0u;
static volatile MOTOR_Direction_t g_motorDirection = MOTOR_DIRECTION_POSITIVE;
static volatile MOTOR_StepMode_t g_motorStepMode = MOTOR_STEP_MODE_HALF;

static uint16_t MOTOR_ClampStepFrequencyHz(uint16_t frequecyHz);

static void MOTOR_AllOutputsOff(void);

static void MOTOR_PhaseAForward(void);
static void MOTOR_PhaseAReverse(void);
static void MOTOR_PhaseAOff(void);

static void MOTOR_PhaseBForward(void);
static void MOTOR_PhaseBReverse(void);
static void MOTOR_PhaseBOff(void);

static void MOTOR_ApplyFullStep(uint8_t index);
static void MOTOR_ApplyHalfStep(uint8_t index);
static void MOTOR_ApplyCurrentStep(void);
static void MOTOR_AdvanceStep(void);

static void MOTOR_ApplyEnablePin(bool enable);
static void MOTOR_ApplyDirectionPin(MOTOR_Direction_t direction);
static void MOTOR_ApplyStepGenerator(uint16_t frequencyHz, bool enable);
static void MOTOR_UpdateState(MOTOR_t *motor);

/* Low-level phase output handling */
static void MOTOR_AllOutputsOff(void){
    PHASE_A_PWM_OFF();
    PHASE_B_PWM_OFF();

    PHASE_A_IN1_OFF();
    PHASE_A_IN2_OFF();

    PHASE_B_IN1_OFF();
    PHASE_B_IN2_OFF();
}


static void MOTOR_PhaseAForward(void) {
    PHASE_A_IN1_ON();
    PHASE_A_IN2_OFF();
    PHASE_A_PWM_ON();
}

static void MOTOR_PhaseAReverse(void) {
    PHASE_A_IN1_OFF();
    PHASE_A_IN2_ON();
    PHASE_A_PWM_ON();
}

static void MOTOR_PhaseAOff(void) {
    PHASE_A_PWM_OFF();
    PHASE_A_IN1_OFF();
    PHASE_A_IN2_OFF();
}

static void MOTOR_PhaseBForward(void) {
    PHASE_B_IN1_ON();
    PHASE_B_IN2_OFF();
    PHASE_B_PWM_ON();
}

static void MOTOR_PhaseBReverse(void) {
    PHASE_B_IN1_OFF();
    PHASE_B_IN2_ON();
    PHASE_B_PWM_ON();
}

static void MOTOR_PhaseBOff(void) {
    PHASE_B_PWM_OFF();
    PHASE_B_IN1_OFF();
    PHASE_B_IN2_OFF();
}

/* Full-step commutation sequence */
static void MOTOR_ApplyFullStep(uint8_t index) {
    
    switch (index & 0x03u) {
        case 0:
            MOTOR_PhaseAForward();
            MOTOR_PhaseBForward();
            break;
            
        case 1:
            MOTOR_PhaseAForward();
            MOTOR_PhaseBReverse();
            break;
            
        case 2:
            MOTOR_PhaseAReverse();
            MOTOR_PhaseBReverse();
            break;

        case 3:
        default:
            MOTOR_PhaseAReverse();
            MOTOR_PhaseBForward();
            break;  
    }
}

/* Half-step commutation sequence */
static void MOTOR_ApplyHalfStep(uint8_t index) {
    switch (index & 0x07u) {
        case 0:
            MOTOR_PhaseAForward();
            MOTOR_PhaseBForward();
            break;

        case 1:
            MOTOR_PhaseAForward();
            MOTOR_PhaseBOff();
            break;

        case 2:
            MOTOR_PhaseAForward();
            MOTOR_PhaseBReverse();
            break;

        case 3:
            MOTOR_PhaseAOff();
            MOTOR_PhaseBReverse();
            break;

        case 4:
            MOTOR_PhaseAReverse();
            MOTOR_PhaseBReverse();
            break;

        case 5:
            MOTOR_PhaseAReverse();
            MOTOR_PhaseBOff();
            break;

        case 6:
            MOTOR_PhaseAReverse();
            MOTOR_PhaseBForward();
            break;

        case 7:
        default:
            MOTOR_PhaseAOff();
            MOTOR_PhaseBForward();
            break;
    }
}

static void MOTOR_ApplyCurrentStep(void) {
    if(g_motorStepMode == MOTOR_STEP_MODE_HALF) MOTOR_ApplyHalfStep(g_motorStepIndex);
    else MOTOR_ApplyFullStep(g_motorStepIndex);
}

static void MOTOR_AdvanceStep(void) {
    uint8_t maxIndex;
    
    if (g_motorStepMode == MOTOR_STEP_MODE_HALF) maxIndex = 8u;
    else maxIndex = 4u;
    
    if (g_motorDirection == MOTOR_DIRECTION_POSITIVE) {
        g_motorStepIndex++;
        if (g_motorStepIndex >= maxIndex) g_motorStepIndex = 0u;
    } else {
        if (g_motorStepIndex == 0u) g_motorStepIndex = maxIndex - 1u;
        else g_motorStepIndex--;
    }
    MOTOR_ApplyCurrentStep();
}

static void MOTOR_ApplyEnablePin(bool enable) {
#if (MOTOR_ENABLE_SIMULATION == 1u)
    (void)enable;
#else
     if (!enable) {
        g_motorStepEnabled = false;
        MOTOR_AllOutputsOff();
    }
#endif   
}

static void MOTOR_ApplyDirectionPin(MOTOR_Direction_t direction) {
#if (MOTOR_ENABLE_SIMULATION == 1u)
    (void)direction;
#else
      g_motorDirection = direction;
#endif
}

static void MOTOR_ApplyStepGenerator(uint16_t frequencyHz, bool enable){
#if (MOTOR_ENABLE_SIMULATION == 1u)
    (void)frequencyHz;
    (void)enable;
    
#else
    uint32_t ticks;

    if ((!enable) || (frequencyHz == 0u)){
        g_motorStepEnabled = false;
        MOTOR_AllOutputsOff();
        return;
    }

    ticks = ((uint32_t)MOTOR_TIMER_ISR_HZ) / ((uint32_t)frequencyHz);

    if (ticks < 1u) ticks = 1u;
    if (ticks > 65535u) ticks = 65535u;

    g_motorTicksPerStep = (uint16_t)ticks;
    g_motorTickCounter = 0u;
    g_motorStepEnabled = true;

    MOTOR_ApplyCurrentStep();
#endif
}


static uint16_t MOTOR_ClampStepFrequencyHz(uint16_t frequencyHz) {
    if (frequencyHz > MOTOR_MAX_STEP_FREQUENCY_HZ) return MOTOR_MAX_STEP_FREQUENCY_HZ;
    if (frequencyHz == 0u) return 0u;
    if (frequencyHz < MOTOR_MIN_STEP_FREQUENCY_HZ) return MOTOR_MIN_STEP_FREQUENCY_HZ;
    return frequencyHz;
}

/*
 * Updates the public motor state based on enable and running flags.
 */
static void MOTOR_UpdateState(MOTOR_t *motor) {
    if (motor == 0) return;
    
    if(!motor->enabled) motor->state = MOTOR_STATE_DISABLED;
    else if(motor->running) motor->state = MOTOR_STATE_RUNNING;
    else motor->state = MOTOR_STATE_IDLE;
}

void MOTOR_Initialise(MOTOR_t *motor){
    if (motor == 0) return;
    
    motor->state = MOTOR_STATE_DISABLED;
    motor->direction = MOTOR_DEFAULT_DIRECTION;
    motor->stepFrequencyHz = 0u;
    motor->enabled = false;
    motor->running = false;
    
    g_motorStepEnabled = false;
    g_motorTicksPerStep = 1u;
    g_motorTickCounter = 0u;
    g_motorStepIndex = 0u;
    g_motorDirection = MOTOR_DEFAULT_DIRECTION;
    g_motorStepMode = MOTOR_STEP_MODE_HALF;

    MOTOR_AllOutputsOff();
      
    MOTOR_ApplyEnablePin(false);
    MOTOR_ApplyDirectionPin(motor->direction);
    MOTOR_ApplyStepGenerator(0u, false);
    MOTOR_UpdateState(motor);
}

/*
 * Timer interrupt service routine used to advance the motor
 * commutation sequence at the configured step frequency.
 */
void MOTOR_TimerISR(void) {
#if (MOTOR_ENABLE_SIMULATION == 1u)
    return;
#else
    if(!g_motorStepEnabled) return;
    
    g_motorTickCounter++;
    
    if (g_motorTickCounter >= g_motorTicksPerStep) {
        g_motorTickCounter = 0u;
        MOTOR_AdvanceStep();
    }
#endif
}

void MOTOR_Enable(MOTOR_t *motor){
    if (motor == 0) return;
    
    if(motor->enabled) {
        MOTOR_UpdateState(motor);
        return;
    }
    
    motor->enabled = true;
    motor->running = false;
    
    MOTOR_ApplyEnablePin(true);
    MOTOR_UpdateState(motor);
}

void MOTOR_Disable(MOTOR_t *motor) {
    if (motor == 0) return;
    
   /* Stop step generation and disable all motor outputs. */
    MOTOR_ApplyStepGenerator(0u, false);
    MOTOR_ApplyEnablePin(false);
    
    motor->enabled = false;
    motor->running = false;
    motor->stepFrequencyHz = 0u;
    
    MOTOR_UpdateState(motor);
}

bool MOTOR_IsEnabled(const MOTOR_t *motor) {
    if (motor == 0) return false;
    return motor->enabled;
}

void MOTOR_SetStepMode(MOTOR_t *motor, MOTOR_StepMode_t mode) {
    if(motor == 0) return;
    
    if ((mode != MOTOR_STEP_MODE_FULL) && (mode != MOTOR_STEP_MODE_HALF)) return;
    
    g_motorStepMode = mode;
    g_motorStepIndex = 0u;
    
    if (motor->running) MOTOR_ApplyCurrentStep();

}

void MOTOR_SetDirection(MOTOR_t *motor, MOTOR_Direction_t direction) {
    if (motor == 0) return;    
    
    if (motor->direction == direction) return;
    
    if (motor->running) return;
    
    motor->direction = direction;
    MOTOR_ApplyDirectionPin(direction);
}

MOTOR_Direction_t MOTOR_GetDirection(const MOTOR_t *motor) {
    if (motor == 0) return MOTOR_DIRECTION_POSITIVE;
    return motor->direction;
}

void MOTOR_SetStepFrequencyHz(MOTOR_t *motor, uint16_t frequencyHz){
    uint16_t clampedFrequencyHz;
    
    if (motor == 0) return;
    
    clampedFrequencyHz = MOTOR_ClampStepFrequencyHz(frequencyHz);
    
    if(motor->stepFrequencyHz == clampedFrequencyHz) return;
    
    motor->stepFrequencyHz = clampedFrequencyHz;

    if (motor->running) {
        if (motor->stepFrequencyHz == 0u) {
            MOTOR_ApplyStepGenerator(0u, false);
            motor->running = false;
            MOTOR_UpdateState(motor);
        }
        else MOTOR_ApplyStepGenerator(motor->stepFrequencyHz, true);
    }
}

uint16_t MOTOR_GetStepFrequencyHz(const MOTOR_t *motor) {
    if (motor == 0) return 0u;
    return motor->stepFrequencyHz;
}

/*
 * Starts motor motion using the configured direction and step frequency.
 */
void MOTOR_Start(MOTOR_t *motor){
    if (motor == 0) return;
    
    if(!motor->enabled) return;
    
    if(motor->stepFrequencyHz < MOTOR_MIN_STEP_FREQUENCY_HZ) return;
    
    if(motor->running) {
        MOTOR_UpdateState(motor);
        return;
    }
    
    /* Apply direction before starting step generation. */
    MOTOR_ApplyDirectionPin(motor->direction);
    MOTOR_ApplyStepGenerator(motor->stepFrequencyHz, true);
    
    motor->running = true;
    MOTOR_UpdateState(motor);  
}

void MOTOR_Stop(MOTOR_t *motor) {
    if (motor == 0) return;
    
    if (!motor->running) {
        MOTOR_UpdateState(motor);
        return;
    }
    
    /* Stop step generation while keeping the motor enabled. */
    MOTOR_ApplyStepGenerator(0u, false);
    motor->running = false;
    MOTOR_UpdateState(motor);
}

bool MOTOR_IsRunning(const MOTOR_t *motor) {
    if (motor == 0) return false;
    return motor->running;
}

MOTOR_State_t MOTOR_GetState(const MOTOR_t *motor){
    if (motor == 0) return MOTOR_STATE_FAULT;
    return motor->state;
}

void MOTOR_Task(MOTOR_t *motor) {
    if (motor == 0) return;
    MOTOR_UpdateState(motor);
}
