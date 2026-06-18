/*
 * File: axis.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements the high-level axis-control module of the distributed
 * motion-control system. The module manages axis initialization,
 * position control, homing execution, motion supervision, fault
 * handling, emergency-stop behavior, and encoder feedback processing.
 *
 * The axis controller operates as a state machine and coordinates
 * the motor and encoder modules to execute commanded movements
 * within configured travel limits.
 *
 * Created: March 14, 2026
 */

#include "axis.h"
#include "milliecond_timers.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define AXIS_TASK_PERIOD_SEC (0.01f)
/* Enables software-only axis simulation for testing purposes. */
#define AXIS_ENABLE_SIMULATION (0u)

#define AXIS_HOMING_STALL_TIMEOUT_MS      (200u)
#define AXIS_HOMING_STALL_MIN_COUNTS      (1)


/* Internal helper function declarations */
static float AXIS_AbsFloat(float value);
static bool AXIS_IsWithinTravelRange(const AXIS_t *axis, float positionMm);
static bool AXIS_IsConfigurationValid(const AXIS_Config_t *config);
static void AXIS_CommandMotor(AXIS_t *axis, MOTOR_Direction_t direction, uint16_t frequencyHz);
static void AXIS_StopMotor(AXIS_t *axis);
static void AXIS_SetFault(AXIS_t *axis, AXIS_Fault_t faultCode);
static void AXIS_UpdateFeedback(AXIS_t *axis);

static float AXIS_GetMotorSpeedMmPerSec(const AXIS_t *axis);
static MOTOR_Direction_t AXIS_GetDirectionForPositiveMove(const AXIS_t *axis);
static MOTOR_Direction_t AXIS_GetDirectionForNegativeMove(const AXIS_t *axis);
static MOTOR_Direction_t AXIS_GetOppositeDirection(MOTOR_Direction_t direction);
static float AXIS_GetHomingSeekTravelMm(const AXIS_t *axis);
static bool AXIS_IsEncoderMotionStopped(AXIS_t *axis);

static void AXIS_RunSimulation (AXIS_t *axis);

/* Homing state-machine handlers */
static void AXIS_HandleHoming(AXIS_t *axis);
static void AXIS_HandleHomingSeekReference(AXIS_t *axis) {
    float seekTravelMm;

    AXIS_CommandMotor(axis,
                  axis->config.homeSeekDirection,
                  axis->config.homingSeekSpeedHz);

    seekTravelMm = AXIS_GetHomingSeekTravelMm(axis);

    if (seekTravelMm > 0.05f) {
        axis->homingSeekMotionVerified = true;
    }

    if (AXIS_IsEncoderMotionStopped(axis)) {
        AXIS_StopMotor(axis);

        axis->homingDetectionCount = 0u;
        axis->homingReleaseMotionVerified = false;
        axis->homingSeekStartPosMm = axis->currentPositionMm;
        axis->homingNoMotionTicks = 0u;

        axis->homingStep = AXIS_HOMING_RELEASE_FROM_REFERENCE;
    }
}

/*
 * Original seek-reference implementation kept for comparison.
static void AXIS_HandleHomingSeekReference(AXIS_t *axis);
static void AXIS_HandleHomingReleaseFromReference(AXIS_t *axis);
static void AXIS_HandleHomingReapproachReference(AXIS_t *axis) {
    float seekTravelMm;

    AXIS_CommandMotor(axis,
                  axis->config.homeSeekDirection,
                  axis->config.homingSeekSpeedHz);

    seekTravelMm = AXIS_GetHomingSeekTravelMm(axis);

    if (seekTravelMm > 0.05f) {
        axis->homingSeekMotionVerified = true;
    }

    if (AXIS_IsEncoderMotionStopped(axis)) {
        AXIS_StopMotor(axis);

        axis->homingDetectionCount++;

        if (axis->homingDetectionCount >= axis->config.homingRequiredDetections) {
            axis->homingStep = AXIS_HOMING_ZERO_ENCODER;
        }
        else {
            axis->homingReleaseMotionVerified = false;
            axis->homingSeekStartPosMm = axis->currentPositionMm;
            axis->homingNoMotionTicks = 0u;

            axis->homingStep = AXIS_HOMING_RELEASE_FROM_REFERENCE;
        }
    }
}

/*
 * Original re-approach implementation kept for comparison.
static void AXIS_HandleHomingReapproachReference(AXIS_t *axis);
static void AXIS_HandleHomingZeroEncoder(AXIS_t *axis);
static void AXIS_HandleHomingFinalBackoff(AXIS_t *axis);
static void AXIS_HandleHomingFinish(AXIS_t *axis);
static bool AXIS_HomingWaitDone(AXIS_t *axis);

/* Public interface */

void AXIS_TestForceHomed(AXIS_t *axis)
{
    if ((axis == 0) || (axis->encoder == 0)) return;

    ENCODER_SetZero(axis->encoder);

    axis->currentPositionMm = 0.0f;
    axis->targetPositionMm = 0.0f;
    axis->positionDeltaMm = 0.0f;

    axis->homed = true;
    axis->faultCode = AXIS_FAULT_NONE;
    axis->emergencyActive = false;
    axis->homingStep = AXIS_HOMING_IDLE;
    axis->state = AXIS_STATE_READY;
}
 */

 */


void AXIS_LoadDefaultConfig(AXIS_Config_t *config) { //promeni ja sprjamo povedenieto
    if (config == 0) return;
   
    config->minTravelMm = 0.0f;
    config->maxTravelMm = 140.0f;
    config->stepsPerMm = 500.0f;
    config->positionToleranceMm = 0.30f;
    config->fastZoneMm = 5.0f;

    config->moveSpeedFastHz = 489u;
    config->moveSpeedSlowHz = 300u;
    
    config->homingSeekSpeedHz = 489u;
    config->homingReleaseSpeedHz = 489u;
    config->homingBackoffSpeedHz = 489u;

    config->homingVerifyDistanceMm = 1.0f;
    config->homingFinalBackoffMm = 1.0f;
    
    /* Minimum travel required before a stall condition may be
   interpreted as successful reference detection. */
    config->homingMinSeekTravelMm = 0.5f;
    config->homingRequiredDetections = 2u;
    config->homingTimeoutMs = 10000u;
    config->homingSettleTicks = 30u;

    config->positiveMoveDirection = MOTOR_DIRECTION_POSITIVE;
    config->homeSeekDirection = MOTOR_DIRECTION_NEGATIVE;
}

AXIS_Result_t AXIS_SetConfig(AXIS_t *axis, const AXIS_Config_t *config) {
    if ((axis == 0) || (config == 0)) return AXIS_ERROR_NULL;
    
    if (!AXIS_IsConfigurationValid(config)) return AXIS_ERROR_INVALIDSTATE;
    
    axis->config = *config;
    return AXIS_OK;
}

void AXIS_Initialise(AXIS_t *axis, MOTOR_t *motor, ENCODER_t *encoder){
    
    AXIS_Config_t defaultConfig;
    
    if ((axis == 0) || (motor == 0)||(encoder == 0)) return;
    
    AXIS_LoadDefaultConfig(&defaultConfig);
    
    axis->state = AXIS_STATE_NOT_HOMED;
    axis->faultCode = AXIS_FAULT_NONE;
    
    axis->currentPositionMm = 0.0f;
    axis->targetPositionMm = 0.0f;
    axis->positionDeltaMm = 0.0f;
    
    axis->homed = false;
    axis->emergencyActive = false;
    
    axis->homingStep = AXIS_HOMING_IDLE;
    axis->homingStartMs = 0u;
    axis->homingDetectionCount = 0u;
    axis->homingReleaseTargetMm = 0.0f;
    axis->homingReleaseMotionVerified = false;
    
    axis->homingLastRawCount = 0;
    axis->homingLastMotionMs = 0u;
    axis->homingStallDetectionArmed = true;
    axis->homingNoMotionTicks = 0u;
    axis->homingWaitTicks = 0u;
    
    axis->homingSeekStartPosMm = 0.0f;
    axis->homingSeekMotionVerified = false;
     
    axis->motor = motor;
    axis->encoder = encoder;
    axis->config = defaultConfig;
    
    MOTOR_Initialise(axis->motor);
    ENCODER_Initialise(axis->encoder);
 }

/*
 * Main axis state-machine execution function.
 * This function shall be called periodically and is responsible for
 * motion control, homing execution, fault supervision, and encoder
 * feedback processing.
 */
void AXIS_Task(AXIS_t *axis){
    float absDeltaMm;
    uint16_t moveFrequencyHz;
    
    if ((axis == 0) || (axis->motor == 0)||(axis->encoder == 0)) return;
    
  
    #if (AXIS_ENABLE_SIMULATION == 1u) 
    AXIS_RunSimulation(axis);
    #endif
    AXIS_UpdateFeedback(axis);
    
    if (axis->state == AXIS_STATE_FAULT) {
        AXIS_StopMotor(axis);
        return;
    }
    
    if (axis->emergencyActive) {
        AXIS_StopMotor(axis);
        axis->state = AXIS_STATE_EMERGENCY;
        return;
    }
      
    switch (axis->state) {
        case AXIS_STATE_UNINITIALISED: {
            AXIS_StopMotor(axis);
            break;
        }
        
        case AXIS_STATE_NOT_HOMED: {
            AXIS_StopMotor(axis);
            break;
        }
        
        case AXIS_STATE_HOMING: {
            AXIS_HandleHoming(axis);
            break;
        }
        
        case AXIS_STATE_READY: {
            AXIS_StopMotor(axis);
            break;
        }
        
        case AXIS_STATE_MOVING: { 
            axis->positionDeltaMm = axis->targetPositionMm - axis->currentPositionMm;
            absDeltaMm = AXIS_AbsFloat(axis->positionDeltaMm);
            
            if(absDeltaMm <= axis->config.positionToleranceMm){
                AXIS_StopMotor(axis);
                //axis->targetPositionMm = axis->currentPositionMm;
                axis->positionDeltaMm = 0.0f;
                axis->state = AXIS_STATE_READY;
                break;
            }
            
            MOTOR_Enable(axis->motor);
            
            if(absDeltaMm > axis->config.fastZoneMm) moveFrequencyHz = axis->config.moveSpeedFastHz;
            else moveFrequencyHz = axis->config.moveSpeedSlowHz;  
            
            if (axis->positionDeltaMm > 0.0f) {
                AXIS_CommandMotor(axis, AXIS_GetDirectionForPositiveMove(axis), moveFrequencyHz);
            } else {
                AXIS_CommandMotor(axis, AXIS_GetDirectionForNegativeMove(axis), moveFrequencyHz);
            }
            
            if(!MOTOR_IsRunning(axis->motor)) {
                MOTOR_Start(axis->motor);
            }
            
            break;  
        }
        
        case AXIS_STATE_STOPPING: {
            AXIS_StopMotor(axis);
            
            if(axis->homed) axis->state = AXIS_STATE_READY;
            else axis->state = AXIS_STATE_NOT_HOMED;
            
            axis->targetPositionMm = axis->currentPositionMm;
            axis->positionDeltaMm = 0.0f;
            axis->homingStep = AXIS_HOMING_IDLE;
            break;
        }
        
        case AXIS_STATE_EMERGENCY: {
            AXIS_StopMotor(axis);
            break;
        }
        
        case AXIS_STATE_FAULT: {
            AXIS_StopMotor(axis);
            break;
        }
        
        default: {
            AXIS_SetFault(axis, AXIS_FAULT_INVALID_STATE); 
            break;
        }
    }
}

/*
 * Starts the homing sequence used to establish the mechanical
 * reference position of the axis.
 */
AXIS_Result_t AXIS_Home(AXIS_t *axis) {
    if(axis == 0) return AXIS_ERROR_NULL;
    
    if(axis->emergencyActive) return AXIS_ERROR_EMERGENCY;
    
    if(axis->faultCode != AXIS_FAULT_NONE) return AXIS_ERROR_FAULT;
    
    if ((axis->state != AXIS_STATE_NOT_HOMED) && (axis->state != AXIS_STATE_READY)) {
        return AXIS_ERROR_BUSY;
    }
    
     axis->homed = false;
    axis->targetPositionMm = axis->currentPositionMm;
    axis->positionDeltaMm = 0.0f;
    
    axis->homingStep = AXIS_HOMING_SEEK_REFERENCE;
    axis->homingDetectionCount = 0u;
    
    axis->homingReleaseMotionVerified = false;
    axis->homingReleaseTargetMm = 0.0f;
    
    axis->homingStartMs = millis();
    
    axis->homingSeekStartPosMm = axis->currentPositionMm;
    axis->homingSeekMotionVerified = false;
    
    axis->homingLastRawCount = ENCODER_GetRawCount(axis->encoder);
    axis->homingLastMotionMs = millis();
    axis->homingStallDetectionArmed = false;
    axis->homingNoMotionTicks = 0u;
    axis->homingWaitTicks = 0u;
    
    axis->state = AXIS_STATE_HOMING;
    
    return AXIS_OK;
}

/*
 * Commands an absolute movement to the requested position
 * expressed in millimeters.
 */
AXIS_Result_t AXIS_MoveToMm(AXIS_t *axis, float positionMm) {
    if (axis == 0) return AXIS_ERROR_NULL;
    
    if (axis->emergencyActive) return AXIS_ERROR_EMERGENCY;
    
    if(axis->faultCode != AXIS_FAULT_NONE) return AXIS_ERROR_FAULT;
        
    if (!axis->homed) return AXIS_ERROR_NOT_HOMED;
    
    if (axis->state != AXIS_STATE_READY) return AXIS_ERROR_BUSY;
    
    if (!AXIS_IsWithinTravelRange(axis, positionMm)) return AXIS_ERROR_OUT_OF_RANGE;
    
    axis->targetPositionMm = positionMm;
    axis->positionDeltaMm = axis->targetPositionMm - axis->currentPositionMm;
    axis->state = AXIS_STATE_MOVING;
    
    return AXIS_OK; 
}

AXIS_Result_t AXIS_MoveDeltaMm(AXIS_t *axis, float deltaMm) {
    if (axis == 0) return AXIS_ERROR_NULL; 
    return AXIS_MoveToMm(axis, axis->currentPositionMm + deltaMm);
}

AXIS_Result_t AXIS_Stop(AXIS_t *axis) {
    if (axis == 0) return AXIS_ERROR_NULL;
    
    if(axis->emergencyActive) return AXIS_ERROR_EMERGENCY;
    
    if(axis->faultCode != AXIS_FAULT_NONE) return AXIS_ERROR_FAULT;
    
    if((axis->state == AXIS_STATE_MOVING) || (axis->state == AXIS_STATE_HOMING)) {
        axis->state = AXIS_STATE_STOPPING;
        return AXIS_OK;
    }
    return AXIS_ERROR_INVALIDSTATE;
}

/*
 * Immediately stops axis motion and places the controller
 * into the emergency-stop state.
 */
AXIS_Result_t AXIS_EmergencyStop(AXIS_t *axis) {
    if(axis == 0) return AXIS_ERROR_NULL;
    
    axis->emergencyActive = true;
    AXIS_StopMotor(axis);
    axis->state = AXIS_STATE_EMERGENCY;
    return AXIS_OK;
}

AXIS_Result_t AXIS_ResetFault(AXIS_t *axis) {
    if(axis == 0) return AXIS_ERROR_NULL;
    
    AXIS_StopMotor(axis);
    
    axis->faultCode = AXIS_FAULT_NONE;
    axis->emergencyActive = false;
    axis->homingStep = AXIS_HOMING_IDLE;
    axis->homingDetectionCount = 0u;
    axis->homingReleaseTargetMm = 0.0f;
    axis->homingReleaseMotionVerified = false;
    
    axis->targetPositionMm = axis->currentPositionMm;
    axis->positionDeltaMm = 0.0f;
    axis->homed = false;
    axis->state = AXIS_STATE_NOT_HOMED;
    
    axis->homingLastRawCount = 0;
    axis->homingLastMotionMs = 0u;
    axis->homingStallDetectionArmed = false;
    axis->homingNoMotionTicks = 0u;
    axis->homingWaitTicks = 0u;
    
    axis->homingSeekStartPosMm = axis->currentPositionMm;
    axis->homingSeekMotionVerified = false; 
    
    return AXIS_OK;
}

//GETTERS:
float AXIS_GetCurrentPositionMm(const AXIS_t *axis) {
    if (axis == 0) return 0.0f;
    return axis->currentPositionMm;
}

float AXIS_GetTargetPositionMm(const AXIS_t *axis) {
    if (axis == 0) return 0.0f;
    return axis->targetPositionMm;
}

float AXIS_GetPositionDeltaMm(const AXIS_t *axis) {
    if (axis == 0) return 0.0f;
    return axis->positionDeltaMm;
}

AXIS_State_t AXIS_GetState(const AXIS_t *axis) {
    if (axis == 0) return AXIS_STATE_FAULT;
    return axis->state;
}

AXIS_Fault_t AXIS_GetFaultCode(const AXIS_t *axis) {
    if (axis == 0) return AXIS_FAULT_INVALID_STATE;
    return axis->faultCode;
}

AXIS_HomingStep_t AXIS_GetHomingStep(const AXIS_t *axis) {
    if(axis == 0) return AXIS_HOMING_IDLE;
    return axis->homingStep;
}

uint8_t AXIS_GetHomingDetectionCount(const AXIS_t *axis) {
    if (axis == 0) return 0u;
    return axis->homingDetectionCount;
}

bool AXIS_IsHomed(const AXIS_t *axis) {
    if(axis == 0) return false;
    return axis->homed;
}
bool AXIS_IsBusy(const AXIS_t *axis) {
    if (axis == 0) return false;
    return ((axis->state == AXIS_STATE_HOMING) || (axis->state == AXIS_STATE_MOVING) || (axis->state == AXIS_STATE_STOPPING));
}

bool AXIS_HasFault(const AXIS_t *axis) {
    if (axis == 0) return true;
    return (axis->faultCode != AXIS_FAULT_NONE);
}

bool AXIS_IsEmergencyActive(const AXIS_t *axis) {
    if (axis == 0) return false;
    return axis->emergencyActive ;
}

//static helpers
static float AXIS_AbsFloat(float value) {
    if (value < 0.0f) return -value;
    return value;
}

static bool AXIS_IsWithinTravelRange(const AXIS_t *axis, float positionMm){
    if (axis == 0) return false;
    
    if (positionMm < axis->config.minTravelMm) return false;
    
    if (positionMm > axis->config.maxTravelMm) return false;
    
    return true;
}

static bool AXIS_IsConfigurationValid(const AXIS_Config_t *config) {
    
    if (config == 0) return false;
    
    if (config->stepsPerMm <= 0.0f) return false;
    
    if (config->positionToleranceMm <= 0.0f) return false;
    
    if (config->maxTravelMm <= config->minTravelMm) return false;
    
    if (config->homingVerifyDistanceMm <= 0.0f) return false;
    
    if (config->homingFinalBackoffMm <= 0.0f) return false;
    
    if (config->homingRequiredDetections == 0u) return false;
    
    if (config->homingTimeoutMs == 0u) return false;
    
    return true;
}

static void AXIS_CommandMotor(AXIS_t *axis, MOTOR_Direction_t direction, uint16_t frequencyHz) {
    if ((axis == 0) || (axis->motor == 0)) return;

    MOTOR_Enable(axis->motor);
    MOTOR_SetDirection(axis->motor, direction);
    MOTOR_SetStepFrequencyHz(axis->motor, frequencyHz);

    if (!MOTOR_IsRunning(axis->motor)) {
        MOTOR_Start(axis->motor);
    }
}

static void AXIS_StopMotor(AXIS_t *axis) {
    if ((axis == 0) || (axis->motor == 0)) return;
    MOTOR_Stop(axis->motor);
}

static void AXIS_SetFault (AXIS_t *axis, AXIS_Fault_t faultCode) {
    if(axis == 0) return;
    
    AXIS_StopMotor(axis);
    axis->faultCode = faultCode;
    axis->state = AXIS_STATE_FAULT;
}

static void AXIS_UpdateFeedback(AXIS_t *axis) {
    if((axis == 0) || (axis->encoder == 0)) return;
    
    if(ENCODER_Update(axis->encoder) != ENCODER_OK) {
        AXIS_SetFault(axis, AXIS_FAULT_ENCODER_UPDATE);
        return;
    }
    
    if (!ENCODER_IsValid(axis->encoder)) {
        AXIS_SetFault(axis, AXIS_FAULT_ENCODER_INVALID);
        return;
    }
    
    axis->currentPositionMm = ENCODER_GetPositionMm(axis->encoder);
    axis->positionDeltaMm = axis->targetPositionMm - axis->currentPositionMm;
}

static float AXIS_GetMotorSpeedMmPerSec(const AXIS_t *axis) {
    if ((axis == 0) || (axis->motor == 0)) {
        return 0.0f;
    }
    if(axis->config.stepsPerMm <= 0.0f) return 0.0f;
    
    return ((float)MOTOR_GetStepFrequencyHz(axis->motor)) / axis->config.stepsPerMm;
}

static MOTOR_Direction_t AXIS_GetDirectionForPositiveMove (const AXIS_t *axis) {
    if(axis->config.positiveMoveDirection == MOTOR_DIRECTION_POSITIVE) {
        return MOTOR_DIRECTION_POSITIVE;
    }
   return MOTOR_DIRECTION_NEGATIVE;
}

static MOTOR_Direction_t AXIS_GetDirectionForNegativeMove (const AXIS_t *axis) {
    if(axis->config.positiveMoveDirection == MOTOR_DIRECTION_POSITIVE) {
        return MOTOR_DIRECTION_NEGATIVE;
    }
   return MOTOR_DIRECTION_POSITIVE;
}

static MOTOR_Direction_t AXIS_GetOppositeDirection(MOTOR_Direction_t direction) {
    if (direction == MOTOR_DIRECTION_POSITIVE) {
        return MOTOR_DIRECTION_NEGATIVE;
    }
    return MOTOR_DIRECTION_POSITIVE;
}

static float AXIS_GetHomingSeekTravelMm(const AXIS_t *axis) {
    if(axis == 0) return 0.0f;
    
    return AXIS_AbsFloat(axis->currentPositionMm - axis->homingSeekStartPosMm);
}

static bool AXIS_IsEncoderMotionStopped(AXIS_t *axis) {
    int32_t currentRaw;
    int32_t deltaRaw;
    
    if((axis == 0) || (axis->encoder == 0)) return false;
    
    currentRaw = ENCODER_GetRawCount(axis->encoder);
    deltaRaw = currentRaw - axis->homingLastRawCount;
    
    if (deltaRaw < 0) deltaRaw = -deltaRaw;
    
    if (deltaRaw >= AXIS_HOMING_STALL_MIN_COUNTS) {
        axis->homingLastRawCount = currentRaw;
        axis->homingLastMotionMs = millis();
        axis->homingNoMotionTicks = 0u;
        return false;
    }
     axis->homingNoMotionTicks++;
    
    if (axis->homingNoMotionTicks >= 20u) return true;
    
    return false;
}

static void AXIS_RunSimulation(AXIS_t *axis) {
    float speedMmPerSec;
    float deltaMoveMm;
    float newPositionMm;

    if ((axis == 0) || (axis->motor == 0) || (axis->encoder == 0)) return;
    
    if (!MOTOR_IsRunning(axis->motor)) return;
    
    speedMmPerSec = AXIS_GetMotorSpeedMmPerSec(axis);
    deltaMoveMm = speedMmPerSec * AXIS_TASK_PERIOD_SEC;
    newPositionMm = axis->currentPositionMm;

    if (MOTOR_GetDirection(axis->motor) == AXIS_GetDirectionForPositiveMove(axis)) {
        newPositionMm += deltaMoveMm;
    }
    else newPositionMm -= deltaMoveMm;
    //SIMULATED HOME
    if (newPositionMm < axis->config.minTravelMm) {
        newPositionMm = axis->config.minTravelMm;
    }

    if (newPositionMm > axis->config.maxTravelMm) {
        newPositionMm = axis->config.maxTravelMm;
    }

    ENCODER_SetPositionMm(axis->encoder, newPositionMm);
    
}

/*
 * Homing sequence:
 * 1. Seek reference position.
 * 2. Release from reference.
 * 3. Re-approach reference.
 * 4. Zero encoder position.
 * 5. Apply final backoff distance.
 * 6. Enter READY state.
 */
static void AXIS_HandleHoming(AXIS_t *axis) {
    if (axis == 0) return;
    
    if ((millis() - axis->homingStartMs) >= axis->config.homingTimeoutMs) {
        AXIS_SetFault(axis, AXIS_FAULT_HOMING_TIMEOUT);
        return;
    }
    
    switch (axis->homingStep){
        case AXIS_HOMING_SEEK_REFERENCE: {
            AXIS_HandleHomingSeekReference(axis);
            break;
        }
        
        case AXIS_HOMING_RELEASE_FROM_REFERENCE: {
            AXIS_HandleHomingReleaseFromReference(axis);
            break;
        }
    
        case AXIS_HOMING_REAPPROACH_REFERENCE: {
            AXIS_HandleHomingReapproachReference(axis);
            break;
        }
    
        case AXIS_HOMING_ZERO_ENCODER: {
            AXIS_HandleHomingZeroEncoder(axis);
            break;
        }
        
        case AXIS_HOMING_WAIT_AFTER_SEEK: {
            if (AXIS_HomingWaitDone(axis)) {
            axis->homingReleaseMotionVerified = false;
            axis->homingSeekStartPosMm = axis->currentPositionMm;
            axis->homingNoMotionTicks = 0u;
            axis->homingStep = AXIS_HOMING_RELEASE_FROM_REFERENCE;
            }
            break;
        }

        case AXIS_HOMING_WAIT_AFTER_RELEASE: {
            if (AXIS_HomingWaitDone(axis)) {
            axis->homingSeekStartPosMm = axis->currentPositionMm;
            axis->homingSeekMotionVerified = false;
            axis->homingLastRawCount = ENCODER_GetRawCount(axis->encoder);
            axis->homingLastMotionMs = millis();
            axis->homingNoMotionTicks = 0u;
            axis->homingStallDetectionArmed = false;
            axis->homingStep = AXIS_HOMING_REAPPROACH_REFERENCE;
            }
            break;
        }
    
        case AXIS_HOMING_FINAL_BACKOFF: {
            AXIS_HandleHomingFinalBackoff(axis);
            break;
        }
        
        case AXIS_HOMING_FINISH: {
            AXIS_HandleHomingFinish(axis);
            break;
        }
        
        case AXIS_HOMING_IDLE:
        default:
        {
            AXIS_SetFault(axis, AXIS_FAULT_INVALID_HOMING_STEP);
            break;
        }
    }
}

static void AXIS_HandleHomingSeekReference(AXIS_t *axis) {
    float seekTravelMm;
    
    AXIS_CommandMotor(axis,
                  axis->config.homeSeekDirection,
                  axis->config.homingSeekSpeedHz);
    
     
    seekTravelMm = AXIS_GetHomingSeekTravelMm(axis);
    
    if(seekTravelMm > axis->config.positionToleranceMm) {
        axis->homingSeekMotionVerified = true;
        axis->homingStallDetectionArmed = true;
    }
    
    if (axis->homingStallDetectionArmed) {
        if (AXIS_IsEncoderMotionStopped(axis)) {
            if(!axis->homingSeekMotionVerified) {
                AXIS_SetFault(axis, AXIS_FAULT_REFERENCE_STUCK);
                return;
          }
        
            if(seekTravelMm < axis->config.homingMinSeekTravelMm) {
                AXIS_SetFault(axis, AXIS_FAULT_REFERENCE_STUCK);
                return;
            }
        
            AXIS_StopMotor(axis);
        
            axis->homingDetectionCount = 0u;
            axis->homingReleaseMotionVerified = false;
           axis->homingSeekStartPosMm = axis->currentPositionMm;
            
           axis->homingStep = AXIS_HOMING_RELEASE_FROM_REFERENCE;
        }
    }
}

static void AXIS_HandleHomingReleaseFromReference(AXIS_t *axis) {
    float releaseTravelMm;

    AXIS_CommandMotor(axis,
                  AXIS_GetOppositeDirection(axis->config.homeSeekDirection),
                  axis->config.homingReleaseSpeedHz);

    releaseTravelMm = AXIS_AbsFloat(axis->currentPositionMm - axis->homingSeekStartPosMm);

    if (releaseTravelMm > axis->config.positionToleranceMm) {
        axis->homingReleaseMotionVerified = true;
    }

    if (releaseTravelMm >= axis->config.homingVerifyDistanceMm) {
        AXIS_StopMotor(axis);

        if (!axis->homingReleaseMotionVerified) {
            AXIS_SetFault(axis, AXIS_FAULT_REFERENCE_STUCK);
            return;
        }

        axis->homingSeekStartPosMm = axis->currentPositionMm;
        axis->homingSeekMotionVerified = false;

        axis->homingLastRawCount = ENCODER_GetRawCount(axis->encoder);
        axis->homingLastMotionMs = millis();
        axis->homingStallDetectionArmed = false;
        axis->homingNoMotionTicks = 0u;

        axis->homingStep = AXIS_HOMING_REAPPROACH_REFERENCE;
    }
}

static void AXIS_HandleHomingReapproachReference(AXIS_t *axis) {
    
    float seekTravelMm;
    
    AXIS_CommandMotor(axis,
                  axis->config.homeSeekDirection,
                  axis->config.homingSeekSpeedHz);
    
    seekTravelMm = AXIS_GetHomingSeekTravelMm(axis);
    
     if (seekTravelMm > axis->config.positionToleranceMm) {
        axis->homingSeekMotionVerified = true;
        axis->homingStallDetectionArmed = true;
    }
    
    if (axis->homingStallDetectionArmed) {
        if (AXIS_IsEncoderMotionStopped(axis)) {
            if (!axis->homingSeekMotionVerified) {
                AXIS_SetFault(axis, AXIS_FAULT_REFERENCE_STUCK);
                return;
            }
         
            if (seekTravelMm < axis->config.homingMinSeekTravelMm) {
                AXIS_SetFault(axis, AXIS_FAULT_REFERENCE_STUCK);
                return;
            }
         
            AXIS_StopMotor(axis);
        
            axis->homingDetectionCount++;
        
            if (axis->homingDetectionCount >= axis->config.homingRequiredDetections) {
                axis->homingStep = AXIS_HOMING_ZERO_ENCODER;
            }
            else {
                axis->homingReleaseMotionVerified = false;
                axis->homingSeekStartPosMm = axis->currentPositionMm;
                
                axis->homingStep = AXIS_HOMING_RELEASE_FROM_REFERENCE;
            }  
        }
    }
}

static void AXIS_HandleHomingZeroEncoder(AXIS_t *axis) {
    ENCODER_SetZero(axis->encoder);

    axis->currentPositionMm = 0.0f;

    axis->homingSeekStartPosMm = axis->currentPositionMm;
    axis->targetPositionMm = axis->config.homingFinalBackoffMm;
    axis->positionDeltaMm = axis->targetPositionMm - axis->currentPositionMm;
    
    axis->homingNoMotionTicks = 0u;
    axis->homingStallDetectionArmed = false;

    axis->homingStep = AXIS_HOMING_FINAL_BACKOFF;
}

static void AXIS_HandleHomingFinalBackoff(AXIS_t *axis) {
    float backoffTravelMm;

    if (axis->config.homingFinalBackoffMm <= axis->config.positionToleranceMm) {
        axis->homingStep = AXIS_HOMING_FINISH;
        return;
    }

    AXIS_CommandMotor(axis,
                  AXIS_GetOppositeDirection(axis->config.homeSeekDirection),
                  axis->config.homingBackoffSpeedHz);

    backoffTravelMm =
        AXIS_AbsFloat(axis->currentPositionMm - axis->homingSeekStartPosMm);

    if (backoffTravelMm >= axis->config.homingFinalBackoffMm) {
        AXIS_StopMotor(axis);
        axis->homingStep = AXIS_HOMING_FINISH;
    }
}

static void AXIS_HandleHomingFinish(AXIS_t *axis) {
    axis->currentPositionMm = ENCODER_GetPositionMm(axis->encoder);
    axis->targetPositionMm = axis->currentPositionMm;
    axis->positionDeltaMm = 0.0f;

    axis->homed = true;
    axis->homingStep = AXIS_HOMING_IDLE;
    axis->state = AXIS_STATE_READY;
}

static bool AXIS_HomingWaitDone(AXIS_t *axis) {
    if (axis == 0) return false;

    if (axis->homingWaitTicks > 0u) {
        axis->homingWaitTicks--;
        AXIS_StopMotor(axis);
        return false;
    }

    return true;
}