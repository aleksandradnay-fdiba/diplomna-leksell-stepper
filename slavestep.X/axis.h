/*
 * File: axis.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the axis-control module.
 * Defines the axis state machine data types, configuration parameters,
 * fault codes, homing states, and control functions used for positioning,
 * homing, stopping, and status monitoring.
 *
 * Created: March 14, 2026
 */

#ifndef AXIS_H
#define	AXIS_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "motor.h"
#include "encoder.h"

/* Axis operating states */
typedef enum {
    AXIS_STATE_UNINITIALISED = 0,
    AXIS_STATE_NOT_HOMED,
    AXIS_STATE_HOMING,
    AXIS_STATE_READY,
    AXIS_STATE_MOVING,
    AXIS_STATE_STOPPING,
    AXIS_STATE_EMERGENCY,
    AXIS_STATE_FAULT
} AXIS_State_t;

/* Result codes returned by axis-control functions */
typedef enum {
    AXIS_OK = 0,
    AXIS_ERROR_NOT_HOMED,
    AXIS_ERROR_BUSY,
    AXIS_ERROR_FAULT,
    AXIS_ERROR_INVALIDSTATE,
    AXIS_ERROR_NULL,
    AXIS_ERROR_OUT_OF_RANGE,
    AXIS_ERROR_EMERGENCY,
} AXIS_Result_t;

/* Fault codes used for axis diagnostics */
typedef enum {
    AXIS_FAULT_NONE = 0,
    AXIS_FAULT_ENCODER_UPDATE,
    AXIS_FAULT_ENCODER_INVALID,
    AXIS_FAULT_HOMING_TIMEOUT,
    AXIS_FAULT_INVALID_HOMING_STEP,
    AXIS_FAULT_INVALID_STATE,
    AXIS_FAULT_REFERENCE_STUCK,
} AXIS_Fault_t;

/* Internal states of the homing sequence */
typedef enum {
    AXIS_HOMING_IDLE = 0,
    AXIS_HOMING_SEEK_REFERENCE,
    AXIS_HOMING_RELEASE_FROM_REFERENCE,
    AXIS_HOMING_REAPPROACH_REFERENCE,
    AXIS_HOMING_ZERO_ENCODER,
    AXIS_HOMING_FINAL_BACKOFF,
    AXIS_HOMING_FINISH,
    AXIS_HOMING_WAIT_AFTER_SEEK,
    AXIS_HOMING_WAIT_AFTER_RELEASE
} AXIS_HomingStep_t;

/* Configurable axis parameters */
typedef struct {
    float minTravelMm;
    float maxTravelMm;
    float stepsPerMm;
    float positionToleranceMm;
    float fastZoneMm;
    
    uint16_t moveSpeedFastHz;
    uint16_t moveSpeedSlowHz;
    uint16_t homingSeekSpeedHz;
    uint16_t homingReleaseSpeedHz;
    uint16_t homingBackoffSpeedHz;
    
    float homingVerifyDistanceMm;
    float homingFinalBackoffMm;
    float homingMinSeekTravelMm;
    uint8_t homingRequiredDetections;
    uint32_t homingTimeoutMs;
    uint16_t homingSettleTicks;
    
    MOTOR_Direction_t positiveMoveDirection;
    MOTOR_Direction_t homeSeekDirection;

} AXIS_Config_t;

/* Runtime axis-control object */
typedef struct {
    AXIS_State_t state;
    AXIS_Fault_t faultCode;
    
    float currentPositionMm;
    float targetPositionMm;
    float positionDeltaMm;
    
    bool homed;
    bool emergencyActive;
    
    AXIS_HomingStep_t homingStep;
    uint32_t homingStartMs;
    uint8_t homingDetectionCount;
    
    float homingReleaseTargetMm;
    bool homingReleaseMotionVerified;
    
    float homingSeekStartPosMm;
    bool homingSeekMotionVerified;
    
    int32_t homingLastRawCount;
    uint32_t homingLastMotionMs;
    bool homingStallDetectionArmed;
    uint16_t homingNoMotionTicks;
    uint16_t homingWaitTicks;
            
    MOTOR_t *motor;
    ENCODER_t *encoder;
    
    AXIS_Config_t config;
} AXIS_t;

/* Initialization and configuration */
void AXIS_Initialise(AXIS_t *axis, MOTOR_t *motor, ENCODER_t *encoder);
void AXIS_LoadDefaultConfig(AXIS_Config_t *config);
AXIS_Result_t AXIS_SetConfig(AXIS_t *axis, const AXIS_Config_t *config);

/* Periodic state-machine execution */
void AXIS_Task(AXIS_t *axis);
void AXIS_TestForceHomed(AXIS_t *axis);

/* Motion-control commands */
AXIS_Result_t AXIS_Home(AXIS_t *axis);
AXIS_Result_t AXIS_MoveToMm(AXIS_t *axis, float positionMm);
AXIS_Result_t AXIS_MoveDeltaMm (AXIS_t *axis, float deltaMm);
AXIS_Result_t AXIS_Stop(AXIS_t *axis);
AXIS_Result_t AXIS_EmergencyStop(AXIS_t *axis);
AXIS_Result_t AXIS_ResetFault(AXIS_t *axis);

/* Position and state feedback */
float AXIS_GetCurrentPositionMm(const AXIS_t *axis);
float AXIS_GetTargetPositionMm(const AXIS_t *axis);
float AXIS_GetPositionDeltaMm (const AXIS_t *axis);

AXIS_State_t AXIS_GetState(const AXIS_t *axis);
AXIS_Fault_t AXIS_GetFaultCode(const AXIS_t *axis);
AXIS_HomingStep_t AXIS_GetHomingStep(const AXIS_t *axis);
uint8_t AXIS_GetHomingDetectionCount(const AXIS_t *axis);

bool AXIS_IsHomed(const AXIS_t *axis);
bool AXIS_IsBusy(const AXIS_t *axis);
bool AXIS_HasFault(const AXIS_t *axis);
bool AXIS_IsEmergencyActive(const AXIS_t *axis);

#endif	/* AXIS_H */