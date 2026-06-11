/*
 * File: slave_node.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the slave-node module.
 * Defines the command set, status structure, runtime slave-node object,
 * and functions used to execute commands and retrieve axis status
 * information within the distributed CAN architecture.
 *
 * Created: April 11, 2026
 */

#ifndef SLAVE_NODE_H
#define	SLAVE_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "axis.h"

/* Slave-node command set */
typedef enum {
    AXIS_NODE_CMD_NONE = 0,
    AXIS_NODE_CMD_HOME,
    AXIS_NODE_CMD_STOP,
    AXIS_NODE_CMD_EMSTOP,
    AXIS_NODE_CMD_RESET,
    AXIS_NODE_CMD_MOVE_ABS_MM,
    AXIS_NODE_CMD_MOVE_REL_MM,
    AXIS_NODE_CMD_GET_STATUS,
    //NEW COMMANDS ADDED FOR EEPROM
    AXIS_NODE_CMD_SET_ENCODER_SCALE,
    AXIS_NODE_CMD_SET_MAX_TRAVEL,
    AXIS_NODE_CMD_SAVE_CONFIG,
    AXIS_NODE_CMD_SET_FAST_SPEED,
    AXIS_NODE_CMD_SET_SLOW_SPEED,
    AXIS_NODE_CMD_SET_POSITION_TOLERANCE,
} SLAVE_NODE_Command_t;

/* Slave-node status information */
typedef struct {
    AXIS_State_t state;
    AXIS_Fault_t faultCode;
    AXIS_HomingStep_t homingStep;
    
    float currentPositionMm;
    float targetPositionMm;
    float positionDeltaMm;
    
    uint8_t homingDetectionCount;
    
    bool homed;
    bool busy;
    bool emergencyActive;
    bool faultActive;
} SLAVE_NODE_Status_t;


/* Runtime slave-node object */
typedef struct {
    AXIS_t *axis;
} SLAVE_NODE_t;

/* Initialization */
void SLAVE_NODE_Initialise(SLAVE_NODE_t *node, AXIS_t *axis);

/* Command execution */
AXIS_Result_t SLAVE_NODE_ExecuteCommand(SLAVE_NODE_t *node,
                                        SLAVE_NODE_Command_t command,
                                        float value);

/* Status acquisition */
void SLAVE_NODE_GetStatus(const SLAVE_NODE_t *node, SLAVE_NODE_Status_t *status);

#endif	/* SLAVE_NODE_H */

