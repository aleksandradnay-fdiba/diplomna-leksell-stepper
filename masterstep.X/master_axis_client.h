/*
 * File: master_axis_client.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the master axis client module.
 * Provides functions for sending commands to slave axis nodes,
 * monitoring command execution status, and accessing the latest
 * state, fault, and status information received from each axis.
 *
 * Created: April 25, 2026
 */

#ifndef MASTER_AXIS_CLIENT_H
#define	MASTER_AXIS_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "can_protocol.h"

/* Configured axis identifiers */
typedef enum {
    AXIS_ID_Y_L = 0,
    AXIS_ID_Y_R,
    AXIS_ID_Z_L,
    AXIS_ID_Z_R,
    AXIS_ID_COUNT
} AXIS_ID_t;

/* Initialization */
void MASTER_AXIS_CLIENT_Init(void);

/* Command transmission */
bool MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_t axisId, SLAVE_NODE_Command_t command, float value);
void MASTER_AXIS_CLIENT_Task(void);

/* Status monitoring */
bool MASTER_AXIS_CLIENT_IsBusy(AXIS_ID_t axisId);
bool MASTER_AXIS_CLIENT_HasFault(AXIS_ID_t axisId);

/* Last received axis information */
AXIS_State_t MASTER_AXIS_CLIENT_GetLastState(AXIS_ID_t axisId);
AXIS_Result_t MASTER_AXIS_CLIENT_GetLastResult(AXIS_ID_t axisId);
AXIS_Fault_t MASTER_AXIS_CLIENT_GetLastFault(AXIS_ID_t axisId);
uint8_t MASTER_AXIS_CLIENT_GetLastFlags(AXIS_ID_t axisId);
#endif	/* MASTER_AXIS_CLIENT_H */

    