/*
 * File: master_axis_client.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements the master-side axis client used to send CAN commands
 * to slave axis nodes and process their response frames. The module
 * validates axis identifiers and commands, assigns sequence numbers,
 * retries command transmission, waits for acknowledgements, stores
 * the latest received axis status, and checks whether an axis is ready
 * for movement.
 *
 * Created: April 25, 2026
 */

#include "master_axis_client.h"
#include "uart_print.h"
#include "mcc_generated_files/mcc.h"
#include <stdint.h>

#define MASTER_AXIS_CLIENT_MAX_RETRIES 3u
#define MASTER_AXIS_CLIENT_ACK_WAIT_LOOPS 50u
#define MASTER_AXIS_CLIENT_ACK_DELAY_MS 10u

/* Status polling parameters used before sending movement commands. */
#define MASTER_AXIS_CLIENT_READY_WAIT_LOOPS     120u
#define MASTER_AXIS_CLIENT_READY_WAIT_DELAY_MS  250u

static uint8_t sequence = 0u;

/* Latest received status values for each configured axis */
static AXIS_State_t lastState[AXIS_ID_COUNT];
static AXIS_Result_t lastResult[AXIS_ID_COUNT];
static AXIS_Fault_t lastFault[AXIS_ID_COUNT];
static uint8_t lastFlags[AXIS_ID_COUNT];

/* Master-to-slave CAN identifiers indexed by AXIS_ID_t */
static const uint16_t masterToAxisId[AXIS_ID_COUNT] = {
    CAN_ID_MASTER_TO_AXIS_Y_L,
    CAN_ID_MASTER_TO_AXIS_Y_R,
    CAN_ID_MASTER_TO_AXIS_Z_L,
    CAN_ID_MASTER_TO_AXIS_Z_R,
};

/* Slave-to-master CAN identifiers indexed by AXIS_ID_t */
static const uint16_t axisToMasterId[AXIS_ID_COUNT] = {
    CAN_ID_AXIS_Y_L_TO_MASTER,
    CAN_ID_AXIS_Y_R_TO_MASTER,
    CAN_ID_AXIS_Z_L_TO_MASTER,
    CAN_ID_AXIS_Z_R_TO_MASTER
};

static bool MASTER_AXIS_CLIENT_IsAxisValid(AXIS_ID_t axisId) {
    return ((uint8_t)axisId < (uint8_t)AXIS_ID_COUNT);
}

static bool MASTER_AXIS_CLIENT_CommandMove(SLAVE_NODE_Command_t command) {
    return ((command == AXIS_NODE_CMD_MOVE_ABS_MM) ||
            (command == AXIS_NODE_CMD_MOVE_REL_MM));
}

static bool MASTER_AXIS_CLIENT_IsCommandValid(SLAVE_NODE_Command_t command) {
    switch (command) {
        case AXIS_NODE_CMD_HOME:
        case AXIS_NODE_CMD_STOP:
        case AXIS_NODE_CMD_EMSTOP:
        case AXIS_NODE_CMD_RESET:
        case AXIS_NODE_CMD_MOVE_ABS_MM:
        case AXIS_NODE_CMD_MOVE_REL_MM:
        case AXIS_NODE_CMD_GET_STATUS:
        case AXIS_NODE_CMD_SET_ENCODER_SCALE:
        case AXIS_NODE_CMD_SET_MAX_TRAVEL:
        case AXIS_NODE_CMD_SET_FAST_SPEED:
        case AXIS_NODE_CMD_SET_SLOW_SPEED:
        case AXIS_NODE_CMD_SET_POSITION_TOLERANCE:
        case AXIS_NODE_CMD_SAVE_CONFIG:
        return true;

        case AXIS_NODE_CMD_NONE:
        default:
            return false;
    }
}

static void MASTER_AXIS_CLIENT_UpdateFromResponse(AXIS_ID_t axisId, const CAN_PROTOCOL_ResponseFrame_t *response) {
    if (response == 0) return;
    if(!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return;

    lastResult[axisId] = response->result;
    lastState[axisId] = response->state;
    lastFault[axisId] = response->faultCode;
    lastFlags[axisId] = response->flags;
}

static void MASTER_AXIS_CLIENT_PrintResponse(const CAN_PROTOCOL_ResponseFrame_t *response) {
    send_string("ACK CMD=");
    send_int16_as_str_to_uart((int16_t)response->commandEcho);

    send_string(" SEQ=");
    send_int16_as_str_to_uart((int16_t)response->sequenceEcho);

    send_string(" RES=");
    send_int16_as_str_to_uart((int16_t)response->result);

    send_string(" STATE=");
    send_int16_as_str_to_uart((int16_t)response->state);

    send_string(" FAULT=");
    send_int16_as_str_to_uart((int16_t)response->faultCode);

    send_string(" FLAGS=");
    send_int16_as_str_to_uart((int16_t)response->flags);

    send_string("\r\n");
}

/*
 * Waits for a response frame from the selected axis and verifies
 * that the received sequence number matches the transmitted command.
 */
static bool MASTER_AXIS_CLIENT_WaitForAck(AXIS_ID_t axisId, uint8_t expectedSequence, CAN_PROTOCOL_ResponseFrame_t *response) {
    bool received;
    uint8_t waitCount;
    CAN_PROTOCOL_Result_t canResult;
    
    received = false;
    
    for (waitCount = 0u; waitCount < MASTER_AXIS_CLIENT_ACK_WAIT_LOOPS; waitCount++) {
        canResult = CAN_PROTOCOL_TryReadResponse(axisToMasterId[axisId], response, &received);
        
        if (canResult != CAN_PROTOCOL_OK) {
            send_string("RX ERR=");
            send_int16_as_str_to_uart((int16_t)canResult);
            send_string("\r\n");
        }
        
        if(received) {
            if (response->sequenceEcho == expectedSequence) {
                MASTER_AXIS_CLIENT_UpdateFromResponse(axisId, response);
                MASTER_AXIS_CLIENT_PrintResponse(response);
                return true;
            }
            send_string("ACK SEQ MISMATCH\r\n");
        }
    __delay_ms(MASTER_AXIS_CLIENT_ACK_DELAY_MS);
    }
    return false;
}


static bool MASTER_AXIS_CLIENT_IsReadyForMove(AXIS_ID_t axisId) {
    uint8_t flags;

    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return false;

    flags = lastFlags[axisId];

    if ((flags & CAN_STATUS_FLAG_HOMED) == 0u) return false;
    if ((flags & CAN_STATUS_FLAG_BUSY) != 0u) return false;
    if ((flags & CAN_STATUS_FLAG_FAULT) != 0u) return false;
    if ((flags & CAN_STATUS_FLAG_EMERGENCY) != 0u) return false;

    return true;
}

static bool MASTER_AXIS_CLIENT_RequestStatusOnce(AXIS_ID_t axisId) {
    CAN_PROTOCOL_CommandFrame_t commandFrame;
    CAN_PROTOCOL_ResponseFrame_t responseFrame;
    CAN_PROTOCOL_Result_t canResult;
    uint8_t thisSequence;
    
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return false;
    
    thisSequence = sequence++;
    
    commandFrame.command = AXIS_NODE_CMD_GET_STATUS;
    commandFrame.sequence = thisSequence;
    commandFrame.value = 0.0f;
    
    canResult = CAN_PROTOCOL_SendCommand(masterToAxisId[axisId], &commandFrame);
    
    if (canResult != CAN_PROTOCOL_OK) return false;
    if (MASTER_AXIS_CLIENT_WaitForAck(axisId, thisSequence, &responseFrame)) return true;
    
    return false;
}

/*
 * Polls the selected axis until it reports that it is homed, idle,
 * and free of fault or emergency conditions.
 */
static bool MASTER_AXIS_CLIENT_WaitUntilReadyForMove(AXIS_ID_t axisId) {
    uint8_t i;
    
    for(i = 0u; i < MASTER_AXIS_CLIENT_READY_WAIT_LOOPS; i++) {
        if(!MASTER_AXIS_CLIENT_RequestStatusOnce(axisId)) {
        __delay_ms(MASTER_AXIS_CLIENT_READY_WAIT_DELAY_MS);
        continue;
        }
        if (MASTER_AXIS_CLIENT_IsReadyForMove(axisId)) return true;
        
        __delay_ms(MASTER_AXIS_CLIENT_READY_WAIT_DELAY_MS);
    }
    return false;
}

void MASTER_AXIS_CLIENT_Init(void) {
    uint8_t id;
    sequence = 0u;
    for(id = 0u; id < AXIS_ID_COUNT; id++) {
        lastState[id] = AXIS_STATE_UNINITIALISED;
        lastResult[id] = AXIS_ERROR_INVALIDSTATE;
        lastFault[id] = AXIS_FAULT_NONE;
        lastFlags[id] = 0u;
    }
}

/*
 * Sends a validated command to the selected axis node.
 * Movement commands are transmitted only after the axis reports
 * a ready state through a status response.
 */
bool MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_t axisId, SLAVE_NODE_Command_t command, float value) {
    CAN_PROTOCOL_CommandFrame_t commandFrame;
    CAN_PROTOCOL_ResponseFrame_t responseFrame;
    CAN_PROTOCOL_Result_t canResult;
    uint8_t retry;
    uint8_t thisSequence;
    
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) {
        send_string("REJECT INVALID AXIS\r\n");
        return false;
    }
    
    if (!MASTER_AXIS_CLIENT_IsCommandValid(command)) {
        send_string("REJECT INVALID CMD\r\n");
        return false;
    }
    
    if (MASTER_AXIS_CLIENT_CommandMove(command)) {
        if(!MASTER_AXIS_CLIENT_WaitUntilReadyForMove(axisId)) {
            send_string("REJECT MOVE: AXIS NOT READY\r\n");
            return false;
        }
    }
    
    thisSequence = sequence++;
    
    commandFrame.command = command;
    commandFrame.sequence = thisSequence;
    commandFrame.value = value;
    
    for (retry = 0u; retry < MASTER_AXIS_CLIENT_MAX_RETRIES; retry++) {
        canResult = CAN_PROTOCOL_SendCommand(masterToAxisId[axisId], &commandFrame);
        
        if (canResult != CAN_PROTOCOL_OK) {
            send_string("TX FAIL=");
            send_int16_as_str_to_uart((int16_t)canResult);
            send_string("\r\n");
            continue;
        }
        send_string("TX AXIS=");
        send_int16_as_str_to_uart((int16_t)axisId);
        send_string(" CMD=");
        send_int16_as_str_to_uart((int16_t)command);
        send_string(" SEQ=");
        send_int16_as_str_to_uart((int16_t)thisSequence);
        send_string(" TRY=");
        send_int16_as_str_to_uart((int16_t)retry);
        send_string("\r\n");
        
        if (MASTER_AXIS_CLIENT_WaitForAck(axisId, thisSequence, &responseFrame)) {
            return true;
        }
        
        send_string("NO ACK RETRY\r\n");
    }
    send_string("COMMAND FAILED: NO ACK\r\n");
    return false;
}


void MASTER_AXIS_CLIENT_Task(void) {
}

bool MASTER_AXIS_CLIENT_IsBusy(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return false;
    return ((lastFlags[axisId] & CAN_STATUS_FLAG_BUSY) != 0u);
}

bool MASTER_AXIS_CLIENT_HasFault(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return true;
    return ((lastFlags[axisId] & CAN_STATUS_FLAG_FAULT) != 0u);
}

AXIS_State_t MASTER_AXIS_CLIENT_GetLastState(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return AXIS_STATE_FAULT;
    return lastState[axisId];
}

AXIS_Result_t MASTER_AXIS_CLIENT_GetLastResult(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return AXIS_ERROR_INVALIDSTATE;
    return lastResult[axisId];
}

AXIS_Fault_t MASTER_AXIS_CLIENT_GetLastFault(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return AXIS_FAULT_INVALID_STATE;
    return lastFault[axisId];
}

uint8_t MASTER_AXIS_CLIENT_GetLastFlags(AXIS_ID_t axisId) {
    if (!MASTER_AXIS_CLIENT_IsAxisValid(axisId)) return 0u;
    return lastFlags[axisId];
}