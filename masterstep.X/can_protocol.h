/*
 * File: can_protocol.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the CAN communication protocol layer.
 * Defines CAN identifiers, message formats, status flags, and
 * communication services used for command and response exchange
 * between the master controller and slave axis nodes.
 *
 * Created: April 25, 2026
 */

#ifndef CAN_PROTOCOL_H
#define	CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "mcc_generated_files/ecan.h"
#include "slave_node.h"

/* Master-to-slave CAN identifiers */
#define CAN_ID_MASTER_TO_AXIS_Y_L 0x101u
#define CAN_ID_MASTER_TO_AXIS_Y_R 0x102u
#define CAN_ID_MASTER_TO_AXIS_Z_L 0x103u
#define CAN_ID_MASTER_TO_AXIS_Z_R 0x104u

/* Slave-to-master CAN identifiers */
#define CAN_ID_AXIS_Y_L_TO_MASTER 0x181u
#define CAN_ID_AXIS_Y_R_TO_MASTER 0x182u
#define CAN_ID_AXIS_Z_L_TO_MASTER 0x183u
#define CAN_ID_AXIS_Z_R_TO_MASTER 0x184u

/* CAN frame lengths */
#define CAN_DLC_COMMAND 8u
#define CAN_DLC_RESPONSE 8u

/* CAN protocol return codes */
typedef enum {
            CAN_PROTOCOL_OK = 0,
            CAN_PROTOCOL_ERROR_NULL,
            CAN_PROTOCOL_ERROR_DLC,
            CAN_PROTOCOL_ERROR_ID,
            CAN_PROTOCOL_ERROR_TX_BUSY
} CAN_PROTOCOL_Result_t;

/* Master-to-slave command frame */
typedef struct {
    SLAVE_NODE_Command_t command;
    uint8_t sequence;
    float value;
} CAN_PROTOCOL_CommandFrame_t;

/* Slave-to-master response frame */
typedef struct {
    SLAVE_NODE_Command_t commandEcho;
    uint8_t sequenceEcho;
    AXIS_Result_t result;
    AXIS_State_t state;
    AXIS_Fault_t faultCode;
    uint8_t flags;
}   CAN_PROTOCOL_ResponseFrame_t;

/* Axis status flags */
#define CAN_STATUS_FLAG_HOMED (1u << 0)
#define CAN_STATUS_FLAG_BUSY (1u << 1)
#define CAN_STATUS_FLAG_FAULT (1u << 2)
#define CAN_STATUS_FLAG_EMERGENCY (1u << 3)

/* Command frame services */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_SendCommand (uint16_t canId, 
                                                const CAN_PROTOCOL_CommandFrame_t *command);

CAN_PROTOCOL_Result_t CAN_PROTOCOL_TryReadCommand (uint16_t expectedCanId,
                                                    CAN_PROTOCOL_CommandFrame_t *command,
                                                     bool *received);

/* Response frame services */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_SendResponse (uint16_t canId,
                                                    const CAN_PROTOCOL_ResponseFrame_t *response);

CAN_PROTOCOL_Result_t CAN_PROTOCOL_TryReadResponse (uint16_t expectedCanId,
                                                    CAN_PROTOCOL_ResponseFrame_t *response,
                                                     bool *receved);

/* Response frame construction */
CAN_PROTOCOL_ResponseFrame_t CAN_PROTOCOL_BuildResponse (SLAVE_NODE_Command_t commandEcho,
                                                         uint8_t sequenceEcho,
                                                         AXIS_Result_t result,
                                                         const SLAVE_NODE_Status_t *status);

#endif	/* CAN_PROTOCOL_H */