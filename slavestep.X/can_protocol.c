/*
 * File: can_protocol.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements the CAN communication protocol used between the master
 * controller and the slave axis nodes. The module provides functions
 * for building, transmitting, receiving, and decoding command and
 * response frames.
 *
 * Command frames contain the requested slave-node command, sequence
 * number, and command value. Response frames return the executed
 * command, sequence number, axis result, axis state, fault code, and
 * status flags.
 *
 * Created: April 25, 2026
 */

#include "can_protocol.h"

/*
 * Converts a floating-point command value into four CAN data bytes.
 */
static void CAN_PROTOCOL_FloatToBytes(float value, uint8_t *bytes) {
    union {
    float f;
    uint8_t b[4];
    } converter;
    
    converter.f = value;
    
    bytes[0] = converter.b[0];
    bytes[1] = converter.b[1];
    bytes[2] = converter.b[2];
    bytes[3] = converter.b[3]; 
}

/*
 * Reconstructs a floating-point value from four CAN data bytes.
 */
static float CAN_PROTOCOL_BytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    union { 
    float f;
    uint8_t b[4];
    } converter;
        
    converter.b[0] = b0;
    converter.b[1] = b1;
    converter.b[2] = b2;
    converter.b[3] = b3;
    
    return converter.f;
}

/*
 * Builds the compact status flag byte from the current slave-node status.
 */
static uint8_t CAN_PROTOCOL_BuildFlags(const SLAVE_NODE_Status_t *status) {
    uint8_t flags = 0u;
    
    if (status == 0) return 0u;
    
    if (status->homed) flags |= CAN_STATUS_FLAG_HOMED;
    if (status->busy) flags |= CAN_STATUS_FLAG_BUSY;
    if (status->faultActive) flags |= CAN_STATUS_FLAG_FAULT;
    if (status->emergencyActive) flags |= CAN_STATUS_FLAG_EMERGENCY;
    
    return flags;
}

/*
 * Sends a master-to-slave command frame.
 */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_SendCommand (uint16_t canId, 
                                                const CAN_PROTOCOL_CommandFrame_t *command) {
    uCAN_MSG msg;
    uint8_t valueBytes[4];
    
    if (command == 0) return CAN_PROTOCOL_ERROR_NULL;
    
    CAN_PROTOCOL_FloatToBytes(command->value, valueBytes);
    
    msg.frame.id = canId;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = CAN_DLC_COMMAND;
    
    msg.frame.data0 = (uint8_t)command->command;
    msg.frame.data1 = command->sequence;
    msg.frame.data2 = valueBytes[0];
    msg.frame.data3 = valueBytes[1];
    msg.frame.data4 = valueBytes[2];
    msg.frame.data5 = valueBytes[3];
    msg.frame.data6 = 0u;
    msg.frame.data7 = 0u;
    
    if (CAN_transmit(&msg) == 0u) return CAN_PROTOCOL_ERROR_TX_BUSY;
    
    return CAN_PROTOCOL_OK;
}

/*
 * Attempts to read a command frame with the expected CAN identifier.
 * If no matching frame is available, received remains false.
 */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_TryReadCommand (uint16_t expectedCanId,
                                                    CAN_PROTOCOL_CommandFrame_t *command,
                                                     bool *received) {
    uCAN_MSG msg;
    
    if ((command == 0) || (received == 0)) return CAN_PROTOCOL_ERROR_NULL;
    
    *received = false;
    
    if (CAN_receive(&msg) == 0u) return CAN_PROTOCOL_OK;
    
    if (msg.frame.id != expectedCanId) return CAN_PROTOCOL_OK;
    
    if (msg.frame.dlc != CAN_DLC_COMMAND) return CAN_PROTOCOL_ERROR_DLC;
    
    command->command = (SLAVE_NODE_Command_t)msg.frame.data0;
    command->sequence = msg.frame.data1;
    command->value = CAN_PROTOCOL_BytesToFloat(msg.frame.data2, msg.frame.data3, msg.frame.data4, msg.frame.data5);
    
    *received = true;
    return CAN_PROTOCOL_OK;
}

/*
 * Sends a slave-to-master response frame.
 */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_SendResponse (uint16_t canId,
                                                    const CAN_PROTOCOL_ResponseFrame_t *response) {
    uCAN_MSG msg;
    
    if (response == 0) return CAN_PROTOCOL_ERROR_NULL;
    
    msg.frame.id = canId;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = CAN_DLC_RESPONSE;
    
    msg.frame.data0 = (uint8_t)response->commandEcho;
    msg.frame.data1 = response->sequenceEcho;
    msg.frame.data2 = (uint8_t)response->result;
    msg.frame.data3 = (uint8_t)response->state;
    msg.frame.data4 = (uint8_t)response->faultCode;
    msg.frame.data5 = response->flags;
    msg.frame.data6 = 0u;
    msg.frame.data7 = 0u;
    
    if (CAN_transmit(&msg) == 0u) return CAN_PROTOCOL_ERROR_TX_BUSY;
    
    return CAN_PROTOCOL_OK;
}

/*
 * Attempts to read a response frame with the expected CAN identifier.
 * If no matching frame is available, received remains false.
 */
CAN_PROTOCOL_Result_t CAN_PROTOCOL_TryReadResponse (uint16_t expectedCanId,
                                                    CAN_PROTOCOL_ResponseFrame_t *response,
                                                     bool *receved) {
    uCAN_MSG msg;
    
    if ((response == 0) || (receved == 0)) return CAN_PROTOCOL_ERROR_NULL;
    
    *receved = false;
    
    if (CAN_receive(&msg) == 0u) return CAN_PROTOCOL_OK;
    
    if (msg.frame.id != expectedCanId) return CAN_PROTOCOL_OK;
    
    if (msg.frame.dlc != CAN_DLC_RESPONSE) return CAN_PROTOCOL_ERROR_DLC;
    
    response->commandEcho = (SLAVE_NODE_Command_t)msg.frame.data0;
    response->sequenceEcho = msg.frame.data1;
    response->result = (AXIS_Result_t)msg.frame.data2;
    response->state = (AXIS_State_t)msg.frame.data3;
    response->faultCode = (AXIS_Fault_t)msg.frame.data4;
    response->flags = msg.frame.data5;
    
    *receved = true;
    return CAN_PROTOCOL_OK;
}

/*
 * Builds a response frame from the command echo, execution result,
 * and current slave-node status.
 */
CAN_PROTOCOL_ResponseFrame_t CAN_PROTOCOL_BuildResponse (SLAVE_NODE_Command_t commandEcho,
                                                         uint8_t sequenceEcho,
                                                         AXIS_Result_t result,
                                                         const SLAVE_NODE_Status_t *status) {
    CAN_PROTOCOL_ResponseFrame_t response;
    
    response.commandEcho = commandEcho;
    response.sequenceEcho = sequenceEcho;
    response.result = result;
    
    if (status != 0) {
        response.state = status->state;
        response.faultCode = status->faultCode;
        response.flags = CAN_PROTOCOL_BuildFlags(status);
    } else {
        response.state = AXIS_STATE_FAULT;
        response.faultCode = AXIS_FAULT_INVALID_STATE;
        response.flags = CAN_STATUS_FLAG_FAULT;    
    }
    return response;
}