/*
 * File: app_slave.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements the main slave-node application layer. The module initializes
 * the motor, encoder, axis-control, EEPROM configuration, and CAN slave-node
 * modules. During runtime, it executes the axis task, receives commands from
 * the master node, applies configuration commands, and sends status responses.
 *
 * Created: June 3, 2026
 */

#include "app_slave.h"
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/tmr2.h"

#include "can_protocol.h"
#include "app_config.h"
#include "slave_node.h"

#include "axis.h"
#include "motor.h"
#include "encoder.h"

#include <stdint.h>
#include <stdbool.h>

#define OK_LED_ON()       IO_RA4_SetHigh()
#define OK_LED_OFF()      IO_RA4_SetLow()

#define ERR_LED_ON()      IO_RA5_SetHigh()
#define ERR_LED_OFF()     IO_RA5_SetLow()

static AXIS_t axis;
static MOTOR_t motor;
static ENCODER_t encoder;
static SLAVE_NODE_t slaveNode;

/* Provides a short visual indication for successful CAN response transmission. */
static void SLAVE_APP_PulseOkLed(void) {
    OK_LED_ON();
    __delay_ms(5);
    OK_LED_OFF();
}

/* Provides a short visual indication for communication or command errors. */
static void SLAVE_APP_PulseErrLed(void) {
    ERR_LED_ON();
    __delay_ms(5);
    ERR_LED_OFF();
}

static void SLAVE_APP_SendResponse(SLAVE_NODE_Command_t commandEcho,
                                   uint8_t sequenceEcho,
                                   AXIS_Result_t axisResult) {
    SLAVE_NODE_Status_t status;
    CAN_PROTOCOL_ResponseFrame_t response;
    CAN_PROTOCOL_Result_t canResult;

    SLAVE_NODE_GetStatus(&slaveNode, &status);

    response = CAN_PROTOCOL_BuildResponse(
                    commandEcho,
                    sequenceEcho,
                    axisResult,
                    &status
               );

    canResult = CAN_PROTOCOL_SendResponse(
                    CAN_ID_AXIS_Y_L_TO_MASTER,
                    &response
                );

    if (canResult == CAN_PROTOCOL_OK) SLAVE_APP_PulseOkLed();
    else SLAVE_APP_PulseErrLed();
}

void SLAVE_APP_Initialise(void) {
    TMR2_SetInterruptHandler(MOTOR_TimerISR);
    
    IOCBF0_SetInterruptHandler(ENCODER_ISR_Update); /* ISR for A only encoder mode*/
    //IOCBF0_SetInterruptHandler(ENCODER_ISR_A);
    //IOCBF1_SetInterruptHandler(ENCODER_ISR_B); /* Used for AB quadrature mode*/
    INTERRUPT_GlobalInterruptHighEnable();

    OK_LED_OFF();
    ERR_LED_OFF();

    AXIS_Initialise(&axis, &motor, &encoder);
    
    /*
    * Load stored configuration from EEPROM. If no valid configuration is found,
     * the default initialized parameters are saved as the initial EEPROM content.
    */
    if (!APP_CONFIG_Load(&axis, &motor, &encoder)) {
    APP_CONFIG_Save(&axis, &motor, &encoder);
    }

    /*
     * Temporary homing bypass for movement/CAN validation.
     */

    SLAVE_NODE_Initialise(&slaveNode, &axis);
}

void SLAVE_APP_Task(void) {
    CAN_PROTOCOL_CommandFrame_t command;
    CAN_PROTOCOL_Result_t canResult;
    AXIS_Result_t axisResult;
    bool received = false;

    AXIS_Task(&axis);

    canResult = CAN_PROTOCOL_TryReadCommand(
                    CAN_ID_MASTER_TO_AXIS_Y_L,
                    &command,
                    &received
                );

    if (canResult != CAN_PROTOCOL_OK) SLAVE_APP_PulseErrLed();

    if (received) {
        switch (command.command) {
            case AXIS_NODE_CMD_SET_ENCODER_SCALE:
                if (command.value > 0.0f) {
                    encoder.scaleMmPerCount = command.value;
                    axisResult = AXIS_OK;
                }
                else {
                    axisResult = AXIS_ERROR_INVALIDSTATE;
                }
            break;

            case AXIS_NODE_CMD_SET_MAX_TRAVEL:
                if (command.value > 0.0f) {
                    axis.config.maxTravelMm = command.value;
                    axisResult = AXIS_OK;
                }
                else {
                    axisResult = AXIS_ERROR_INVALIDSTATE;
                }
            break;
            
             case AXIS_NODE_CMD_SET_FAST_SPEED:
            if (command.value > 0.0f) {
                axis.config.moveSpeedFastHz = (uint16_t)command.value;
                axisResult = AXIS_OK;
            }
            else {
                axisResult = AXIS_ERROR_INVALIDSTATE;
            }
            break;

            case AXIS_NODE_CMD_SET_SLOW_SPEED:
                if (command.value > 0.0f){
                    axis.config.moveSpeedSlowHz = (uint16_t)command.value;
                    axisResult = AXIS_OK;
                }
                else {
                    axisResult = AXIS_ERROR_INVALIDSTATE;
                }
            break;

            case AXIS_NODE_CMD_SET_POSITION_TOLERANCE:
                if (command.value > 0.0f) {
                    axis.config.positionToleranceMm = command.value;
                    axisResult = AXIS_OK;
                }
                else {
                    axisResult = AXIS_ERROR_INVALIDSTATE;
                }
            break;

            case AXIS_NODE_CMD_SAVE_CONFIG:
                if (APP_CONFIG_Save(&axis, &motor, &encoder)) axisResult = AXIS_OK;
                
                else axisResult = AXIS_ERROR_FAULT;
            break;

             default:
                axisResult = SLAVE_NODE_ExecuteCommand(
                            &slaveNode,
                            command.command,
                            command.value
                            );
            break;
        }

        SLAVE_APP_SendResponse(
            command.command,
            command.sequence,
            axisResult
        );
    }

    __delay_ms(1);
}
