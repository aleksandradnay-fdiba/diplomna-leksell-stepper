/*
 * File: slave_node.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements the slave-node command handling layer.
 * The module connects received CAN commands to the local axis-control
 * functions and provides status information used for response frames.
 *
 * Created: April 11, 2026
 */

#include "slave_node.h"

/*
 * Links the slave-node object to the controlled axis instance.
 */
void SLAVE_NODE_Initialise(SLAVE_NODE_t *node, AXIS_t *axis) {
    if((node == 0) || (axis == 0)) return;
    
    node->axis = axis;

}

/*
 * Executes a received slave-node command by calling the corresponding
 * axis-control function.
 */
AXIS_Result_t SLAVE_NODE_ExecuteCommand(SLAVE_NODE_t *node,
                                        SLAVE_NODE_Command_t command,
                                        float value) {
    if ((node == 0) || (node->axis == 0)) return AXIS_ERROR_NULL;
    
    switch (command) {
        case AXIS_NODE_CMD_HOME:
            return AXIS_Home(node->axis);
            
        case AXIS_NODE_CMD_STOP:
            return AXIS_Stop(node->axis);

        case AXIS_NODE_CMD_EMSTOP:
            return AXIS_EmergencyStop(node->axis);

        case AXIS_NODE_CMD_RESET:
            return AXIS_ResetFault(node->axis);

        case AXIS_NODE_CMD_MOVE_ABS_MM:
            return AXIS_MoveToMm(node->axis, value);

        case AXIS_NODE_CMD_MOVE_REL_MM:
            return AXIS_MoveDeltaMm(node->axis, value);

        case AXIS_NODE_CMD_GET_STATUS:
            return AXIS_OK;

        case AXIS_NODE_CMD_NONE:
        default:
            return AXIS_ERROR_INVALIDSTATE;
    }
}

/*
 * Copies the current axis status into a slave-node status structure.
 */
void SLAVE_NODE_GetStatus(const SLAVE_NODE_t *node, SLAVE_NODE_Status_t *status) {
    if ((node == 0) || (node->axis == 0) || (status == 0)) return;
    
    status->state = AXIS_GetState(node->axis);
    status->faultCode = AXIS_GetFaultCode(node->axis);
    status->homingStep = AXIS_GetHomingStep(node->axis);
    
    status->currentPositionMm = AXIS_GetCurrentPositionMm(node->axis);
    status->targetPositionMm = AXIS_GetTargetPositionMm(node->axis);
    status->positionDeltaMm = AXIS_GetPositionDeltaMm(node->axis);
    
    status->homingDetectionCount = AXIS_GetHomingDetectionCount(node->axis);

    status->homed = AXIS_IsHomed(node->axis);
    status->busy = AXIS_IsBusy(node->axis);
    status->emergencyActive = AXIS_IsEmergencyActive(node->axis);
    status->faultActive = AXIS_HasFault(node->axis);
}
