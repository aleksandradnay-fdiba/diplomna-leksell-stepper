#include "master_console.h"
#include "master_axis_client.h"
#include "uart_print.h"
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/uart1.h"

#include <stdint.h>
#include <stdbool.h>

static bool MASTER_CONSOLE_ReadChar(uint8_t *ch) {
    if (ch == 0) return false;

    if (UART1_is_rx_ready()) {
        *ch = UART1_Read();
        return true;
    }

    return false;
}

static void MASTER_CONSOLE_PrintMenu(void) {
    send_string("\r\n--- MASTER MENU ---\r\n");
    send_string("h = HOME\r\n");
    send_string("s = STOP\r\n");
    send_string("e = EMSTOP\r\n");
    send_string("r = RESET\r\n");
    send_string("g = GET STATUS\r\n");
    send_string("1 = MOVE ABS 1mm\r\n");
    send_string("5 = MOVE ABS 5mm\r\n");
    send_string("? = MENU\r\n");
    send_string("-------------------\r\n");
}

void MASTER_CONSOLE_Init(void) {
    send_string("\r\nMASTER CONSOLE\r\n");
    MASTER_CONSOLE_PrintMenu();
}

void MASTER_CONSOLE_Task(void) {
    uint8_t ch;
    
    if(!MASTER_CONSOLE_ReadChar(&ch)) return;
    
    switch (ch) {
        case 'h':
        case 'H':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_HOME, 0.0f);
            break;
            
        case 's':
        case 'S':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_STOP, 0.0f);
            break; 
            
        case 'e':
        case 'E':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_EMSTOP, 0.0f);
            break;
            
        case 'r':
        case 'R':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_RESET, 0.0f);
            break;
            
        case 'g':
        case 'G':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_GET_STATUS, 0.0f);
            break;
            
        case '1':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_MOVE_ABS_MM, 1.0f);
            break;

        case '5':
            MASTER_AXIS_CLIENT_SendCommand(AXIS_ID_Y_L, AXIS_NODE_CMD_MOVE_ABS_MM, 5.0f);
            break;
       
        case '?':
            MASTER_CONSOLE_PrintMenu();
            break;  
            
        case '\r':
        case '\n':
            break;
            
       default:
            send_string("UNKNOWN CMD\r\n");
            MASTER_CONSOLE_PrintMenu();
            break;
    }
}
