#include "mcc_generated_files/mcc.h"
#include "master_axis_client.h"

#include <stdint.h>
#include <stdbool.h>

#define OK_LED_ON()       IO_RA4_SetHigh()
#define OK_LED_OFF()      IO_RA4_SetLow()

#define ERR_LED_ON()      IO_RA5_SetHigh()
#define ERR_LED_OFF()     IO_RA5_SetLow()

static void pulse_ok_led(void)
{
    OK_LED_ON();
    __delay_ms(50);
    OK_LED_OFF();
}

static void pulse_err_led(void)
{
    ERR_LED_ON();
    __delay_ms(100);
    ERR_LED_OFF();
}

static void send_and_blink(SLAVE_NODE_Command_t cmd, float value)
{
    bool ok;

    ok = MASTER_AXIS_CLIENT_SendCommand(
            AXIS_ID_Y_L,
            cmd,
            value
         );

    if (ok) pulse_ok_led();
    else pulse_err_led();
}

void main(void)
{
    SYSTEM_Initialize();

    /*
     * For new hardware master: keep OFF if global interrupts cause storm.
     * If old master works with it ON, try ON only after CAN is proven.
     */
    // INTERRUPT_GlobalInterruptHighEnable();

    OK_LED_OFF();
    ERR_LED_OFF();

    MASTER_AXIS_CLIENT_Init();

    __delay_ms(5000);

    /* 1) GET STATUS first */
    send_and_blink(AXIS_NODE_CMD_GET_STATUS, 0.0f);

    __delay_ms(500);

    /* 2) MOVE TO 15 mm */
    send_and_blink(AXIS_NODE_CMD_MOVE_ABS_MM, 15.0f);

    __delay_ms(3000);

    /* 3) MOVE TO 0 mm */
    send_and_blink(AXIS_NODE_CMD_MOVE_ABS_MM, 0.0f);

    __delay_ms(3000);

    /* 4) MOVE TO 30 mm */
    send_and_blink(AXIS_NODE_CMD_MOVE_ABS_MM, 30.0f);

    __delay_ms(3000);

    /* 5) MOVE TO 0 mm */
    //send_and_blink(AXIS_NODE_CMD_MOVE_ABS_MM, 0.0f);

    while (1)
    {
        send_and_blink(AXIS_NODE_CMD_GET_STATUS, 0.0f);
        __delay_ms(1000);
    }
}