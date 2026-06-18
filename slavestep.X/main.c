#include "mcc_generated_files/mcc.h"
#include "app_slave.h"

void main(void)
{
    SYSTEM_Initialize();

    SLAVE_APP_Initialise();

    while (1)
    {
        SLAVE_APP_Task();
    }
}