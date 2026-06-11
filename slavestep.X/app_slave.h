/*
 * File: app_slave.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Public interface of the slave application layer.
 * Provides initialization and periodic task functions used to
 * operate the CAN-based slave node within the distributed
 * motion-control system.
 *
 * Created: June 3, 2026
 */

#ifndef SLAVE_APP_H
#define SLAVE_APP_H

void SLAVE_APP_Initialise(void);
void SLAVE_APP_Task(void);

#endif

