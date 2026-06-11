/*
 * File: app_slave.h
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Provides the public interface of the slave application layer.
 * The slave application initializes the local motion-control modules
 * and executes the periodic task responsible for CAN command handling,
 * axis control, and status response generation.
 *
 * Created: June 3, 2026
 */

#ifndef SLAVE_APP_H
#define SLAVE_APP_H

void SLAVE_APP_Initialise(void);
void SLAVE_APP_Task(void);

#endif

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "axis.h"
#include "motor.h"
#include "encoder.h"

#include <stdbool.h>
#include <stdint.h>

bool APP_CONFIG_Load(AXIS_t *axis, MOTOR_t *motor, ENCODER_t *encoder);
bool APP_CONFIG_Save(const AXIS_t *axis, const MOTOR_t *motor, const ENCODER_t *encoder);
bool APP_CONFIG_IsValidInEeprom(void);

#endif

