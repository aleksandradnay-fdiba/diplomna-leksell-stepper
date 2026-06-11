/*
 * File: app_config.c
 * Author: Aleksandra Naydenova
 *
 * Description:
 * Implements persistent configuration storage for the slave node.
 * The module stores selected axis, motor, and encoder parameters in
 * EEPROM and restores them during startup if the stored data is valid.
 *
 * The configuration data is protected using a magic value, version
 * number, and checksum to detect uninitialized or corrupted EEPROM data.
 *
 * Created: June 3, 2026
 */

#include "app_config.h"
#include "mcc_generated_files/memory.h"

#include <stdint.h>
#include <stdbool.h>

#define APP_CONFIG_EEPROM_BASE_ADDR     0x00u
#define APP_CONFIG_MAGIC                0xCAFE
#define APP_CONFIG_VERSION              1u

typedef struct {
    uint16_t magic;
    uint8_t version;

    float encoderScaleMmPerCount;
    float maxTravelMm;
    float fastZoneMm;
    float positionToleranceMm;

    uint16_t moveSpeedFastHz;
    uint16_t moveSpeedSlowHz;

    uint8_t encoderMode;  /* Reserved for future runtime encoder mode selection */
    uint8_t motorStepMode; /* Reserved for future runtime motor step mode selection */

    uint16_t checksum;
} APP_CONFIG_Data_t;

static uint16_t APP_CONFIG_CalculateChecksum(const APP_CONFIG_Data_t *config) {
    const uint8_t *bytes;
    uint16_t sum;
    uint8_t i;

    if (config == 0) return 0u;

    bytes = (const uint8_t *)config;
    sum = 0u;

    for (i = 0u; i < (sizeof(APP_CONFIG_Data_t) - sizeof(uint16_t)); i++) {
        sum = (uint16_t)(sum + bytes[i]);
    }

    return sum;
}

static void APP_CONFIG_ReadBytes(uint16_t address, uint8_t *data, uint8_t length) {
    uint8_t i;

    if (data == 0) return;

    for (i = 0u; i < length; i++) {
        data[i] = DATAEE_ReadByte((uint16_t)(address + i));
    }
}

static void APP_CONFIG_WriteBytes(uint16_t address, const uint8_t *data, uint8_t length) {
    uint8_t i;

    if (data == 0) return;

    for (i = 0u; i < length; i++) {
        DATAEE_WriteByte((uint16_t)(address + i), data[i]);
    }
}

static bool APP_CONFIG_IsDataValid(const APP_CONFIG_Data_t *config) {
    if (config == 0) return false;

    if (config->magic != APP_CONFIG_MAGIC) return false;
    if (config->version != APP_CONFIG_VERSION) return false;
    if (config->checksum != APP_CONFIG_CalculateChecksum(config)) return false;

    if (config->encoderScaleMmPerCount <= 0.0f) return false;
    if (config->maxTravelMm <= 0.0f) return false;
    if (config->moveSpeedFastHz == 0u) return false;
    if (config->moveSpeedSlowHz == 0u) return false;
    if (config->positionToleranceMm <= 0.0f) return false;

    return true;
}

bool APP_CONFIG_Load(AXIS_t *axis, MOTOR_t *motor, ENCODER_t *encoder)
{
    APP_CONFIG_Data_t config;

    if ((axis == 0) || (motor == 0) || (encoder == 0)) return false;

    APP_CONFIG_ReadBytes(
        APP_CONFIG_EEPROM_BASE_ADDR,
        (uint8_t *)&config,
        sizeof(APP_CONFIG_Data_t)
    );

    if (!APP_CONFIG_IsDataValid(&config)) return false;

    encoder->scaleMmPerCount = config.encoderScaleMmPerCount;

    axis->config.maxTravelMm = config.maxTravelMm;
    axis->config.fastZoneMm = config.fastZoneMm;
    axis->config.positionToleranceMm = config.positionToleranceMm;

    axis->config.moveSpeedFastHz = config.moveSpeedFastHz;
    axis->config.moveSpeedSlowHz = config.moveSpeedSlowHz;

    return true;
}

bool APP_CONFIG_Save(const AXIS_t *axis, const MOTOR_t *motor, const ENCODER_t *encoder) {
    APP_CONFIG_Data_t config;

    if ((axis == 0) || (motor == 0) || (encoder == 0)) return false;

    config.magic = APP_CONFIG_MAGIC;
    config.version = APP_CONFIG_VERSION;

    config.encoderScaleMmPerCount = encoder->scaleMmPerCount;

    config.maxTravelMm = axis->config.maxTravelMm;
    config.fastZoneMm = axis->config.fastZoneMm;
    config.positionToleranceMm = axis->config.positionToleranceMm;

    config.moveSpeedFastHz = axis->config.moveSpeedFastHz;
    config.moveSpeedSlowHz = axis->config.moveSpeedSlowHz;

    config.checksum = APP_CONFIG_CalculateChecksum(&config);

    APP_CONFIG_WriteBytes(
        APP_CONFIG_EEPROM_BASE_ADDR,
        (const uint8_t *)&config,
        sizeof(APP_CONFIG_Data_t)
    );

    return true;
}

bool APP_CONFIG_IsValidInEeprom(void) {
    APP_CONFIG_Data_t config;

    APP_CONFIG_ReadBytes(
        APP_CONFIG_EEPROM_BASE_ADDR,
        (uint8_t *)&config,
        sizeof(APP_CONFIG_Data_t)
    );

    return APP_CONFIG_IsDataValid(&config);
}
