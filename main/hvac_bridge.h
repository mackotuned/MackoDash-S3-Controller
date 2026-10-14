#ifndef HVAC_BRIDGE_H
#define HVAC_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool connected;
    uint8_t fan;
    uint8_t mode;
    uint8_t temp;
    uint8_t target_fan;
    uint8_t target_mode;
    uint8_t target_temp;
    uint8_t moving_mask;
    uint8_t fault_flags;
} hvac_bridge_state_t;

typedef void (*hvac_bridge_state_callback_t)(const hvac_bridge_state_t *state,
                                             void *context);

esp_err_t hvac_bridge_init(hvac_bridge_state_callback_t callback, void *context);
esp_err_t hvac_bridge_set(uint8_t fan, uint8_t mode, uint8_t temp);

#endif