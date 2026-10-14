#ifndef RUIN_AIR_BRIDGE_H
#define RUIN_AIR_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define RUIN_AIR_RELEASE_COMMAND 23

typedef struct {
    bool connected;
    bool pressure_valid;
    bool compressor_valid;
    bool compressor_on;
    uint16_t tank_psi;
    uint16_t front_left_psi;
    uint16_t front_right_psi;
    uint16_t rear_left_psi;
    uint16_t rear_right_psi;
} ruin_air_state_t;

typedef void (*ruin_air_state_callback_t)(const ruin_air_state_t *state, void *context);
typedef void (*ruin_air_tx_callback_t)(const uint8_t *frame, size_t length, void *context);

esp_err_t ruin_air_bridge_init(ruin_air_state_callback_t callback, void *context);
void ruin_air_bridge_set_tx_callback(ruin_air_tx_callback_t callback, void *context);
void ruin_air_bridge_ingest_frame(const uint8_t *frame, size_t length);
esp_err_t ruin_air_bridge_request(uint8_t command);
esp_err_t ruin_air_bridge_air_out(void);
esp_err_t ruin_air_bridge_set_compressor(bool enabled, uint16_t on_psi, uint16_t off_psi);

#endif