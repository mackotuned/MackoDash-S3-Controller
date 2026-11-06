#pragma once

/*
 * Wireless link to the dash. The dash runs a private Wi-Fi network named
 * "MackoDash-XXXX"; the knob joins the strongest one it hears (preferring
 * the one it used last) and exchanges the usual MD frames over UDP.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*knob_link_frame_cb_t)(const uint8_t *frame, size_t length);

void knob_link_start(knob_link_frame_cb_t on_frame);

/* Sends a complete frame to the dash; false when the wireless link is down. */
bool knob_link_send(const uint8_t *frame, size_t length);

bool knob_link_up(void);

/* The dash's address on its knob network (network byte order), 0 if not linked. */
uint32_t knob_link_gateway(void);
