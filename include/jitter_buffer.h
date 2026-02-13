/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *jitter_buffer_handle_t;

/** State events posted to config.event_loop when non-NULL */
ESP_EVENT_DECLARE_BASE(JITTER_BUFFER_EVENTS);

enum jitter_buffer_event_id {
    JITTER_EVENT_BUFFERING = 0,  /**< Enter buffering state */
    JITTER_EVENT_UNDERRUN,       /**< Enter underrun state */
    JITTER_EVENT_PLAYING,        /**< Enter playing state */
    JITTER_EVENT_MAX
};

typedef enum {
    AUDIO_FORMAT_ID_OPUS = 0,
    AUDIO_FORMAT_ID_PCM,
} audio_format_id_t;

#define DEFAULT_JITTER_BUFFER_CONFIG() {     \
    .on_output_data = NULL,                  \
    .with_header = false,                    \
    .buffer_size = 11 * 1024,                \
    .frame_size = 512,                       \
    .frame_interval = 20,                    \
    .high_water = 20,                        \
    .low_water = 10,                         \
    .output_silence_on_empty = false,        \
    .audio_format_id = AUDIO_FORMAT_ID_OPUS, \
    .event_loop = NULL,                      \
}

/** State event: event_data is a pointer to a copied jitter_buffer_handle_t; use *(jitter_buffer_handle_t *)event_data to get the handle */

typedef struct {
    /* Callback */
    void (*on_output_data)(const uint8_t *data, size_t len);

    /* Buffer layout */
    size_t                   buffer_size;   /**< Ring buffer size in bytes */
    bool                     with_header;   /**< true: variable-length frames stored as [2-byte BE length][payload] */
    uint32_t                 frame_size;    /**< Fixed frame size (no header) or max payload per frame (with_header) */
    uint32_t                 frame_interval; /**< Output interval (ms). For OPUS+output_silence_on_empty: 20/40/60/120 only */
    uint32_t                 high_water;    /**< Start playing when frame count reaches this */
    uint32_t                 low_water;     /**< Enter underrun when frame count drops below this */

    /* Audio format and silence */
    audio_format_id_t        audio_format_id;       /**< AUDIO_FORMAT_ID_OPUS or AUDIO_FORMAT_ID_PCM */
    bool                     output_silence_on_empty; /**< true: output silence when no data; false: skip on_output_data */

    /* Optional event notification */
    esp_event_loop_handle_t  event_loop;    /**< If non-NULL, post BUFFERING/UNDERRUN/PLAYING events */
} jitter_buffer_config_t;

/* Breif: Create a jitter buffer
 *
 * config[in]  The configuration of the jitter buffer
 *
 * return:
 *       - NULL: Create failed
 *       - Others: The handle of the jitter buffer
 */
jitter_buffer_handle_t jitter_buffer_create(const jitter_buffer_config_t *config);

/* Breif: Start the jitter buffer
 *
 * handle[in]  The handle of the jitter buffer
 *
 * return:
 *       - ESP_OK: Start success
 *       - Others: Start failed
 */
esp_err_t jitter_buffer_start(jitter_buffer_handle_t handle);

/* Breif: Stop the jitter buffer
 *
 * handle[in]  The handle of the jitter buffer
 *
 * return:
 *       - ESP_OK: Stop success
 *       - Others: Stop failed
 */
esp_err_t jitter_buffer_stop(jitter_buffer_handle_t handle);

/* Breif: Destroy the jitter buffer
 *
 * handle[in]  The handle of the jitter buffer
 *
 * return:
 *       - ESP_OK: Destroy success
 *       - Others: Destroy failed
 */
esp_err_t jitter_buffer_destroy(jitter_buffer_handle_t handle);

/* Breif: Reset the jitter buffer
 *
 * handle[in]  The handle of the jitter buffer
 *
 * return:
 *       - ESP_OK: Reset success
 *       - Others: Reset failed
 */
esp_err_t jitter_buffer_reset(jitter_buffer_handle_t handle);

/* Breif: Write data to the jitter buffer
 *
 * handle[in]  The handle of the jitter buffer
 * data[in]    The data to be written
 * len[in]     The length of the data
 *
 * return:
 *       - ESP_OK: Write success
 *       - Others: Write failed
 */
esp_err_t jitter_buffer_write(jitter_buffer_handle_t handle, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
