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

/** 状态事件：在 config.event_loop 非 NULL 时，状态切换会 post 到该 loop */
ESP_EVENT_DECLARE_BASE(JITTER_BUFFER_EVENTS);

enum jitter_buffer_event_id {
    JITTER_EVENT_BUFFERING = 0,  /**< 进入缓冲（蓄水）状态 */
    JITTER_EVENT_UNDERRUN,       /**< 进入欠载状态 */
    JITTER_EVENT_PLAYING,        /**< 进入播放状态 */
    JITTER_EVENT_MAX
};

#define DEFAULT_JITTER_BUFFER_CONFIG() {   \
    .on_output_data = NULL,                \
    .with_header = false,                  \
    .buffer_size = 11 * 1024,              \
    .frame_size = 512,                     \
    .frame_interval = 20,                  \
    .high_water = 20,                      \
    .low_water = 10,                       \
    .output_silence_on_empty = false,      \
    .event_loop = NULL,                    \
}

/** 状态事件：handler 收到的 event_data 指向已拷贝的 jitter_buffer_handle_t，即 (*(jitter_buffer_handle_t *)event_data) 为对应 handle */

typedef struct {
    void (*on_output_data)(const uint8_t *data, size_t len);
    bool                     with_header;              /**< true: 每帧长度不固定，由每次 write 的 len 决定，存储为 [2 字节大端长度][payload] */
    size_t                   buffer_size;              /**< 环形缓冲大小（字节） */
    uint32_t                 frame_size;               /**< 无头时为固定帧长；with_header 时为单帧 payload 最大长度（用于校验与输出缓冲） */
    uint32_t                 frame_interval;           /**< 输出间隔（ms） */
    uint32_t                 high_water;               /**< 达到此帧数开始播放 */
    uint32_t                 low_water;                /**< 低于此帧数进入欠载 */
    bool                     output_silence_on_empty;  /**< true: 无数据时输出静音包；false: 无数据时不调用 on_output_data */
    esp_event_loop_handle_t  event_loop;               /**< 可选；非 NULL 时在 BUFFERING/UNDERRUN/PLAYING 切换时 post 事件 */
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
}
#endif
