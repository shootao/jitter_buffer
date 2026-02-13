/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "jitter_buffer.h"

#define JITTER_BUFFER_EVENT_START (1 << 0)
#define JITTER_BUFFER_EVENT_STOP  (1 << 1)
#define JITTER_BUFFER_EVENT_EXIT  (1 << 2)

#define JITTER_BUFFER_EVENT_ACK (1 << 0)

#define JITTER_HEADER_LEN 2  /* with_header 时长度字段：大端 2 字节 */

ESP_EVENT_DEFINE_BASE(JITTER_BUFFER_EVENTS);

static const char *TAG = "JITTER_BUFFER";

typedef enum {
    JITTER_STATE_IDLE,       // 空闲
    JITTER_STATE_BUFFERING,  // 正在缓冲，等待达到高水位
    JITTER_STATE_PLAYING,    // 正在播放
    JITTER_STATE_UNDERRUN,   // 欠载，需要重新缓冲
} jitter_buffer_state_t;

typedef struct {
    jitter_buffer_config_t  config;
    uint8_t                *buffer;
    size_t                  buffer_size;
    size_t                  write_pos;
    size_t                  read_pos;
    size_t                  data_size;
    size_t                  total_read;
    size_t                  total_written;
    uint8_t                *frame_buffer;
    jitter_buffer_state_t   state;
    SemaphoreHandle_t       mutex;
    EventGroupHandle_t      event_group;
    EventGroupHandle_t      event_group_ack;
    TaskHandle_t            task_handle;
    TickType_t              last_wake_time;
    uint32_t                underrun_count;
    uint32_t                overrun_count;
    bool                    running;
} jitter_buffer_t;

static int s_jitter_buffer_read(jitter_buffer_t *jitter_buffer, uint8_t *data, size_t len);

/** 状态切换时向 config.event_loop 发送事件（若已配置） */
static void s_post_state_event(jitter_buffer_t *jb, int32_t event_id)
{
    if (jb->config.event_loop == NULL) {
        return;
    }
    jitter_buffer_handle_t h = (jitter_buffer_handle_t)jb;
    esp_err_t ret = esp_event_post_to(jb->config.event_loop, JITTER_BUFFER_EVENTS, event_id,
                                      &h, sizeof(h), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_event_post_to failed: %s", esp_err_to_name(ret));
    }
}

/* 环形缓冲原始写（调用方需已持有 mutex，且保证有空间或已先 discard） */
static void s_ring_write(jitter_buffer_t *jb, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }
    size_t first = jb->buffer_size - jb->write_pos;
    if (first >= len) {
        memcpy(jb->buffer + jb->write_pos, data, len);
    } else {
        memcpy(jb->buffer + jb->write_pos, data, first);
        memcpy(jb->buffer, data + first, len - first);
    }
    jb->write_pos = (jb->write_pos + len) % jb->buffer_size;
    jb->data_size += len;
    jb->total_written += len;
}

/* 环形缓冲 peek，不移动 read_pos（调用方需已持有 mutex） */
static void s_ring_peek(jitter_buffer_t *jb, uint8_t *buf, size_t len)
{
    size_t to_read = (len < jb->data_size) ? len : jb->data_size;
    if (to_read == 0) {
        return;
    }
    size_t first = jb->buffer_size - jb->read_pos;
    if (first >= to_read) {
        memcpy(buf, jb->buffer + jb->read_pos, to_read);
    } else {
        memcpy(buf, jb->buffer + jb->read_pos, first);
        memcpy(buf + first, jb->buffer, to_read - first);
    }
}

/* 环形缓冲原始读，返回实际读到的字节数（调用方需已持有 mutex） */
static size_t s_ring_read(jitter_buffer_t *jb, uint8_t *data, size_t len)
{
    size_t to_read = (len < jb->data_size) ? len : jb->data_size;
    if (to_read == 0) {
        return 0;
    }
    size_t first = jb->buffer_size - jb->read_pos;
    if (first >= to_read) {
        memcpy(data, jb->buffer + jb->read_pos, to_read);
    } else {
        memcpy(data, jb->buffer + jb->read_pos, first);
        memcpy(data + first, jb->buffer, to_read - first);
    }
    jb->read_pos = (jb->read_pos + to_read) % jb->buffer_size;
    jb->data_size -= to_read;
    jb->total_read += to_read;
    return to_read;
}

/* with_header 时从 read_pos 起逐帧解析，返回完整帧个数（调用方需已持有 mutex） */
static size_t s_get_frame_count_with_header(jitter_buffer_t *jb)
{
    size_t offset = 0;
    size_t remaining = jb->data_size;
    size_t count = 0;

    while (remaining >= JITTER_HEADER_LEN) {
        uint8_t b0 = jb->buffer[(jb->read_pos + offset) % jb->buffer_size];
        uint8_t b1 = jb->buffer[(jb->read_pos + offset + 1) % jb->buffer_size];
        uint16_t L = (uint16_t)((b0 << 8) | b1);
        /* 防止异常长度导致死循环，单帧不超过 buffer 一半视为合理 */
        if (L > jb->buffer_size / 2) {
            break;
        }
        if (remaining < JITTER_HEADER_LEN + L) {
            break;
        }
        count++;
        offset += JITTER_HEADER_LEN + L;
        remaining -= (JITTER_HEADER_LEN + L);
    }
    return count;
}

static esp_err_t s_jitter_buffer_process(jitter_buffer_t *jitter_buffer)
{

    vTaskDelayUntil(&jitter_buffer->last_wake_time, pdMS_TO_TICKS(jitter_buffer->config.frame_interval));
    int read_len = s_jitter_buffer_read(jitter_buffer, jitter_buffer->frame_buffer, jitter_buffer->config.frame_size);
    if (read_len > 0) {
        jitter_buffer->config.on_output_data(jitter_buffer->frame_buffer, read_len);
    } else if (read_len == 0 && jitter_buffer->config.output_silence_on_empty) {
        memset(jitter_buffer->frame_buffer, 0, jitter_buffer->config.frame_size);
        jitter_buffer->config.on_output_data(jitter_buffer->frame_buffer, jitter_buffer->config.frame_size);
    }
    return ESP_OK;
}

static void jitter_buffer_task(void *arg)
{
    jitter_buffer_t *jitter_buffer = (jitter_buffer_t *)arg;
    jitter_buffer->running = true;
    while (jitter_buffer->running) {
        EventBits_t bits = xEventGroupWaitBits(jitter_buffer->event_group, JITTER_BUFFER_EVENT_START | JITTER_BUFFER_EVENT_EXIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & JITTER_BUFFER_EVENT_EXIT) {
            ESP_LOGI(TAG, "Jitter buffer task exit");
            break;
        }
        xEventGroupSetBits(jitter_buffer->event_group, JITTER_BUFFER_EVENT_START);
        if (bits & JITTER_BUFFER_EVENT_START) {
            jitter_buffer->last_wake_time = xTaskGetTickCount();
            if (jitter_buffer->event_group_ack != NULL) {
                xEventGroupSetBits(jitter_buffer->event_group_ack, JITTER_BUFFER_EVENT_ACK);
            }
            while (1) {
                EventBits_t bits = xEventGroupWaitBits(jitter_buffer->event_group, JITTER_BUFFER_EVENT_START | JITTER_BUFFER_EVENT_EXIT | JITTER_BUFFER_EVENT_STOP, pdFALSE, pdFALSE, portMAX_DELAY);
                if (bits & JITTER_BUFFER_EVENT_EXIT) {
                    ESP_LOGI(TAG, "Jitter buffer task exit");
                    jitter_buffer->running = false;
                    break;
                }
                if (bits & JITTER_BUFFER_EVENT_STOP) {
                    ESP_LOGI(TAG, "Jitter buffer task stop");
                    if (jitter_buffer->event_group_ack != NULL) {
                        xEventGroupSetBits(jitter_buffer->event_group_ack, JITTER_BUFFER_EVENT_ACK);
                    }
                    break;
                }
                if (bits & JITTER_BUFFER_EVENT_START) {
                    if (jitter_buffer->event_group_ack != NULL) {
                        xEventGroupSetBits(jitter_buffer->event_group_ack, JITTER_BUFFER_EVENT_ACK);
                    }
                }
                s_jitter_buffer_process(jitter_buffer);
            }
        }
    }
    if (jitter_buffer->event_group_ack != NULL) {
        xEventGroupSetBits(jitter_buffer->event_group_ack, JITTER_BUFFER_EVENT_ACK);
    }
    vTaskDelete(NULL);
}

static int s_jitter_buffer_read(jitter_buffer_t *jitter_buffer, uint8_t *data, size_t len)
{
    if (jitter_buffer->buffer == NULL || jitter_buffer->mutex == NULL) {
        return -1;
    }

    if (xSemaphoreTake(jitter_buffer->mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Jitter buffer read: mutex timeout");
        return -1;
    }

    size_t frame_count = jitter_buffer->config.with_header
        ? s_get_frame_count_with_header(jitter_buffer)
        : (jitter_buffer->data_size / jitter_buffer->config.frame_size);

    // 状态机：在读路径也检查高水位，避免“刚切到 PLAYING 时 buffer 已满、下一拍写 overrun”
    if (jitter_buffer->state == JITTER_STATE_BUFFERING ||
        jitter_buffer->state == JITTER_STATE_UNDERRUN) {
        if (frame_count >= jitter_buffer->config.high_water) {
            jitter_buffer->state = JITTER_STATE_PLAYING;
            s_post_state_event(jitter_buffer, JITTER_EVENT_PLAYING);
            ESP_LOGI(TAG, "Jitter buffer: start playing (read path), frames=%zu", frame_count);
        } else {
            xSemaphoreGive(jitter_buffer->mutex);
            return 0;
        }
    }

    // 状态机：低于低水位时进入欠载状态
    if (jitter_buffer->state == JITTER_STATE_PLAYING) {
        if (frame_count < jitter_buffer->config.low_water) {
            jitter_buffer->state = JITTER_STATE_UNDERRUN;
            jitter_buffer->underrun_count++;
            s_post_state_event(jitter_buffer, JITTER_EVENT_UNDERRUN);
            ESP_LOGW(TAG, "Jitter buffer underrun: frames=%zu, count=%lu",
                     frame_count, (unsigned long)jitter_buffer->underrun_count);
            xSemaphoreGive(jitter_buffer->mutex);
            return 0;  // 返回0表示暂时没有数据，但不是错误
        }
    }

    // 读取数据
    if (jitter_buffer->config.with_header) {
        /* 带头格式：先 peek 2 字节大端长度，数据够再整帧读 */
        if (jitter_buffer->data_size < JITTER_HEADER_LEN) {
            xSemaphoreGive(jitter_buffer->mutex);
            return 0;
        }
        uint8_t hdr[JITTER_HEADER_LEN];
        s_ring_peek(jitter_buffer, hdr, JITTER_HEADER_LEN);
        uint16_t payload_len = (uint16_t)((hdr[0] << 8) | hdr[1]);
        if (payload_len > jitter_buffer->config.frame_size) {
            /* frame_size 为 with_header 时单帧 payload 上限 */
            ESP_LOGW(TAG, "Jitter buffer read: header len=%u > max_payload(%u), skip frame", payload_len, jitter_buffer->config.frame_size);
            if (jitter_buffer->data_size < JITTER_HEADER_LEN + payload_len) {
                xSemaphoreGive(jitter_buffer->mutex);
                return 0;
            }
            (void)s_ring_read(jitter_buffer, hdr, JITTER_HEADER_LEN);
            size_t left = (size_t)payload_len;
            while (left > 0) {
                size_t chunk = (left < jitter_buffer->config.frame_size) ? left : jitter_buffer->config.frame_size;
                (void)s_ring_read(jitter_buffer, jitter_buffer->frame_buffer, chunk);
                left -= chunk;
            }
            xSemaphoreGive(jitter_buffer->mutex);
            return 0;  /* 丢弃整帧，下次从下一帧头对齐 */
        }
        if (jitter_buffer->data_size < JITTER_HEADER_LEN + payload_len) {
            xSemaphoreGive(jitter_buffer->mutex);
            return 0;  /* 整帧未到齐，不消费 */
        }
        (void)s_ring_read(jitter_buffer, hdr, JITTER_HEADER_LEN);  /* 跳过 2 字节头 */
        size_t got = s_ring_read(jitter_buffer, data, (size_t)payload_len);
        xSemaphoreGive(jitter_buffer->mutex);
        return (int)got;
    }

    /* 无头：按固定帧长读 */
    size_t read_len = (len < jitter_buffer->data_size) ? len : jitter_buffer->data_size;
    if (read_len == 0) {
        xSemaphoreGive(jitter_buffer->mutex);
        return 0;
    }
    s_ring_read(jitter_buffer, data, read_len);
    xSemaphoreGive(jitter_buffer->mutex);
    return (int)read_len;
}

jitter_buffer_handle_t jitter_buffer_create(const jitter_buffer_config_t *config)
{
    if (config->frame_interval <= 0) {
        ESP_LOGE(TAG, "Jitter buffer create: frame_interval <= 0");
        return NULL;
    }
    jitter_buffer_t *jitter_buffer = (jitter_buffer_t *)malloc(sizeof(jitter_buffer_t));
    if (jitter_buffer == NULL) {
        ESP_LOGE(TAG, "Jitter buffer create: malloc failed");
        return NULL;
    }
    jitter_buffer->config = *config;
    jitter_buffer->buffer = NULL;
    jitter_buffer->buffer_size = config->buffer_size;
    /* with_header 时每帧长度不固定；frame_size 为 payload 上限，按最坏（每帧均为上限）保证至少能容纳 high_water 帧 */
    if (config->with_header) {
        size_t min_size = (size_t)config->high_water * (JITTER_HEADER_LEN + config->frame_size);
        if (jitter_buffer->buffer_size < min_size) {
            ESP_LOGW(TAG, "Jitter buffer: with_header needs buffer_size >= %zu (high_water*(2+max_payload)), adjust %zu -> %zu",
                     min_size, jitter_buffer->buffer_size, min_size);
            jitter_buffer->buffer_size = min_size;
        }
    }
    jitter_buffer->write_pos = 0;
    jitter_buffer->read_pos = 0;
    jitter_buffer->data_size = 0;
    jitter_buffer->total_read = 0;
    jitter_buffer->total_written = 0;
    jitter_buffer->underrun_count = 0;
    jitter_buffer->overrun_count = 0;
    jitter_buffer->state = JITTER_STATE_IDLE;
    jitter_buffer->buffer = heap_caps_calloc_prefer(1, jitter_buffer->buffer_size, 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_INTERNAL);
    if (jitter_buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Jitter buffer create: heap_caps_calloc_prefer failed, buffer_size=%zu", jitter_buffer->buffer_size);
        free(jitter_buffer);
        return NULL;
    }
    jitter_buffer->frame_buffer = heap_caps_calloc_prefer(1, config->frame_size, 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_INTERNAL);
    if (jitter_buffer->frame_buffer == NULL) {
        free(jitter_buffer->buffer);
        ESP_LOGE(TAG, "Jitter buffer create: heap_caps_calloc_prefer failed");
        free(jitter_buffer);
        return NULL;
    }
    jitter_buffer->mutex = xSemaphoreCreateMutex();
    if (jitter_buffer->mutex == NULL) {
        free(jitter_buffer->buffer);
        free(jitter_buffer->frame_buffer);
        ESP_LOGE(TAG, "Jitter buffer create: xSemaphoreCreateMutex failed");
        free(jitter_buffer);
        return NULL;
    }
    jitter_buffer->event_group = xEventGroupCreate();
    if (jitter_buffer->event_group == NULL) {
        vSemaphoreDelete(jitter_buffer->mutex);
        free(jitter_buffer->buffer);
        free(jitter_buffer->frame_buffer);
        ESP_LOGE(TAG, "Jitter buffer create: xEventGroupCreate failed");
        free(jitter_buffer);
        return NULL;
    }
    jitter_buffer->event_group_ack = xEventGroupCreate();
    if (jitter_buffer->event_group_ack == NULL) {
        vEventGroupDelete(jitter_buffer->event_group);
        vSemaphoreDelete(jitter_buffer->mutex);
        free(jitter_buffer->buffer);
        free(jitter_buffer->frame_buffer);
        ESP_LOGE(TAG, "Jitter buffer create: xEventGroupCreate(event_group_ack) failed");
        free(jitter_buffer);
        return NULL;
    }
    jitter_buffer->task_handle = NULL;
    jitter_buffer->running = true;

#if (configSUPPORT_STATIC_ALLOCATION == 1) && defined(CONFIG_SPIRAM_BOOT_INIT)
    xTaskCreatePinnedToCoreWithCaps(jitter_buffer_task,
                                    "jitter_buffer_task",
                                    4096,
                                    jitter_buffer,
                                    10,
                                    &jitter_buffer->task_handle,
                                    1,
                                    (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#else
    xTaskCreatePinnedToCore(jitter_buffer_task,
                            "jitter_buffer_task",
                            4096,
                            jitter_buffer,
                            10,
                            &jitter_buffer->task_handle,
                            1);
#endif  /* (configSUPPORT_STATIC_ALLOCATION == 1) && defined(CONFIG_SPIRAM_BOOT_INIT) */

    if (jitter_buffer->task_handle == NULL) {
        ESP_LOGE(TAG, "Create jitter buffer task failed");
        goto __err;
    }

    return (jitter_buffer_handle_t)jitter_buffer;

__err:
    if (jitter_buffer->event_group_ack != NULL) {
        vEventGroupDelete(jitter_buffer->event_group_ack);
    }
    vEventGroupDelete(jitter_buffer->event_group);
    vSemaphoreDelete(jitter_buffer->mutex);
    free(jitter_buffer->buffer);
    free(jitter_buffer->frame_buffer);
    free(jitter_buffer);
    return NULL;
}

esp_err_t jitter_buffer_destroy(jitter_buffer_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    jitter_buffer_t *jitter_buffer = (jitter_buffer_t *)handle;

    xEventGroupSetBits(jitter_buffer->event_group, JITTER_BUFFER_EVENT_EXIT);
    if (jitter_buffer->event_group_ack != NULL) {
        xEventGroupWaitBits(jitter_buffer->event_group_ack, JITTER_BUFFER_EVENT_ACK,
                           pdFALSE, pdFALSE, pdMS_TO_TICKS(500));
    }

    if (jitter_buffer->buffer != NULL) {
        free(jitter_buffer->buffer);
        jitter_buffer->buffer = NULL;
    }
    if (jitter_buffer->frame_buffer != NULL) {
        free(jitter_buffer->frame_buffer);
        jitter_buffer->frame_buffer = NULL;
    }
    if (jitter_buffer->mutex != NULL) {
        vSemaphoreDelete(jitter_buffer->mutex);
        jitter_buffer->mutex = NULL;
    }
    if (jitter_buffer->event_group != NULL) {
        vEventGroupDelete(jitter_buffer->event_group);
        jitter_buffer->event_group = NULL;
    }
    if (jitter_buffer->event_group_ack != NULL) {
        vEventGroupDelete(jitter_buffer->event_group_ack);
        jitter_buffer->event_group_ack = NULL;
    }
    jitter_buffer->task_handle = NULL;
    free(handle);
    return ESP_OK;
}

esp_err_t jitter_buffer_reset(jitter_buffer_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    jitter_buffer_t *jitter_buffer = (jitter_buffer_t *)handle;
    if (xSemaphoreTake(jitter_buffer->mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Jitter buffer reset: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    jitter_buffer->write_pos = 0;
    jitter_buffer->read_pos = 0;
    jitter_buffer->data_size = 0;
    jitter_buffer->state = JITTER_STATE_BUFFERING;
    s_post_state_event(jitter_buffer, JITTER_EVENT_BUFFERING);
    xSemaphoreGive(jitter_buffer->mutex);
    return ESP_OK;
}

esp_err_t jitter_buffer_write(jitter_buffer_handle_t handle, const uint8_t *data, size_t len)
{
    if (handle == NULL) {
        ESP_LOGW(TAG, "Jitter buffer write: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    jitter_buffer_t *jitter_buffer = (jitter_buffer_t *)handle;

    if (jitter_buffer->buffer == NULL || jitter_buffer->mutex == NULL) {
        ESP_LOGW(TAG, "Jitter buffer write: buffer or mutex is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(jitter_buffer->mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Jitter buffer write: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    size_t write_len = len;
    if (jitter_buffer->config.with_header) {
        write_len = JITTER_HEADER_LEN + len;  /* 2 字节长度 + payload */
    }

    size_t available_space = jitter_buffer->buffer_size - jitter_buffer->data_size;
    if (write_len > available_space) {
        if (jitter_buffer->config.with_header) {
            /* 带头格式必须按整帧丢弃，否则 read_pos 会错位，后续会把 payload 当长度解析 */
            size_t discarded_frames = 0;
            while (available_space < write_len && jitter_buffer->data_size >= JITTER_HEADER_LEN) {
                uint8_t b0 = jitter_buffer->buffer[jitter_buffer->read_pos];
                uint8_t b1 = jitter_buffer->buffer[(jitter_buffer->read_pos + 1) % jitter_buffer->buffer_size];
                uint16_t L = (uint16_t)((b0 << 8) | b1);
                if (L > jitter_buffer->buffer_size / 2) {
                    break;
                }
                if (jitter_buffer->data_size < JITTER_HEADER_LEN + L) {
                    break;
                }
                jitter_buffer->read_pos = (jitter_buffer->read_pos + JITTER_HEADER_LEN + L) % jitter_buffer->buffer_size;
                jitter_buffer->data_size -= (JITTER_HEADER_LEN + L);
                discarded_frames++;
                available_space = jitter_buffer->buffer_size - jitter_buffer->data_size;
            }
            if (available_space < write_len) {
                size_t discard = write_len - available_space;
                jitter_buffer->read_pos = (jitter_buffer->read_pos + discard) % jitter_buffer->buffer_size;
                jitter_buffer->data_size -= discard;
                ESP_LOGW(TAG, "Jitter buffer overrun: alignment lost, discarded %zu bytes (frames=%zu)",
                         discard, (unsigned long)discarded_frames);
            }
            jitter_buffer->overrun_count++;
            if (discarded_frames > 0) {
                ESP_LOGW(TAG, "Jitter buffer overrun: discarded %zu frame(s), count=%lu, write_len=%zu",
                         (unsigned long)discarded_frames, (unsigned long)jitter_buffer->overrun_count, write_len);
            }
        } else {
            size_t discard = write_len - available_space;
            jitter_buffer->read_pos = (jitter_buffer->read_pos + discard) % jitter_buffer->buffer_size;
            jitter_buffer->data_size -= discard;
            jitter_buffer->overrun_count++;
            ESP_LOGW(TAG, "Jitter buffer overrun: discarded %zu bytes, count=%lu, len=%zu, available_space=%zu",
                     discard, (unsigned long)jitter_buffer->overrun_count, write_len, available_space);
        }
    }

    if (jitter_buffer->config.with_header) {
        uint8_t hdr[JITTER_HEADER_LEN];
        hdr[0] = (uint8_t)((len >> 8) & 0xff);
        hdr[1] = (uint8_t)(len & 0xff);
        s_ring_write(jitter_buffer, hdr, JITTER_HEADER_LEN);
        s_ring_write(jitter_buffer, data, len);
    } else {
        s_ring_write(jitter_buffer, data, len);
    }

    size_t frame_count = jitter_buffer->config.with_header
        ? s_get_frame_count_with_header(jitter_buffer)
        : (jitter_buffer->data_size / jitter_buffer->config.frame_size);

    // 状态机：达到高水位开始播放
    if (jitter_buffer->state == JITTER_STATE_BUFFERING ||
        jitter_buffer->state == JITTER_STATE_UNDERRUN) {
        if (frame_count >= jitter_buffer->config.high_water) {
            jitter_buffer->state = JITTER_STATE_PLAYING;
            s_post_state_event(jitter_buffer, JITTER_EVENT_PLAYING);
            ESP_LOGI(TAG, "Jitter buffer: start playing, frames=%zu", frame_count);
        }
    }

    xSemaphoreGive(jitter_buffer->mutex);
    return ESP_OK;
}

esp_err_t jitter_buffer_start(jitter_buffer_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    jitter_buffer_t *jb = (jitter_buffer_t *)handle;
    jb->state = JITTER_STATE_BUFFERING;  /* 启动后先蓄水，达到 high_water 再播放 */
    s_post_state_event(jb, JITTER_EVENT_BUFFERING);
    if (jb->event_group_ack != NULL) {
        xEventGroupClearBits(jb->event_group_ack, JITTER_BUFFER_EVENT_ACK);
    }
    xEventGroupSetBits(jb->event_group, JITTER_BUFFER_EVENT_START);
    if (jb->event_group_ack != NULL) {
        xEventGroupWaitBits(jb->event_group_ack, JITTER_BUFFER_EVENT_ACK,
                           pdFALSE, pdFALSE, pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Jitter buffer start");
    return ESP_OK;
}

esp_err_t jitter_buffer_stop(jitter_buffer_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    jitter_buffer_t *jb = (jitter_buffer_t *)handle;
    if (jb->event_group_ack != NULL) {
        xEventGroupClearBits(jb->event_group_ack, JITTER_BUFFER_EVENT_ACK);
    }
    xEventGroupSetBits(jb->event_group, JITTER_BUFFER_EVENT_STOP);
    if (jb->event_group_ack != NULL) {
        xEventGroupWaitBits(jb->event_group_ack, JITTER_BUFFER_EVENT_ACK,
                           pdFALSE, pdFALSE, pdMS_TO_TICKS(500));
    }
    return ESP_OK;
}
