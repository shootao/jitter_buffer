/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_system.h"
#include "jitter_buffer.h"

static jitter_buffer_handle_t jitter_buffer_handle;

static const char *TAG = "JITTER_BUFFER_EXAMPLE";

/* 测试 case：0=创建/销毁泄漏, 1=reset, 2=start/stop, 3=正常跑, 其他=依次跑 0,1,2 再跑 3 */
#ifndef JITTER_EXAMPLE_CASE
#define JITTER_EXAMPLE_CASE 3
#endif

/* 校验用：期望的下一个输出帧序号（大端 4 字节），用于验证数据顺序与内容正确 */
#define PAYLOAD_HEADER_LEN 4
#define PAYLOAD_PATTERN_BYTE 0x55

static uint32_t s_expected_seq;
static size_t   s_total_bytes_written;
static size_t   s_total_bytes_received;
static uint32_t s_output_ok_count;
static uint32_t s_output_err_count;

static bool is_silence_frame(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (data[i] != 0) {
            return false;
        }
    }
    return true;
}

static void on_output_data(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }

    s_total_bytes_received += len;

    /* 静音帧（BUFFERING 阶段 jitter buffer 输出的填充）：不参与序号校验 */
    if (is_silence_frame(data, len)) {
        return;
    }

    /* 至少要有序号 4 字节才能校验 */
    if (len < PAYLOAD_HEADER_LEN) {
        ESP_LOGW(TAG, "on_output_data: len=%zu too short, skip verify", len);
        return;
    }

    uint32_t seq = (uint32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    if (seq != s_expected_seq) {
        ESP_LOGE(TAG, "on_output_data: seq mismatch expected=%lu got=%lu", (unsigned long)s_expected_seq, (unsigned long)seq);
        s_output_err_count++;
    } else {
        s_output_ok_count++;
    }
    s_expected_seq = seq + 1;

    for (size_t i = PAYLOAD_HEADER_LEN; i < len; i++) {
        if (data[i] != PAYLOAD_PATTERN_BYTE) {
            ESP_LOGE(TAG, "on_output_data: pattern error at offset %zu expected=0x%02x got=0x%02x",
                     i, (unsigned)PAYLOAD_PATTERN_BYTE, (unsigned)data[i]);
            s_output_err_count++;
            break;
        }
    }

    /* 每 50 帧或出错时打印统计，避免刷屏 */
    if (s_output_err_count > 0 || (s_output_ok_count + s_output_err_count) % 50 == 0) {
        ESP_LOGI(TAG, "on_output_data: %zu bytes, ok=%lu err=%lu, total_rx=%zu",
                 len, (unsigned long)s_output_ok_count, (unsigned long)s_output_err_count, s_total_bytes_received);
    }
}

static void fill_write_buffer(uint8_t *buf, size_t len, uint32_t seq)
{
    if (len < PAYLOAD_HEADER_LEN) {
        return;
    }
    buf[0] = (uint8_t)(seq >> 24);
    buf[1] = (uint8_t)(seq >> 16);
    buf[2] = (uint8_t)(seq >> 8);
    buf[3] = (uint8_t)(seq & 0xff);
    for (size_t i = PAYLOAD_HEADER_LEN; i < len; i++) {
        buf[i] = PAYLOAD_PATTERN_BYTE;
    }
}

static void on_output_data_noop(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
}

#define LEAK_TEST_LOOPS 20

/** Case 0: 反复创建/销毁，看堆是否回收，用于检查内存泄漏 */
static void run_case_create_destroy_leak(void)
{
    ESP_LOGI(TAG, "========== Case 0: create/destroy leak test (loops=%d) ==========", LEAK_TEST_LOOPS);

    jitter_buffer_config_t config = {
        .on_output_data = on_output_data_noop,
        .buffer_size = 10 * 1024,
        .frame_size = 512,
        .low_water = 10,
        .high_water = 20,
        .frame_interval = 20,
        .with_header = true,
    };

    size_t free_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "free heap before: %zu", free_before);

    
    for (int i = 0; i < LEAK_TEST_LOOPS; i++) {
        ESP_LOGI(TAG, "create jitter_buffer_handle_t at loop %d", i);
        jitter_buffer_handle_t h = jitter_buffer_create(&config);
        if (h == NULL) {
            ESP_LOGE(TAG, "create failed at loop %d", i);
            break;
        }
        esp_err_t ret = jitter_buffer_destroy(h);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "destroy failed at loop %d", i);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    size_t free_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "free heap after:  %zu (diff=%d)", free_after, (int)(free_after - free_before));

    if (free_after < free_before && (free_before - free_after) > 1024) {
        ESP_LOGW(TAG, "possible leak: lost %zu bytes", free_before - free_after);
    } else {
        ESP_LOGI(TAG, "no significant leak detected");
    }
}

/** Case 1: 创建 -> start -> 写若干帧 -> reset -> 再写 -> destroy，检查 reset 与回收 */
static void run_case_reset(void)
{
    ESP_LOGI(TAG, "========== Case 1: reset test ==========");

    jitter_buffer_config_t config = {
        .on_output_data = on_output_data,
        .buffer_size = 10 * 1024,
        .frame_size = 512,
        .low_water = 10,
        .high_water = 20,
        .frame_interval = 20,
        .with_header = true,
    };

    jitter_buffer_handle_t h = jitter_buffer_create(&config);
    if (h == NULL) {
        ESP_LOGE(TAG, "create failed");
        return;
    }

    size_t free_after_create = esp_get_free_heap_size();
    jitter_buffer_start(h);

    uint8_t *data = (uint8_t *)malloc(512);
    if (data == NULL) {
        jitter_buffer_destroy(h);
        ESP_LOGE(TAG, "malloc failed");
        return;
    }

    s_expected_seq = 0;
    s_output_ok_count = 0;
    s_output_err_count = 0;

    for (uint32_t i = 0; i < 30; i++) {
        fill_write_buffer(data, 512, i);
        jitter_buffer_write(h, data, 512);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    ESP_LOGI(TAG, "reset (after 30 writes)");
    jitter_buffer_reset(h);

    for (uint32_t i = 30; i < 60; i++) {
        fill_write_buffer(data, 512, i);
        jitter_buffer_write(h, data, 512);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "after reset: ok=%lu err=%lu", (unsigned long)s_output_ok_count, (unsigned long)s_output_err_count);

    free(data);
    esp_err_t ret = jitter_buffer_destroy(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "destroy failed");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    size_t free_after_destroy = esp_get_free_heap_size();
    ESP_LOGI(TAG, "free after destroy: %zu (vs after create: %zu)", free_after_destroy, free_after_create);
}

/** Case 2: start -> 写若干帧 -> stop -> 再 start -> 写若干帧 -> destroy，检查 start/stop 与回收 */
static void run_case_start_stop(void)
{
    ESP_LOGI(TAG, "========== Case 2: start/stop test ==========");

    jitter_buffer_config_t config = {
        .on_output_data = on_output_data,
        .buffer_size = 10 * 1024,
        .frame_size = 512,
        .low_water = 10,
        .high_water = 20,
        .frame_interval = 20,
        .with_header = true,
    };

    jitter_buffer_handle_t h = jitter_buffer_create(&config);
    if (h == NULL) {
        ESP_LOGE(TAG, "create failed");
        return;
    }

    uint8_t *data = (uint8_t *)malloc(512);
    if (data == NULL) {
        jitter_buffer_destroy(h);
        ESP_LOGE(TAG, "malloc failed");
        return;
    }

    s_expected_seq = 0;
    s_output_ok_count = 0;
    s_output_err_count = 0;

    ESP_LOGI(TAG, "start (first time)");
    jitter_buffer_start(h);
    for (uint32_t i = 0; i < 25; i++) {
        fill_write_buffer(data, 512, i);
        jitter_buffer_write(h, data, 512);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "stop (first time), ok=%lu err=%lu", (unsigned long)s_output_ok_count, (unsigned long)s_output_err_count);

    jitter_buffer_stop(h);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 再次 start，续写；序号从 25 起 */
    ESP_LOGI(TAG, "start (second time)");
    jitter_buffer_start(h);
    for (uint32_t i = 25; i < 55; i++) {
        fill_write_buffer(data, 512, i);
        jitter_buffer_write(h, data, 512);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "after start/stop/start: ok=%lu err=%lu", (unsigned long)s_output_ok_count, (unsigned long)s_output_err_count);

    jitter_buffer_stop(h);
    free(data);
    esp_err_t ret = jitter_buffer_destroy(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "destroy failed");
    }
    ESP_LOGI(TAG, "start/stop case done");
}

/** Case 3: 正常跑数据 + on_output_data 校验 */
static void run_case_normal(void)
{
    ESP_LOGI(TAG, "========== Case 3: normal run with data verify ==========");

    jitter_buffer_config_t config = {
        .on_output_data = on_output_data,
        .buffer_size = 10 * 1024,
        .frame_size = 512,
        .low_water = 10,
        .high_water = 20,
        .frame_interval = 20,
        .with_header = true,
    };
    jitter_buffer_handle = jitter_buffer_create(&config);
    if (jitter_buffer_handle == NULL) {
        ESP_LOGE(TAG, "Jitter buffer create failed");
        return;
    }
    jitter_buffer_start(jitter_buffer_handle);

    uint8_t *data = (uint8_t *)malloc(512);
    if (data == NULL) {
        ESP_LOGE(TAG, "malloc write buffer failed");
        return;
    }

    s_expected_seq = 0;
    s_total_bytes_written = 0;
    s_total_bytes_received = 0;
    s_output_ok_count = 0;
    s_output_err_count = 0;

    uint32_t write_seq = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));

        fill_write_buffer(data, 512, write_seq);
        write_seq++;

        esp_err_t ret = jitter_buffer_write(jitter_buffer_handle, data, 512);
        if (ret == ESP_OK) {
            s_total_bytes_written += 512;
        }
    }
}

void app_main(void)
{
#if (JITTER_EXAMPLE_CASE == 0)
    run_case_create_destroy_leak();
#elif (JITTER_EXAMPLE_CASE == 1)
    run_case_reset();
#elif (JITTER_EXAMPLE_CASE == 2)
    run_case_start_stop();
#elif (JITTER_EXAMPLE_CASE == 3)
    run_case_normal();
#else
    /* 默认：先跑 0,1,2 再跑正常 case 3 */
    run_case_create_destroy_leak();
    vTaskDelay(pdMS_TO_TICKS(200));
    run_case_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
    run_case_start_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    run_case_normal();
#endif
}
