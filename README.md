# Jitter Buffer

音频/流数据的抖缓冲组件，用于平滑网络或生产端的抖动，保证输出稳定。

## 特性

- 环形缓冲，高/低水位状态机（BUFFERING → PLAYING → UNDERRUN）
- 支持固定帧长（PCM）和变长帧（with_header 模式）
- 状态事件通过 `esp_event` 通知（可选）
- start/stop 带 ACK 同步确认

## 基本用法

```c
#include "jitter_buffer.h"

void on_output_data(const uint8_t *data, size_t len) {
    // 输出到扬声器或下游
}

jitter_buffer_config_t config = {
    .on_output_data = on_output_data,
    .buffer_size = 11 * 1024,
    .frame_size = 512,
    .frame_interval = 20,
    .high_water = 20,
    .low_water = 10,
    .with_header = false,
};
jitter_buffer_handle_t h = jitter_buffer_create(&config);
jitter_buffer_start(h);

// 写入数据
jitter_buffer_write(h, data, len);

jitter_buffer_stop(h);
jitter_buffer_destroy(h);
```

## 配置说明

| 参数 | 说明 |
|------|------|
| `buffer_size` | 环形缓冲大小（字节） |
| `frame_size` | 固定帧长（无头）或单帧 payload 上限（with_header） |
| `frame_interval` | 输出间隔（ms） |
| `high_water` | 达到此帧数开始播放 |
| `low_water` | 低于此帧数进入欠载 |
| `with_header` | true: 变长帧，存储为 [2 字节大端长度][payload] |

## 示例

`examples/simple_example` 包含创建/销毁、reset、start/stop、正常跑数据等测试用例。
