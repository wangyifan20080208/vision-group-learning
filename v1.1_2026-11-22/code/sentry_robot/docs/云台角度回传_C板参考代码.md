# 云台角度回传 —— C 板侧参考代码

> 目标：把 GM6020(yaw) 的绝对角度发给 NUC，让 ROS 能发布
> `base_footprint → gimbal_yaw` 的 TF。
>
> 平台：RoboMaster 开发板 C 型（STM32F407），HAL 库。
> 链路：`GM6020 --CAN--> C 板 --UART--> NUC(USB 转串口)`

---

## 一、先看清楚 6020 的反馈报文

GM6020 收到电流/电压指令后，会在**同一 CAN ID 上回一帧 8 字节**：

| 字节 | 内容 | 说明 |
|---|---|---|
| 0-1 | 转子机械角度 | 0~8191，对应 0~360°，**每圈归零** |
| 2-3 | 转子转速 | rpm，int16，注意符号 |
| 4-5 | 实际转矩电流 | 原始值 |
| 6 | 温度 | ℃ |
| 7 | 保留 | |

**关键点：字节 0-1 是"每圈归零"的**，转过 8191 会跳回 0。
直接用它会每圈跳 360°，必须解算成连续角度。

---

## 二、解零点跳变（这一段最容易错）

```c
/* gimbal_feedback.h */
#ifndef GIMBAL_FEEDBACK_H
#define GIMBAL_FEEDBACK_H

#include <stdint.h>

#define GM6020_CPR      8192.0f
#define TWO_PI_F        6.28318530718f

typedef struct {
    uint16_t last_raw;      /* 上一帧原始角度 0~8191 */
    int32_t  total_cnt;     /* 累计计数（可正可负，可多圈） */
    float    yaw_rad;       /* 连续角度（弧度） */
    float    yaw_rate_rps;  /* 角速度（弧度/秒） */
    int16_t  rpm;           /* 原始转速 */
    uint8_t  temp;          /* 温度 */
    uint8_t  inited;
} GimbalState;

extern GimbalState g_gimbal;

void gimbal_update(uint8_t *rx_data);
void gimbal_reset(void);

#endif
```

```c
/* gimbal_feedback.c */
#include "gimbal_feedback.h"

GimbalState g_gimbal = {0};

void gimbal_reset(void)
{
    g_gimbal.inited = 0;
    g_gimbal.last_raw = 0;
    g_gimbal.total_cnt = 0;
    g_gimbal.yaw_rad = 0.0f;
    g_gimbal.yaw_rate_rps = 0.0f;
}

/*
 * rx_data: 8 字节 CAN 反馈
 *
 * 解零点跳变的原理：
 *   相邻两帧的原始值差如果超过半圈（4096），说明跨了零，
 *   要把差值补/减一整圈（8192），才能得到真实的增量。
 */
void gimbal_update(uint8_t *rx_data)
{
    uint16_t raw = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    int16_t  rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);

    if (!g_gimbal.inited) {
        g_gimbal.last_raw  = raw;
        g_gimbal.total_cnt = 0;
        g_gimbal.inited    = 1;
    } else {
        int32_t delta = (int32_t)raw - (int32_t)g_gimbal.last_raw;

        /* 正向跨零：8191 → 0，差值是很大的负数，补一圈 */
        if (delta < -4096)      delta += 8192;
        /* 反向跨零：0 → 8191，差值是很大的正数，减一圈 */
        else if (delta > 4096)  delta -= 8192;

        g_gimbal.total_cnt += delta;
        g_gimbal.last_raw   = raw;
    }

    g_gimbal.yaw_rad = (float)g_gimbal.total_cnt * (TWO_PI_F / GM6020_CPR);

    /* rpm → rad/s：一圈 2π，一分钟 60 秒 */
    g_gimbal.yaw_rate_rps = (float)rpm * (TWO_PI_F / 60.0f);
    g_gimbal.rpm  = rpm;
    g_gimbal.temp = rx_data[6];
}
```

### 三个容易踩的点

| 坑 | 症状 | 解法 |
|---|---|---|
| **不初始化 last_raw** | 上电第一帧就产生一个巨大的假增量 | `inited` 标志，第一帧只记录不累加 |
| **阈值用 4096 但比较方向写反** | 转一圈后角度突变 ±360° | 照抄上面两个 `if`，注意符号 |
| **用 float 存 total_cnt** | 长时间运行后精度丢失（>1600 万计数） | 用 `int32_t`，上限约 26 万圈 |

---

## 三、串口发送（21 字节定长帧）

```c
/* gimbal_uart.h */
#include <stdint.h>

void gimbal_send_frame(UART_HandleTypeDef *huart,
                       float yaw_rad,
                       float pitch_rad,
                       float yaw_rate_rps,
                       uint32_t tick_ms);
```

```c
/* gimbal_uart.c */
#include "gimbal_uart.h"
#include <string.h>

/*
 * 帧格式（小端）：
 *   0    0xA5          帧头
 *   1    0x5A          帧头
 *   2    16            负载长度
 *   3    0x01          类型：云台角度
 *   4..7   float yaw        连续角度 rad
 *   8..11  float pitch      连续角度 rad
 *   12..15 float yaw_rate   rad/s
 *   16..19 uint32 tick_ms
 *   20    xor           前 20 字节异或
 *
 * 为什么用定长 + 异或：C 板不需要 sprintf（省 flash、省时间），
 * NUC 侧也容易在乱码里重新对齐帧头。
 */
void gimbal_send_frame(UART_HandleTypeDef *huart,
                       float yaw_rad, float pitch_rad,
                       float yaw_rate_rps, uint32_t tick_ms)
{
    uint8_t buf[21];
    buf[0] = 0xA5;
    buf[1] = 0x5A;
    buf[2] = 16;
    buf[3] = 0x01;
    memcpy(&buf[4],  &yaw_rad,        4);
    memcpy(&buf[8],  &pitch_rad,      4);
    memcpy(&buf[12], &yaw_rate_rps,   4);
    memcpy(&buf[16], &tick_ms,        4);

    uint8_t x = 0;
    for (int i = 0; i < 20; i++) x ^= buf[i];
    buf[20] = x;

    HAL_UART_Transmit_DMA(huart, buf, 21);   /* 或 _IT，别用阻塞版 */
}
```

> **注意**：`memcpy` 依赖小端。STM32 是小端，PC 也是小端，没问题。
> 如果换成大端平台要改成手动移位。

---

## 四、在主循环里怎么调

```c
/* 100 Hz 定时任务里（或 CAN 接收回调里） */
void gimbal_task_100hz(void)
{
    gimbal_send_frame(&huart1,
                      g_gimbal.yaw_rad,
                      0.0f,                 /* pitch 暂时不用，填 0 */
                      g_gimbal.yaw_rate_rps,
                      HAL_GetTick());
}
```

**建议 100 Hz 发送**（协议够小，21 字节 × 100 = 2.1 KB/s，921600 波特率毫无压力）。

---

## 五、波特率与接线

| 项 | 值 |
|---|---|
| 波特率 | **921600**（或 460800，越高的角度越新） |
| 数据位/停止位 | 8 / 1，无校验 |
| 接线 | C 板 TX → NUC RX，C 板 RX → NUC TX，共地 |
| NUC 侧设备 | `/dev/ttyUSB0`（CH340），可用 `ls /dev/ttyUSB*` 确认 |

**如果串口设备名每次都变**（插拔导致），加一条 udev 规则固定：

```bash
# /etc/udev/rules.d/99-gimbal.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gimbal"
```

然后用 `port:=/dev/gimbal`。

---

## 六、上电顺序与安全

| 情况 | 后果 | 建议 |
|---|---|---|
| C 板未上电就启动 ROS | TF 缺失，`livox_scan` 报 TF 超时 | 节点有 warn，等 C 板起来自动恢复 |
| 串口断开 | 云台角度冻结在最后一个值 | 节点每 5 秒告警一次 |
| **上电瞬间 6020 角度是随机的** | 第一次 TF 可能和真车朝向差很多 | 上电后让云台回零位再启动导航 |

**最后一条很重要**：6020 的零位是机械安装决定的，上电时不知道云台真实朝哪。
所以**开机流程应该是**：云台先回零 → 再启动导航。
