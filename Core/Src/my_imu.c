#include "my_imu.h"
#include <string.h>

/* ---------- 环形缓冲 ---------- */
static volatile uint8_t  s_rx_buffer[IMU_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_write_index = 0;
static volatile uint16_t s_rx_read_index  = 0;

static inline uint16_t _rxbuf_next(uint16_t index) {
    return (uint16_t)((index + 1u) % IMU_UART_RX_BUF_SIZE);
}

static inline int _rxbuf_is_empty(void) {
    return s_rx_write_index == s_rx_read_index;
}

static inline void _rxbuf_push(uint8_t byte_value) {
    uint16_t next_index = _rxbuf_next(s_rx_write_index);
    if (next_index == s_rx_read_index) {
        s_rx_read_index = _rxbuf_next(s_rx_read_index); 
    }
    s_rx_buffer[s_rx_write_index] = byte_value;
    s_rx_write_index = next_index;
}

static inline int _rxbuf_pop(uint8_t *out_byte) {
    if (_rxbuf_is_empty()) return -1;
    *out_byte = s_rx_buffer[s_rx_read_index];
    s_rx_read_index = _rxbuf_next(s_rx_read_index);
    return 0;
}

/* ---------- 内部状态缓存 ---------- */
static volatile float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;

// 核心解析：根据新协议（小端模式，IEEE-754浮点数直接传输）
static void _parse_frame_data(uint8_t frame_function, const uint8_t *frame_data) {
    if (frame_function == IMU_FUNC_EULER) {
        // 数据包：小端序拼接，用 uint32_t 无符号整型接住原生的二进制比特流
        uint32_t raw_roll  = (frame_data[3]<<24)  | (frame_data[2]<<16)  | (frame_data[1]<<8)  | frame_data[0];
        uint32_t raw_pitch = (frame_data[7]<<24)  | (frame_data[6]<<16)  | (frame_data[5]<<8)  | frame_data[4];
        uint32_t raw_yaw   = (frame_data[11]<<24) | (frame_data[10]<<16) | (frame_data[9]<<8)  | frame_data[8];

        // 核心修正：利用指针类型转换，告诉单片机“把这串 32 位的 0101 当作 float 来看待”
        s_roll  = *(float*)&raw_roll;
        s_pitch = *(float*)&raw_pitch;
        s_yaw   = *(float*)&raw_yaw;
        
        // 不需要再除以 1000，因为协议传输的已经是真实的浮点数弧度值了！
    }
}

/* ---------- 供外部接收调用的接口 ---------- */
// 接收多字节（适合配合DMA）
void IMU_UART_RxBytes(uint8_t *data, uint16_t len) {
    if (!data || len == 0) return;
    for (uint16_t i = 0; i < len; ++i) {
        _rxbuf_push(data[i]);
    }
}

// 接收单字节（适合普通接收中断）
void IMU_UART_RxByte_Callback(uint8_t data) {
    _rxbuf_push(data);
}

/* ---------- 核心解析状态机 ---------- */
void IMU_UART_Process(void) {
    enum {
        RX_STATE_EXPECT_HEAD1 = 0,
        RX_STATE_EXPECT_HEAD2,
        RX_STATE_EXPECT_LENGTH,
        RX_STATE_EXPECT_FUNCTION,
        RX_STATE_COLLECT_DATA
    };

    static uint8_t  rx_state = RX_STATE_EXPECT_HEAD1;
    static uint8_t  frame_length = 0;
    static uint8_t  frame_function = 0;
    static uint8_t  frame_buffer[64]; 
    static uint16_t frame_index = 0;
    static uint8_t  current_checksum_sum = 0; // 用于实时累加校验和
    uint8_t current_byte = 0;

    while (_rxbuf_pop(&current_byte) == 0) {
        switch (rx_state) {
        case RX_STATE_EXPECT_HEAD1:
            if (current_byte == FRAME_HEAD1) {
                rx_state = RX_STATE_EXPECT_HEAD2;
                current_checksum_sum = current_byte; // 协议要求校验和包含包头
            }
            break;

        case RX_STATE_EXPECT_HEAD2:
            if (current_byte == FRAME_HEAD2) {
                rx_state = RX_STATE_EXPECT_LENGTH;
                current_checksum_sum += current_byte;
            } else {
                rx_state = RX_STATE_EXPECT_HEAD1;
            }
            break;

        case RX_STATE_EXPECT_LENGTH:
            frame_length = current_byte;
            current_checksum_sum += current_byte;
            rx_state = RX_STATE_EXPECT_FUNCTION;
            break;

        case RX_STATE_EXPECT_FUNCTION:
            frame_function = current_byte;
            current_checksum_sum += current_byte;
            frame_index = 0;
            rx_state = RX_STATE_COLLECT_DATA;
            break;

      case RX_STATE_COLLECT_DATA: {
            // 修正：总长度为 frame_length，已经收了4个字节 (Head1, Head2, Length, Function)
            // 所以剩下的字节数为 frame_length - 4 (包含数据段 + 1字节校验和)
            uint16_t total_remaining = (frame_length >= 5) ? (uint16_t)(frame_length - 4) : 0;
            // 存入数据段
            frame_buffer[frame_index++] = current_byte;
            
            // 如果还没收到最后一个字节（校验和），则累加校验
            if (frame_index < total_remaining) {
                current_checksum_sum += current_byte;
            } else {
                // 收齐了，最后一个字节就是校验和
                uint8_t received_checksum = current_byte; 
                
                // 校验通过 (Sum == Checksum)
                if (current_checksum_sum == received_checksum) {
                    _parse_frame_data(frame_function, frame_buffer);
                }
                // 无论成功失败，重置状态机等待下一帧
                rx_state = RX_STATE_EXPECT_HEAD1;
            }
        } break;
        
        default:
            rx_state = RX_STATE_EXPECT_HEAD1;
            break;
        }
    }
}

/* ---------- 提取数据接口 ---------- */
int IMU_UART_GetEuler(float out[3]) {
    if (!out) return -1;
    const float RAD2DEG = 57.2957795f; // 弧度转角度系数
    out[0] = s_roll  * RAD2DEG;
    out[1] = s_pitch * RAD2DEG;
    out[2] = s_yaw   * RAD2DEG;
    return 0;
}

float IMU_Get_Yaw(void) {
    const float RAD2DEG = 57.2957795f;
    return s_yaw * RAD2DEG;
}