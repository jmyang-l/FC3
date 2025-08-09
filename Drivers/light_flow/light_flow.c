#include "light_flow.h"
#include "usart.h"

    // 定义范围传感器数据载荷结构体
    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

    MICOLINK_PAYLOAD_RANGE_SENSOR_t analysed_payload;
//——————————光流payload数据处理部分—————————

// 交换无符号16位整数的字节序
static void swap_uint16(uint16_t *val) {
    *val = (*val >> 8) | (*val << 8);
}

// 交换有符号16位整数的字节序
static void swap_int16(int16_t *val) {
    // 强制转换为无符号类型进行位操作，保持符号不变
    uint16_t *uval = (uint16_t*)val;
    *uval = (*uval >> 8) | (*uval << 8);
}

// 交换无符号32位整数的字节序
static void swap_uint32(uint32_t *val) {
    *val = (*val >> 24) | ((*val >> 8) & 0xFF00) |
           ((*val << 8) & 0xFF0000) | (*val << 24);
}

// 转换整个结构体的字节序
void convert_endian(MICOLINK_PAYLOAD_RANGE_SENSOR_t *src, MICOLINK_PAYLOAD_RANGE_SENSOR_t *dst) {
    // 复制原始数据到目标结构体
    memcpy(dst, src, sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t));

    // 转换32位无符号整数成员
    swap_uint32(&dst->time_ms);
    swap_uint32(&dst->distance);

    // 转换16位有符号整数成员
    swap_int16(&dst->flow_vel_x);
    swap_int16(&dst->flow_vel_y);

    // 转换16位无符号整数成员
    swap_uint16(&dst->reserved2);

    // 注意：uint8_t类型成员不需要字节序转换，因为它们只有一个字节
}



//——————————光流Micolink协议读取部分——————————
/*
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
光流速度值单位：cm/s@1m
飞控中只需要将光流速度值*高度，即可得到真实水平位移速度
计算公式：实际速度(cm/s)=光流速度*高度(m)
*/

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);



void micolink_decode(uint8_t data)
{
    static MICOLINK_MSG_t msg;

    // 解析接收到的字符，如果解析失败则直接返回
    if(micolink_parse_char(&msg, data) == false)
        return;

    // 根据消息ID处理不同的消息类型
    switch(msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
        	if(msg.payload[10] == 1 && msg.payload[17] == 1)
        	{
                // 将消息载荷数据复制到payload结构体中
                memcpy(&payload, msg.payload, msg.len);

//                convert_endian(&payload, &analysed_payload);
        	}


            /*
                此处可获取传感器数据:

                距离        = payload.distance;
                强度        = payload.strength;
                精度        = payload.precision;
                距离状态    = payload.tof_status;
                光流速度x轴 = payload.flow_vel_x;
                光流速度y轴 = payload.flow_vel_y;
                光流质量    = payload.flow_quality;
                光流状态    = payload.flow_status;
            */

//            HAL_UART_Transmit(&huart4, (uint8_t*)&payload, sizeof(payload), 100);


            break;
        }

        default:
            break;
        }
}

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    // 计算消息的实际长度加上头部和尾部的固定长度（6字节）
    uint8_t length = msg->len + 6;
    // 创建一个临时数组用于存储消息的所有字节
    uint8_t temp[MICOLINK_MAX_LEN];
    // 初始化校验和为0
    uint8_t checksum = 0;

    // 将消息的所有字节复制到临时数组中
    memcpy(temp, msg, length);

    // 遍历临时数组中的每个字节，累加到校验和中
    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    // 检查计算得到的校验和是否与消息中的校验和相等
    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:     //帧头
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1:     // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;

    case 2:     // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;

    case 3:     // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;

    case 4:     // 包序列
        msg->seq = data;
        msg->status++;
        break;

    case 5:     // 负载长度
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6:     // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7:     // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}



// 静态变量记录上次时间和状态
static uint32_t last_time_ms = 0;
static bool first_call = true;  // 首次调用标志

/**
 * 检测analysed_payload.time_ms是否在5ms内保持不变
 * 需在定时器中断中每1ms调用一次
 */
void check_time_stability_simple(void (*handler)(void)) {
    static uint32_t stable_counter = 0;

    // 首次调用初始化
    if (first_call) {
        last_time_ms = analysed_payload.time_ms;
        first_call = false;
        return;
    }

    // 判断time_ms是否变化
    if (analysed_payload.time_ms == last_time_ms) {
        stable_counter++;
        if (stable_counter >= 5) {  // 5ms = 5 * 1ms
            if (handler != NULL) {
                handler();
            }
            stable_counter = 0;  // 重置计数器
        }
    } else {
        // 时间变化，重置计数器
        last_time_ms = analysed_payload.time_ms;
        stable_counter = 0;
    }
}
