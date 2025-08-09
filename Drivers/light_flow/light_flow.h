#ifndef LIGHT_FLOW_LIGHT_FLOW_H_
#define LIGHT_FLOW_LIGHT_FLOW_H_



#pragma once

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "flow_lowpass.h"

#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7

/*
    消息ID定义
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,     // 测距传感器
};

/*
    消息结构体定义
*/
typedef struct
{
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

/*
    数据负载定义
*/
#pragma pack (1)
// 测距传感器
typedef struct
{
    uint32_t  time_ms;			    // 系统时间 ms
    uint32_t  distance;			    // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t   strength;	            // 信号强度
    uint8_t   precision;	        // 精度
    uint8_t   tof_status;	        // 状态
    uint8_t  reserved1;			    // 预留
    int16_t   flow_vel_x;	        // 光流速度x轴
    int16_t   flow_vel_y;	        // 光流速度y轴
    uint8_t   flow_quality;	        // 光流质量
    uint8_t   flow_status;	        // 光流状态
    uint16_t  reserved2;	        // 预留
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack ()

extern MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
extern MICOLINK_PAYLOAD_RANGE_SENSOR_t analysed_payload;




void micolink_decode(uint8_t data);
void convert_endian(MICOLINK_PAYLOAD_RANGE_SENSOR_t *src, MICOLINK_PAYLOAD_RANGE_SENSOR_t *dst);
void check_time_stability_simple(void (*handler)(void));


#endif /* LIGHT_FLOW_LIGHT_FLOW_H_ */
