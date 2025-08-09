#include "flow_lowpass.h"

// 全局上一帧输出
static float last_x = 0.0f;

// alpha ∈ [0,1]，越小越平滑
float flow_lpf(float raw, float alpha)
{
    float out = last_x + alpha * (raw - last_x);
    last_x  = out;
    return out;
}
