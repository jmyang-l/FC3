include"400hz_offset.h"

#define SUB_FIFO_LEN 8
static ImuSubSample_t gyr_fifo[SUB_FIFO_LEN];
static ImuSubSample_t acc_fifo[SUB_FIFO_LEN];
static uint8_t gyr_head = 0, acc_head = 0;

/* 三维向量叉乘 a×b → c */
static inline void cross3(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/* 将新的陀螺角增量 dθ 与时间戳存入 FIFO */
void on_gyro_raw(const float dθ[3], uint64_t ts) {
    gyr_fifo[gyr_head] = (ImuSubSample_t){.dθ = {dθ[0],dθ[1],dθ[2]}, .ts = ts};
    gyr_head = (gyr_head + 1) & (SUB_FIFO_LEN-1);
}

/* 将新的加速度速度增量 dv 与时间戳存入 FIFO */
void on_acc_raw(const float dv[3], uint64_t ts) {
    acc_fifo[acc_head] = (ImuSubSample_t){.dv = {dv[0],dv[1],dv[2]}, .ts = ts};
    acc_head = (acc_head + 1) & (SUB_FIFO_LEN-1);
}


/*────────────────────────────────────────────────────────────
 *  5 子样圆锥补偿（Coning Compensation）
 * 功能
 *   把 5 个离散陀螺角增量 dθ 合成 1 个无漂移的等效角增量 Δθ
 * 参数
 *   s   : 输入子样数组（≥5）
 *   n   : 实际子样数量
 *   out : 输出补偿后的角增量 Δθ（rad）
 * 算法
 *   Δθ = Σdθ + k · Σ(dθ[i-1] × dθ[i])
 *   其中 k = 1/60 为 5 子样最优系数
 *────────────────────────────────────────────────────────────*/
static void coning_5sample(const ImuSubSample_t *s,
                           int n,
                           float out[3])
{
    /* 1. 线性累加：Σdθ */
    float sum[3] = {0};
    for (int i = 0; i < n; i++) {
        sum[0] += s[i].dθ[0];
        sum[1] += s[i].dθ[1];
        sum[2] += s[i].dθ[2];
    }

    /* 2. 计算交叉项 Σ(dθ[i-1] × dθ[i]) */
    float cross[3] = {0};
    for (int i = 1; i < n; i++) {
        float tmp[3];
        cross3(s[i-1].dθ, s[i].dθ, tmp);
        for (int k = 0; k < 3; k++) cross[k] += tmp[k];
    }

    /* 3. 合成最终结果 */
    const float k = 1.0f / 60.0f;   // 5 子样最优系数
    for (int k = 0; k < 3; k++)
        out[k] = sum[k] + k * cross[k];
}


/*────────────────────────────────────────────────────────────
 *  4 子样简化划桨补偿（Sculling Compensation）
 * 功能
 *   把 4 个离散加速度速度增量 dv 与对应陀螺角增量 dθ
 *   合成 1 个无漂移的等效速度增量 Δv
 * 参数
 *   a : 加速度子样数组（≥4）
 *   n : 加速度子样数量
 *   g : 陀螺子样数组（≥4）
 *   m : 陀螺子样数量
 *   out: 输出补偿后的速度增量 Δv（m/s）
 * 算法
 *   Δv = k · Σ(dθ[i] × dv[i])
 *   其中 k = 1/12 为简化 4 子样系数
 *────────────────────────────────────────────────────────────*/
static void sculling_4sample(const ImuSubSample_t *a,
                             int n,
                             const ImuSubSample_t *g,
                             int m,
                             float out[3])
{
    /* 1. 计算交叉项 Σ(dθ[i] × dv[i]) */
    float cross[3] = {0};
    for (int i = 0; i < MIN(n, m); i++) {
        float tmp[3];
        cross3(g[i].dθ, a[i].dv, tmp);
        for (int k = 0; k < 3; k++) cross[k] += tmp[k];
    }

    /* 2. 合成最终结果 */
    const float k = 1.0f / 12.0f;   // 4 子样简化系数
    for (int k = 0; k < 3; k++)
        out[k] = cross[k] * k;
}

/*────────────────────────────────────────────────────────────
 *  从环形 FIFO 提取指定时间窗口内的子样
 * 参数
 *   fifo   : 环形缓冲区首地址
 *   head   : 最新写入位置的索引（写指针）
 *   t0, t1 : 时间窗口 [t0, t1]（µs）
 *   out    : 输出数组，用于存放符合条件的子样
 *   max    : 输出数组最大容量
 * 返回
 *   实际拷贝到 out 中的子样数量
 * 注意
 *   由于 FIFO 是环形，索引通过位与运算 & (SUB_FIFO_LEN-1) 自动回绕
 *────────────────────────────────────────────────────────────*/
static int extract_window(const ImuSubSample_t *fifo,
                          uint8_t head,
                          uint64_t t0,
                          uint64_t t1,
                          ImuSubSample_t *out,
                          int max)
{
    int cnt = 0;
    for (int i = 0; i < max; i++) {
        /* 从最新开始倒序遍历，索引自动回绕 */
        uint8_t idx = (head - 1 - i) & (SUB_FIFO_LEN - 1);
        /* 时间戳落在窗口内才拷贝 */
        if (fifo[idx].ts >= t0 && cnt < max)
            out[cnt++] = fifo[idx];
    }
    return cnt;
}


static uint64_t last_t = 0;
/**
 * @brief  400 Hz IMU 融合任务
 *
 * 每 2.5 ms 被调用一次，完成以下工作：
 * 1. 计算距上次调用的时间间隔 dt
 * 2. 从 FIFO 中提取最近 2.5 ms 内的陀螺/加计子样
 * 3. 用圆锥补偿把离散角增量合成无漂移 Δθ
 * 4. 用划桨补偿把离散速度增量合成无漂移 Δv
 * 5. 将 Δθ, Δv 和 dt 喂给姿态积分器
 */
void imu_400hz_task(void)
{
    /* 当前时间戳 (µs) */
    uint64_t now   = micros();            // 微秒级单调时钟
    uint64_t dt_us = now - last_t;        // 距上次调用的时间间隔
    last_t = now;

    /*----------------------------------------------------------
     * 1. 从环形 FIFO 提取最近 2.5 ms 内的全部子样
     *----------------------------------------------------------*/
    ImuSubSample_t gyr_buf[5];  // 陀螺最多 5 子样 (2000 Hz → 2.5 ms)
    ImuSubSample_t acc_buf[4];  // 加计最多 4 子样 (1600 Hz → 2.5 ms)

    int n_g = extract_window(gyr_fifo, gyr_head,
                             now - 2500, now, gyr_buf, 5);
    int n_a = extract_window(acc_fifo, acc_head,
                             now - 2500, now, acc_buf, 4);

    /*----------------------------------------------------------
     * 2. 圆锥补偿：把 5 个陀螺子样 → 1 个无漂移角增量 Δθ
     *----------------------------------------------------------*/
    float dθ_corr[3];
    coning_5sample(gyr_buf, n_g, dθ_corr);   // 单位：rad

    /*----------------------------------------------------------
     * 3. 划桨补偿：把 4 个加计子样 → 1 个无漂移速度增量 Δv
     *----------------------------------------------------------*/
    float dv_corr[3];
    sculling_4sample(acc_buf, n_a, dv_corr); // 单位：m/s

    /*----------------------------------------------------------
     * 4. 喂给 400 Hz 姿态积分器
     *----------------------------------------------------------*/
    attitude_update(dθ_corr, dv_corr, dt_us * 1e-6f); // dt 单位：s
}



