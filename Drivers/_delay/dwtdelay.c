#include "dwtdelay.h"

/* ===================================================================
 * 1. DWT 初始化（仅需执行一次）
 * 打开 DWT 模块总闸，清零 CYCCNT 与 EXCCNT，启动周期计数器
 * =================================================================== */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 置位 TRCENA，使能 DWT/ITM/Trace
    DWT->CYCCNT  = 0;                                // 清零 32 位 CPU 周期计数器
    DWT->EXCCNT  = 0;                                // 清零 8 位“异常/中断占用周期”计数器
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;           // 置位 CYCCNTENA，启动 CYCCNT
}

/* ===================================================================
 * 2. 高精度测量函数执行时间（含中断开销拆分）
 * 利用 CYCCNT 测总周期，EXCCNT 测中断周期，二者差值即函数本体周期
 * 要求：在测量期间不得关闭 DEMCR 时钟，且 EXCCNT 不能溢出 255
 * =================================================================== */
void measure_my_func(void (*f)(void))//无参数函数测量
{
    uint32_t cy_start  = DWT->CYCCNT;   // 记录总周期起点
    uint32_t exc_start = DWT->EXCCNT;   // 记录中断周期起点

    /* ↓↓↓ 把要测试的函数放在这里，可包含会触发中断的代码 ↓↓↓ */
    f();

    uint32_t cy_end  = DWT->CYCCNT;     // 记录总周期终点
    uint32_t exc_end = DWT->EXCCNT;     // 记录中断周期终点

    /* 3. 计算结果（无符号差值天然抗回卷） */
    uint32_t total_cycles = cy_end  - cy_start;   // 函数 + 中断总耗时
    uint32_t int_cycles   = exc_end - exc_start;  // 仅中断服务程序耗时
    uint32_t func_pure    = total_cycles - int_cycles; // 函数本体净耗时

    /* 4. 输出或保存数据（可按需要改为 UART、SWO、存储器） */
    printf("total = %lu, int = %lu, func = %lu cycles\r\n",
           total_cycles, int_cycles, func_pure);
}

void measure_my_func_1x(void (*f)(int),int a)//一个int类型参数的函数测量
{
    uint32_t cy_start  = DWT->CYCCNT;   // 记录总周期起点
    uint32_t exc_start = DWT->EXCCNT;   // 记录中断周期起点

    /* ↓↓↓ 把要测试的函数放在这里，可包含会触发中断的代码 ↓↓↓ */
    f(a);

    uint32_t cy_end  = DWT->CYCCNT;     // 记录总周期终点
    uint32_t exc_end = DWT->EXCCNT;     // 记录中断周期终点

    /* 3. 计算结果（无符号差值天然抗回卷） */
    uint32_t total_cycles = cy_end  - cy_start;   // 函数 + 中断总耗时
    uint32_t int_cycles   = exc_end - exc_start;  // 仅中断服务程序耗时
    uint32_t func_pure    = total_cycles - int_cycles; // 函数本体净耗时

    /* 4. 输出或保存数据（可按需要改为 UART、SWO、存储器） */
    printf("total = %lu, int = %lu, func = %lu cycles\r\n",
           total_cycles, int_cycles, func_pure);
}





