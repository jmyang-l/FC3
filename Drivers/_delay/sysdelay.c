#include "sysdelay.h"

/* ========= 1. 修改 SysTick 到 1us ========= */
void SysTick_Init_1us(void)//放到SystemClock_Config();后面，保证已经把 SystemCoreClock 更新到 480 MHz
{
    /* 关 SysTick 先 */
    SysTick->CTRL = 0;

    /* 重装值 = CPU时钟 / 1MHz -1 */
    SysTick->LOAD = (SystemCoreClock / 1000000U) - 1;
    SysTick->VAL  = 0;

    /* 使能中断 + 使能计数器 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

/* ========= 2. 延时函数 ========= */
void delay_us(uint32_t us)
{
    uint32_t start = SysTick->VAL;          // 当前剩余值
    uint32_t ticks = us * (SystemCoreClock / 1000000U); // 需要的 ticks
                                                        //1秒 = 1 000 000 微秒 所以 1 微秒内的时钟周期数=(SystemCoreClock / 1000000U)
    while ((start - SysTick->VAL) < ticks) { /* 空转 */ }
}

///* ========= 3. 中断服务函数：每 1us 进来一次，但 1000 次才++uwTick ========= */在 stm32h7xx_it.c 里面修改
//void SysTick_Handler(void)
//{
//    static uint32_t cnt = 0;
//    if (++cnt >= 1000) {        // 1000 * 1us = 1ms
//        cnt = 0;
//        HAL_IncTick();          // 保持 HAL_Delay / uwTick 按 1ms 步进, 让HAL_Delay()这个函数正常使用
//    }
//}

//“非堵塞”的1s延时，使用uwtick实现，要保证uwtick依旧是1ms增加一次
void delay_1s(void)
{
    uint32_t tick = HAL_GetTick();
    while ((HAL_GetTick() - tick) < 1000) {
        /* 这里可以干别的事，只要保证主循环足够快 */
    }
}


/* ========= 补充. 修改 SysTick 到 100us ========= */
void SysTick_Init_100us(void)//放到SystemClock_Config();后面，保证已经把 SystemCoreClock 更新到 480 MHz
{
    /* 关 SysTick 先 */
    SysTick->CTRL = 0;

    /* 重装值 = CPU时钟 / 1MHz -1 */
    SysTick->LOAD = (SystemCoreClock / 10000U) - 1;
    SysTick->VAL  = 0;

    /* 使能中断 + 使能计数器 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}