// Shared lazy-initialized TIM7 (1 MHz / 1 us) hardware timer dispatcher.

#include "hw_tim7.h"

static bool                    g_tim7_initialized = false;
static bool                    g_dwt_initialized = false;
static volatile HwTim7StepFn_t g_tim7_stepFn = NULL;

void HW_DWT_EnsureInit(void)
{
    if (g_dwt_initialized) {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_initialized = true;
}

uint32_t HW_DWT_Micros(void)
{
    if (!g_dwt_initialized) {
        HW_DWT_EnsureInit();
    }
    return DWT->CYCCNT / 168U;
}

void HW_DWT_DelayUs(uint32_t us)
{
    if (!g_dwt_initialized) {
        HW_DWT_EnsureInit();
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 168U;
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

void HW_TIM7_EnsureInit(void)
{
    if (g_tim7_initialized) {
        return;
    }
    __HAL_RCC_TIM7_CLK_ENABLE();

    TIM7->CR1  = TIM_CR1_OPM | TIM_CR1_URS;
    TIM7->PSC  = 83U; // 84 MHz APB1 timer clock / 84 = 1 MHz (1 us)
    TIM7->ARR  = 10U;
    TIM7->EGR  = TIM_EGR_UG;
    TIM7->SR   = 0U;
    TIM7->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM7_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    g_tim7_initialized = true;
}

void HW_TIM7_DeInit(void)
{
    if (!g_tim7_initialized) {
        return;
    }
    TIM7->CR1  &= ~TIM_CR1_CEN;
    TIM7->DIER &= ~TIM_DIER_UIE;
    TIM7->SR    = 0U;
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
    __HAL_RCC_TIM7_CLK_DISABLE();
    g_tim7_stepFn      = NULL;
    g_tim7_initialized = false;
}

void HW_TIM7_ScheduleUs(uint32_t delayUs, HwTim7StepFn_t fn)
{
    if (!g_tim7_initialized) {
        HW_TIM7_EnsureInit();
    }
    if (delayUs < 2U) {
        delayUs = 2U;
    } else if (delayUs > 65535U) {
        delayUs = 65535U;
    }
    TIM7->CR1 &= ~TIM_CR1_CEN;
    g_tim7_stepFn = fn;
    TIM7->CNT  = 0U;
    TIM7->ARR  = (uint16_t)(delayUs - 1U);
    TIM7->EGR  = TIM_EGR_UG;
    TIM7->SR   = 0U;
    TIM7->CR1 |= TIM_CR1_CEN;
}

void HW_TIM7_Cancel(void)
{
    if (g_tim7_initialized) {
        TIM7->CR1 &= ~TIM_CR1_CEN;
        TIM7->SR   = 0U;
    }
    g_tim7_stepFn = NULL;
}

bool HW_TIM7_IsBusy(void)
{
    return g_tim7_stepFn != NULL;
}

void HW_TIM7_OnIrq(void)
{
    if ((TIM7->SR & TIM_SR_UIF) == 0U) {
        return;
    }
    TIM7->SR = 0U;
    HwTim7StepFn_t fn = g_tim7_stepFn;
    if (fn == NULL) {
        return;
    }
    uint32_t nextUs = fn();
    if (nextUs > 0U) {
        if (nextUs < 2U) {
            nextUs = 2U;
        } else if (nextUs > 65535U) {
            nextUs = 65535U;
        }
        TIM7->CNT  = 0U;
        TIM7->ARR  = (uint16_t)(nextUs - 1U);
        TIM7->EGR  = TIM_EGR_UG;
        TIM7->SR   = 0U;
        TIM7->CR1 |= TIM_CR1_CEN;
    } else {
        g_tim7_stepFn = NULL;
    }
}
