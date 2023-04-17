#include <stdint.h>
#include <stdbool.h>

#include "hal.h"

//-------------------
// RCC configuration
//-------------------

#define CPU_FREQENCY 48000000U // CPU frequency: 48 MHz
#define ONE_MILLISECOND (CPU_FREQENCY / 1000U)

void board_clocking_init()
{
    // (1) Clock HSE and wait for oscillations to setup.
    *REG_RCC_CR = VAL_RCC_CR_HSEON;
    while ((*REG_RCC_CR & VAL_RCC_CR_HSERDY) != VAL_RCC_CR_HSERDY)
        ;

    // (2) Configure PLL:
    // PREDIV output: HSE/2 = 4 MHz
    *REG_RCC_CFGR2 |= VAL_RCC_CFGR2_PREDIV(2);

    // (3) Select PREDIV output as PLL input (4 MHz):
    *REG_RCC_CFGR |= VAL_RCC_CFGR_PLLSRC_HSE;

    // (4) Set PLLMUL to 12:
    // SYSCLK frequency = 48 MHz
    *REG_RCC_CFGR |= VAL_RCC_CFGR_PLLMUL(12);

    // (5) Enable PLL:
    *REG_RCC_CR |= VAL_RCC_CR_PLLON;
    while ((*REG_RCC_CR & VAL_RCC_CR_PLLRDY) != VAL_RCC_CR_PLLRDY)
        ;

    // (6) Configure AHB frequency to 48 MHz:
    *REG_RCC_CFGR |= VAL_RCC_CFGR_HPRE_NOTDIVIDED;

    // (7) Select PLL as SYSCLK source:
    *REG_RCC_CFGR |= VAL_RCC_CFGR_SW_PLL;
    while ((*REG_RCC_CFGR & VAL_RCC_CFGR_SWS_MASK) != VAL_RCC_CFGR_SWS_PLL)
        ;

    // (8) Set APB frequency to 48 MHz
    MODIFY_REG( REG_RCC_CFGR, VAL_RCC_CFGR_PPRE_mask, VAL_RCC_CFGR_PPRE_notdiv);
}

#define ONE_SEC_DELAY_TIME 3692308U // 48000000 / 13
void more_precise_delay_forbidden_by_quantum_mechanics_1000ms()
{
    for (uint32_t i = 0; i < ONE_SEC_DELAY_TIME; ++i);
}

//--------------------
// GPIO configuration
//--------------------

void board_gpio_init()
{
    // (1) Configure PC8 and PC9:
    *REG_RCC_AHBENR |= VAL_RCC_AHBENR_IOPCEN;

    // (2) Configure LED modes:
    MODIFY_REG( GPIOx_MODER( GPIOC_BASE), GPIOx_MODE_bits( 8, GPIOx_MODE_mask), GPIOx_MODE_bits( 8, GPIOx_MODE_gpout));
    MODIFY_REG( GPIOx_MODER( GPIOC_BASE), GPIOx_MODE_bits( 9, GPIOx_MODE_mask), GPIOx_MODE_bits( 9, GPIOx_MODE_gpout));

    // (3) Configure LED types:
    MODIFY_REG( GPIOx_TYPER( GPIOC_BASE), GPIOx_TYPE_bits( 8, GPIOx_TYPE_mask), GPIOx_MODE_bits( 8, GPIOx_TYPE_pushpull));
    MODIFY_REG( GPIOx_TYPER( GPIOC_BASE), GPIOx_TYPE_bits( 9, GPIOx_TYPE_mask), GPIOx_MODE_bits( 9, GPIOx_TYPE_pushpull));
}

//-----------------------
// SysTick configuration
//-----------------------

void systick_init( uint32_t period_us)
{
    // (0) Read STM32F051 SysTick configuration:
    // Assumptions:
    // - There is a reference clock and it can be chosen as clock source.
    // - The SYST_CALIB SKEW bit is 1.
    uint32_t reload_value = period_us * (CPU_FREQENCY / 1000000U) / 8;

    // (1) Program the reload value:
    // *SYSTICK_RVR = (reload_value - 1U) & 0x00FFFFFFU;
    MODIFY_REG( SYST_RVR, SYST_RVR_RELOAD, (reload_value - 1U));

    // (2) Clear the current value:
    // *SYSTICK_CVR = 0U;
    MODIFY_REG( SYST_CVR, SYST_CVR_CURRENT, 0U);

    // (3) Program the CSR:
    // Watch out for the clock source!
    // *SYSTICK_CSR = 0x3U;
    MODIFY_REG( SYST_CSR, SYST_CSR_TICKINT | SYST_CSR_ENABLE, SYST_CSR_TICKINT | SYST_CSR_ENABLE); 
}

void systick_handler()
{
    static int handler_ticks = 0U;
    static bool prev_value = 1U;

    handler_ticks += 1U;

    if (handler_ticks == 10000U)
    {
        handler_ticks = 0U;

        // uint32_t reg_gpioc_odr = READ_REG( GPIOx_ODR( GPIOC_BASE), GPIOx_OD_bits( 8, GPIOx_OD_mask));

        MODIFY_REG( GPIOx_ODR( GPIOC_BASE), GPIOx_OD_bits( 8, GPIOx_OD_mask), GPIOx_OD_bits( 8, prev_value));
        prev_value = !prev_value;

        // uint32_t reg_gpio_odr = *GPIOC_ODR;
        // *GPIOC_ODR = (reg_gpio_odr & ~0x0100U) | (~reg_gpio_odr & 0x0100U);
    }
}

//------
// Main
//------

int main(void)
{
    board_clocking_init();

    board_gpio_init();

    systick_init(100U);

    while (1)
    {
        more_precise_delay_forbidden_by_quantum_mechanics_1000ms();

        *GPIOx_BSRR( GPIOC_BASE) |= GPIOx_BSRR_set_bit( 9);

        more_precise_delay_forbidden_by_quantum_mechanics_1000ms();

        *GPIOx_BSRR( GPIOC_BASE) |= GPIOx_BSRR_reset_bit( 9);
    }
}
