#include <stdint.h>

//---------------
// Macros
//---------------

#define MODIFY_REG(REG, MODIFYMASK, VALUE) *(REG) = ( ((MODIFYMASK) & (VALUE)) | (*(REG) & ~(MODIFYMASK)) )

//---------------
// RCC Registers
//---------------

#define REG_RCC_CR     (volatile uint32_t*)(uintptr_t)0x40021000U // Clock Control Register
    #define VAL_RCC_CR_HSEON  (0x1 << 16U)
    #define VAL_RCC_CR_HSERDY (0x1 << 17U)
    #define VAL_RCC_CR_PLLON  (0x1 << 24U)
    #define VAL_RCC_CR_PLLRDY (0x1 << 25U)

#define REG_RCC_CFGR   (volatile uint32_t*)(uintptr_t)0x40021004U // PLL Configuration Register
    #define VAL_RCC_CFGR_PLLSRC_HSE (0b10 << 15U)
    #define VAL_RCC_CFGR_PLLMUL(FACTOR) ((FACTOR - 2U) << 18U)    // Valid: FACTOR = [2:16]
    #define VAL_RCC_CFGR_HPRE_NOTDIVIDED (0b0000U << 4U)
    #define VAL_RCC_CFGR_SW_PLL (0b10U)
    #define VAL_RCC_CFGR_SWS_MASK (0b11U << 2U)
    #define VAL_RCC_CFGR_SWS_PLL (0b10U << 2U)
    #define VAL_RCC_CFGR_PPRE_2 (0b100 << 8U)

#define REG_RCC_AHBENR (volatile uint32_t*)(uintptr_t)0x40021014U // AHB1 Peripheral Clock Enable Register
    #define VAL_RCC_AHBENR_IOPCEN (0b1U << 19U)
#define REG_RCC_CFGR2  (volatile uint32_t*)(uintptr_t)0x4002102CU // Clock configuration register 2
    #define VAL_RCC_CFGR2_PREDIV(FACTOR) (FACTOR - 1U)            // Valid: FACTOR = [1:16]

//----------------
// GPIO Registers
//----------------

#define GPIOC_MODER (volatile uint32_t*)(uintptr_t)0x48000800U // GPIO port mode register
    #define GPIO_MODERn(N, MODE) ((MODE) << (2U * (N)))        // Valid: N = [0:15]
    #define GPIO_MODER_INPUT  (0b00U)
    #define GPIO_MODER_GPOUT  (0b01U)
    #define GPIO_MODER_ALT    (0b10U)
    #define GPIO_MODER_ANALOG (0b11U)

#define GPIOC_TYPER (volatile uint32_t*)(uintptr_t)0x48000804U // GPIO port output type register
    #define GPIO_TYPERn(N) (0b1U << (N))

#define GPIOC_ODR (volatile uint32_t*)(uintptr_t)0x48000814U   // GPIO output data register
    #define GPIOC_ODRn(N) (0b1U << (N))

//------
// Main
//------

#define CPU_FREQENCY 48000000U // CPU frequency: 48 MHz
#define ONE_MILLISECOND CPU_FREQENCY/1000U

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

    // (8) Set APB frequency to 24 MHz
    *REG_RCC_CFGR |= VAL_RCC_CFGR_PPRE_2;
}

void board_gpio_init()
{
    // (1) Enable GPIOC clocking:
    *REG_RCC_AHBENR |= VAL_RCC_AHBENR_IOPCEN;

    // (2) Configure PC mode:
    *GPIOC_MODER |= GPIO_MODERn(8, GPIO_MODER_GPOUT);
    *GPIOC_MODER |= GPIO_MODERn(9, GPIO_MODER_GPOUT);

    // (3) Configure PC type:
    *GPIOC_TYPER |= GPIO_TYPERn(8);
    *GPIOC_TYPER |= GPIO_TYPERn(9);
    // &=
}

void totally_accurate_quantum_femtosecond_precise_super_delay_3000_100000ms()
{
    for (uint32_t i = 0; i < 100000U * ONE_MILLISECOND; ++i)
    {
        // Insert NOP for power consumption:
        __asm__ volatile("nop");
    }
}

int main()
{
#ifndef INSIDE_QEMU
    board_clocking_init();
#endif

    board_gpio_init();

    while (1)
    {
        *GPIOC_ODR |= GPIOC_ODRn(8U);
        *GPIOC_ODR &= ~GPIOC_ODRn(9U);

        MODIFY_REG(GPIOC_ODR, GPIOC_ODRn(8U), GPIOC_ODRn(8U));
        MODIFY_REG(GPIOC_ODR, GPIOC_ODRn(9U), 0U);

        totally_accurate_quantum_femtosecond_precise_super_delay_3000_100000ms();

        *GPIOC_ODR &= ~GPIOC_ODRn(8U);
        *GPIOC_ODR |= GPIOC_ODRn(9U);

        MODIFY_REG(GPIOC_ODR, GPIOC_ODRn(8U), 0U);
        MODIFY_REG(GPIOC_ODR, GPIOC_ODRn(9U), GPIOC_ODRn(9U));


        totally_accurate_quantum_femtosecond_precise_super_delay_3000_100000ms();
    }
}
