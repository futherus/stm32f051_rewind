#ifndef HAL_H
#define HAL_H

//---------------
// Macros
//---------------

#define MODIFY_REG( REG, MODIFYMASK, VALUE) *(REG) = ( ((MODIFYMASK) & (VALUE)) | (*(REG) & ~(MODIFYMASK)) )

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
    #define VAL_RCC_CFGR_PLLMUL( FACTOR) (((FACTOR) - 2U) << 18U)    // Valid: FACTOR = [2:16]
    #define VAL_RCC_CFGR_HPRE_NOTDIVIDED (0b0000U << 4U)
    #define VAL_RCC_CFGR_SW_PLL (0b10U)
    #define VAL_RCC_CFGR_SWS_MASK (0b11U << 2U)
    #define VAL_RCC_CFGR_SWS_PLL (0b10U << 2U)
    #define VAL_RCC_CFGR_PPRE_2 (0b100 << 8U)

#define REG_RCC_AHBENR (volatile uint32_t*)(uintptr_t)0x40021014U // AHB1 Peripheral Clock Enable Register
    #define VAL_RCC_AHBENR_IOPAEN (0b1U << 17U)
    #define VAL_RCC_AHBENR_IOPCEN (0b1U << 19U)

#define REG_RCC_CFGR2  (volatile uint32_t*)(uintptr_t)0x4002102CU // Clock configuration register 2
    #define VAL_RCC_CFGR2_PREDIV( FACTOR) ((FACTOR) - 1U)            // Valid: FACTOR = [1:16]

//----------------
// GPIO Registers
//----------------
#define GPIOA_BASE (uintptr_t)(0x48000000U) // GPIO port mode register
#define GPIOC_BASE (uintptr_t)(0x48000800U) // GPIO port mode register

#define GPIOx_MODER( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x0U))
    #define GPIOx_MODE_bits( N, MODE) ((MODE) << (2U * (N)))        // Valid: N = [0:15]
    #define GPIOx_MODE_mask   (0b11U)
    #define GPIOx_MODE_input  (0b00U)
    #define GPIOx_MODE_gpout  (0b01U)
    #define GPIOx_MODE_alt    (0b10U)
    #define GPIOx_MODE_analog (0b11U)

#define GPIOx_TYPER( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x4U))
    #define GPIOx_TYPE_bits( N, MODE)  ((MODE) << (N))
    #define GPIOx_TYPE_mask      (0b1U)
    #define GPIOx_TYPE_pushpull  (0b0U)
    #define GPIOx_TYPE_opendrain (0b1U)

#define GPIOx_SPEEDR( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x8U))
    #define GPIOx_SPEED_bits( N, MODE) ((MODE) << (2U * (N)))
    #define GPIOx_SPEED_mask   (0b11U)
    #define GPIOx_SPEED_low    (0b00U)
    #define GPIOx_SPEED_medium (0b10U)
    #define GPIOx_SPEED_high   (0b11U)

#define GPIOx_PUPDR( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0xCU))
    #define GPIOx_PUPD_bits( N, MODE) ((MODE) << (2U * (N)))
    #define GPIOx_PUPD_mask     (0b11U)
    #define GPIOx_PUPD_none     (0b00U)
    #define GPIOx_PUPD_pullup   (0b01U)
    #define GPIOx_PUPD_pulldown (0b10U)

#define GPIOx_IDR( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x10U))

#define GPIOx_ODR( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x14U))
    #define GPIOx_OD_bits( N, MODE) ((MODE) << (N))
    #define GPIOx_OD_mask (0b1U)

#endif // HAL_H
