#ifndef HAL_H
#define HAL_H

//---------------
// Macros
//---------------

#define MODIFY_REG( REG, MODIFYMASK, VALUE) *(REG) = ( ((MODIFYMASK) & (VALUE)) | (*(REG) & ~(MODIFYMASK)) )

#define READ_REG( REG, READMASK) (*(REG) & (READMASK))

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
    #define VAL_RCC_CFGR_PPRE_mask (0b111U << 8U)
    #define VAL_RCC_CFGR_PPRE_notdiv (0b000U << 8U)
    #define VAL_RCC_CFGR_PPRE_2 (0b100U << 8U)

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

#define GPIOx_BSRR( BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x18U))
    #define GPIOx_BSRR_reset_bit( N) (0b1U << (16U + (N)))
    #define GPIOx_BSRR_set_bit( N)   (0b1U << (N))

//-------------------
// SysTick registers
//-------------------

#define SYST_CSR   (volatile uint32_t*)(uintptr_t)0xE000E010U // SysTick Control and Status Register
    #define SYST_CSR_COUNTFLAG (1U << 16U)
    #define SYST_CSR_CLKSOURCE (1U <<  2U)
        #define SYST_CLR_CLKSOURCE_mask (1U)
        #define SYST_CSR_CLKSOURCE_ref  (0U)
        #define SYST_CSR_CLKSOURCE_proc (1U)

    #define SYST_CSR_TICKINT   (1U <<  1U)
    #define SYST_CSR_ENABLE    (1U <<  0U)

#define SYST_RVR   (volatile uint32_t*)(uintptr_t)0xE000E014U // SysTick Reload Value Register
    #define SYST_RVR_RELOAD (0xFFFFFFU)

#define SYST_CVR   (volatile uint32_t*)(uintptr_t)0xE000E018U // SysTick Current Value Register
    #define SYST_CVR_CURRENT (0xFFFFFFU)

#define SYST_CALIB (volatile uint32_t*)(uintptr_t)0xE000E01CU // SysTick Calibration Value Register
    #define SYST_CALIB_NOREF (1U << 31U)
    #define SYST_CALIB_SKEW  (1U << 30U)
    #define SYST_CALIB_TENMS (0xFFFFFFU)

#endif // HAL_H
