#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "seg.h"

//-------------------
// RCC configuration
//-------------------

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

    // (8) Set APB frequency to 48 MHz
    MODIFY_REG( REG_RCC_CFGR, VAL_RCC_CFGR_PPRE_mask, VAL_RCC_CFGR_PPRE_notdiv);
}

void to_get_more_accuracy_pay_2202_2013_2410_3805_1ms()
{
    for (uint32_t i = 0; i < ONE_MILLISECOND/3U; ++i)
    {
        // Insert NOP for power consumption:
        __asm__ volatile("nop");
    }
}

//--------------------
// GPIO configuration
//--------------------

void board_gpio_init()
{
    // (1) Configure PA1-PA12 as output:
    *REG_RCC_AHBENR |= VAL_RCC_AHBENR_IOPAEN;
    *REG_RCC_AHBENR |= VAL_RCC_AHBENR_IOPCEN;

    // Configure mode register:
    for (uint32_t i = 1; i < 13; i++)
        MODIFY_REG( GPIOx_MODER( GPIOA_BASE), GPIOx_MODE_bits( i, GPIOx_MODE_mask), GPIOx_MODE_bits( i, GPIOx_MODE_gpout));

    // Configure type register:
    *GPIOx_TYPER( GPIOA_BASE) = 0U;

    // (2) Configure PA0 as button:
    // MODIFY_REG( GPIOx_MODER( GPIOA_BASE), GPIOx_MODE_bits( 0, GPIOx_MODE_mask), GPIOx_MODE_bits( 0, GPIOx_MODE_input));

    // Configure PA0 as pull-down pin:
    // MODIFY_REG( GPIOx_PUPDR( GPIOA_BASE), GPIOx_PUPD_bits( 0, GPIOx_PUPD_mask), GPIOx_PUPD_bits( 0, GPIOx_PUPD_pulldown));
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

//------
// Main
//------

#define GPIO_GREEN (9U)
#define GPIO_BLUE  (8U)

typedef struct
{
    uintptr_t reg;
    uint32_t number;
} GpioPort;

void gpioPortInit( GpioPort* this, uint32_t mode, uint32_t type, uint32_t pupd)
{
    MODIFY_REG( GPIOx_MODER( this->reg), GPIOx_MODE_bits( this->number, GPIOx_MODE_mask), GPIOx_MODE_bits( this->number, mode));
    MODIFY_REG( GPIOx_TYPER( this->reg), GPIOx_TYPE_bits( this->number, GPIOx_TYPE_mask), GPIOx_TYPE_bits( this->number, type));
    MODIFY_REG( GPIOx_PUPDR( this->reg), GPIOx_PUPD_bits( this->number, GPIOx_PUPD_mask), GPIOx_PUPD_bits( this->number, pupd));
}

bool gpioPortPoll( GpioPort* this)
{
    return *GPIOx_IDR( this->reg) & (1U << this->number);
}

void gpioPortEmit( GpioPort* this, bool val)
{
    MODIFY_REG( GPIOx_ODR( this->reg), GPIOx_OD_bits( this->number, GPIOx_OD_mask), GPIOx_OD_bits( this->number, val));
}

int32_t REQUIRED_SATURATION = 5;

typedef struct
{
    int32_t saturation;
    bool state;
    GpioPort port;
} Button;

bool buttonPoll( Button* this)
{
    bool active = gpioPortPoll( &this->port);

    if (active)
    {
        if (this->saturation == REQUIRED_SATURATION)
            return true;

        this->saturation++;

        return false;
    }
    else
    {
        if (this->saturation == -REQUIRED_SATURATION)
            return false;

        this->saturation--;

        return true;
    }

    return false;
}

void blinkingLed( GpioPort* led, uint32_t tick, uint32_t tickrate, bool is_blinking)
{
    if (!is_blinking)
    {
        gpioPortEmit( led, false);
        return;
    }

    if (tick % tickrate < tickrate / 2)
        gpioPortEmit( led, true);
    else
        gpioPortEmit( led, false);
}

const uint32_t PAUSE_TICKS = 3000;
const uint32_t TICKRATE_WIN = 150;
const uint32_t TICKRATE_LOSS = 1000;

uint32_t segNumcat( uint8_t high, uint8_t low)
{
    return high * 100 + low;
}

typedef struct
{
    uint8_t score;
    bool curr;
    bool prev;
    Button button;
    GpioPort led;
} Player;

struct
{
    Player* first;
    Player* second;

    Button* reset_game;

    SegDisplay* seg;

    uint32_t tick;
    uint32_t pause_ticks;
    bool first_won;
} gGame;

void systickHandler()
{
    gGame.first->curr = buttonPoll( &gGame.first->button);
    gGame.second->curr = buttonPoll( &gGame.second->button);

    if (buttonPoll( gGame.reset_game))
    {
        gGame.pause_ticks = PAUSE_TICKS;

        gGame.first->score = gGame.second->score = 0;
    }
    if (gGame.pause_ticks > 0)
    {
        blinkingLed( &gGame.first->led, gGame.tick, gGame.first_won ? TICKRATE_WIN : TICKRATE_LOSS, true);
        blinkingLed( &gGame.second->led, gGame.tick, gGame.first_won ? TICKRATE_LOSS : TICKRATE_WIN, true);

        gGame.pause_ticks--;
        if (!gGame.pause_ticks)
        {
            gpioPortEmit( &gGame.first->led, false);
            gpioPortEmit( &gGame.second->led, false);
        } 
    }
    else
    {
        if (gGame.first->curr && gGame.second->curr)
        {
            if (gGame.first->prev && !gGame.second->prev)
            {
                gGame.first_won = false;
                gGame.second->score++;
                gGame.pause_ticks = PAUSE_TICKS;
            }
            else if (!gGame.first->prev && gGame.second->prev)
            {
                gGame.first_won = true;
                gGame.first->score++;
                gGame.pause_ticks = PAUSE_TICKS;
            }
        } 
    }

    gGame.first->prev = gGame.first->curr;
    gGame.second->prev = gGame.second->curr;

    segSetNumber( gGame.seg, segNumcat( gGame.first->score, gGame.second->score));
    segUpdate( gGame.seg, gGame.tick);
    segShow( gGame.seg);

    gGame.tick++;
    // to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
}

int main()
{
    board_clocking_init();

    board_gpio_init();

    SegDisplay seg = {
        .number = 0
    };
    gGame.seg = &seg;

    Player first = {
        .score = 0,
        .curr = 0,
        .prev = 0,
        .button = {
            .port = {
                .reg = GPIOC_BASE,
                .number = 0
            }
        },
        .led = {
            .reg = GPIOC_BASE,
            .number = 2
        }
    };
    gGame.first = &first;
    gpioPortInit( &first.button.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);
    gpioPortInit( &first.led, GPIOx_MODE_gpout, GPIOx_TYPE_pushpull, GPIOx_PUPD_none);

    Player second = {
        .score = 0,
        .curr = 0,
        .prev = 0,
        .button = {
            .port = {
                .reg = GPIOC_BASE,
                .number = 1
            }
        },
        .led = {
            .reg = GPIOC_BASE,
            .number = 3
        }
    };
    gGame.second = &second;
    gpioPortInit( &second.button.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);
    gpioPortInit( &second.led, GPIOx_MODE_gpout, GPIOx_TYPE_pushpull, GPIOx_PUPD_none);

    Button reset_game = {
        .port = {
            .reg = GPIOA_BASE,
            .number = 0
        },
        .saturation = 0,
        .state = 0
    };
    gGame.reset_game = &reset_game;
    gpioPortInit( &reset_game.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);

    gGame.tick = 0;
    gGame.pause_ticks = 0;
    gGame.first_won = false;

    systick_init( 1000U);

    while (true)
        ;

    return 0;
}
