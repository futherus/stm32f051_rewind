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

    // (8) Set APB frequency to 24 MHz
    *REG_RCC_CFGR |= VAL_RCC_CFGR_PPRE_2;
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
    MODIFY_REG( GPIOx_MODER( GPIOA_BASE), GPIOx_MODE_bits( 0, GPIOx_MODE_mask), GPIOx_MODE_bits( 0, GPIOx_MODE_input));

    // Configure PA0 as pull-down pin:
    MODIFY_REG( GPIOx_PUPDR( GPIOA_BASE), GPIOx_PUPD_bits( 0, GPIOx_PUPD_mask), GPIOx_PUPD_bits( 0, GPIOx_PUPD_pulldown));
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

const uint32_t PAUSE_TICKS = 600;
const uint32_t TICKRATE_WIN = 30;
const uint32_t TICKRATE_LOSS = 200;

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

void ledOnButton()
{
    SegDisplay seg = {
        .number = 0
    };

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
    gpioPortInit( &second.button.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);
    gpioPortInit( &second.led, GPIOx_MODE_gpout, GPIOx_TYPE_pushpull, GPIOx_PUPD_none);

    uint32_t tick = 0;
    uint32_t pause_ticks = 0;
    bool first_won = false;

    while (1)
    {
        first.curr = buttonPoll( &first.button);
        second.curr = buttonPoll( &second.button);

        if (pause_ticks > 0)
        {
            blinkingLed( &first.led, tick, first_won ? TICKRATE_WIN : TICKRATE_LOSS, true);
            blinkingLed( &second.led, tick, first_won ? TICKRATE_LOSS : TICKRATE_WIN, true);

            pause_ticks--;
            if (!pause_ticks)
            {
                gpioPortEmit( &first.led, false);
                gpioPortEmit( &second.led, false);
            } 
        }
        else
        {
            if (first.curr && second.curr)
            {
                if (first.prev && !second.prev)
                {
                    first_won = false;
                    second.score++;
                    pause_ticks = PAUSE_TICKS;
                }
                else if (!first.prev && second.prev)
                {
                    first_won = true;
                    first.score++;
                    pause_ticks = PAUSE_TICKS;
                }
            } 
        }

        first.prev = first.curr;
        second.prev = second.curr;

        segSetNumber( &seg, segNumcat( first.score, second.score));
        segUpdate( &seg, tick);
        segShow( &seg);

        tick++;
        to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
    }
}


int main()
{
    board_clocking_init();

    board_gpio_init();

    ledOnButton();

    return 0;
}
