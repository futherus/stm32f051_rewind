#include <stdint.h>
#include <stdbool.h>

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
    #define VAL_RCC_CFGR_PLLMUL(FACTOR) (((FACTOR) - 2U) << 18U)    // Valid: FACTOR = [2:16]
    #define VAL_RCC_CFGR_HPRE_NOTDIVIDED (0b0000U << 4U)
    #define VAL_RCC_CFGR_SW_PLL (0b10U)
    #define VAL_RCC_CFGR_SWS_MASK (0b11U << 2U)
    #define VAL_RCC_CFGR_SWS_PLL (0b10U << 2U)
    #define VAL_RCC_CFGR_PPRE_2 (0b100 << 8U)

#define REG_RCC_AHBENR (volatile uint32_t*)(uintptr_t)0x40021014U // AHB1 Peripheral Clock Enable Register
    #define VAL_RCC_AHBENR_IOPAEN (0b1U << 17U)
    #define VAL_RCC_AHBENR_IOPCEN (0b1U << 19U)

#define REG_RCC_CFGR2  (volatile uint32_t*)(uintptr_t)0x4002102CU // Clock configuration register 2
    #define VAL_RCC_CFGR2_PREDIV(FACTOR) ((FACTOR) - 1U)            // Valid: FACTOR = [1:16]

//----------------
// GPIO Registers
//----------------
#define GPIOA_BASE (uintptr_t)(0x48000000U) // GPIO port mode register
#define GPIOC_BASE (uintptr_t)(0x48000800U) // GPIO port mode register

#define GPIOx_MODER(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x0U))
    #define GPIOx_MODE_bits(N, MODE) ((MODE) << (2U * (N)))        // Valid: N = [0:15]
    #define GPIOx_MODE_mask     (0b11U)
    #define GPIOx_MODE_input    (0b00U)
    #define GPIOx_MODE_gpout    (0b01U)
    #define GPIOx_MODE_alt      (0b10U)
    #define GPIOx_MODE_analog   (0b11U)

#define GPIOx_TYPER(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x4U))
    #define GPIOx_TYPE_bits(N, MODE)  ((MODE) << (N))
    #define GPIOx_TYPE_mask      (0b1U)
    #define GPIOx_TYPE_pushpull  (0b0U)
    #define GPIOx_TYPE_opendrain (0b1U)

#define GPIOx_SPEEDR(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x8U))
    #define GPIOx_SPEED_bits(N, MODE) ((MODE) << (2U * (N)))
    #define GPIOx_SPEED_mask     (0b11U)
    #define GPIOx_SPEED_low      (0b00U)
    #define GPIOx_SPEED_medium   (0b10U)
    #define GPIOx_SPEED_high     (0b11U)

#define GPIOx_PUPDR(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0xCU))
    #define GPIOx_PUPD_bits(N, MODE) ((MODE) << (2U * (N)))
    #define GPIOx_PUPD_mask     (0b11U)
    #define GPIOx_PUPD_none     (0b00U)
    #define GPIOx_PUPD_pullup   (0b01U)
    #define GPIOx_PUPD_pulldown (0b10U)

#define GPIOx_IDR(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x10U))

#define GPIOx_ODR(BASE) ((volatile uint32_t*)((uintptr_t)(BASE) + 0x14U))
    #define GPIOx_OD_bits(N, MODE) ((MODE) << (N))
    #define GPIOx_OD_mask      (0b1U)
 
#if 0
#define GPIOC_AFRL  (volatile uint32_t*)(uintptr_t)0x48000820U // GPIO port alternate function selection for lower (0 - 7) pins

#define GPIOx_AFRL_offset  (0x20U)
#define AFSEL_size_in_bits (0x4U)
#define AFSEL_AF0          (0b0000)
#define AFSEL_AF1          (0b0001) 
#define AFSEL_AF2          (0b0010)
#define AFSEL_AF3          (0b0011)
#define AFSEL_AF4          (0b0100)
#define AFSEL_AF5          (0b0101) 
#define AFSEL_AF6          (0b0110)
#define AFSEL_AF7          (0b0111)

#define GPIOA_BASE ((volatile uint32_t*)(uintptr_t)0x48000000U) // GPIO port mode register
#define GPIOC_BASE ((volatile uint32_t*)(uintptr_t)0x48000800U) // GPIO port mode register

#define PIN0 (0U)
#define PIN1 (1U)
#define PIN2 (2U)
#define PIN3 (3U)
#define PIN4 (4U)
#define PIN5 (5U)
#define PIN6 (6U)
#define PIN7 (7U)
#define PIN8 (8U)
#define PIN9 (9U)
#define PIN10 (10U)
#define PIN11 (10U)
#define PIN12 (10U)
#define PIN14 (10U)
#define PIN15 (10U)
#endif

//-------------------
// 7-segment display
//-------------------

// SUPER-DUPER TRUSTWORTHY Pin Mapping:
#define A  (1U << 11U)
#define B  (1U << 7U)
#define C  (1U << 4U)
#define D  (1U << 2U)
#define E  (1U << 1U)
#define F  (1U << 10U)
#define G  (1U << 5U)
#define DP (1U << 3U)

#define POS0 (1U << 12U)
#define POS1 (1U << 9U)
#define POS2 (1U << 8U)
#define POS3 (1U << 6U)

static const uint32_t PINS_USED = A|B|C|D|E|F|G|DP|POS0|POS1|POS2|POS3;

// TOTALLY CORRECT digit composition:
#define _ 0
static const uint32_t DIGITS[10] =
{
    A|B|C|D|E|F|_, // 0
    _|B|C|_|_|_|_, // 1
    A|B|_|D|E|_|G, // 2
    A|B|C|D|_|_|G, // 3
    _|B|C|_|_|F|G, // 4
    A|_|C|D|_|F|G, // 5
    A|_|C|D|E|F|G, // 6
    A|B|C|_|_|_|_, // 7
    A|B|C|D|E|F|G, // 8
    A|B|C|D|_|F|G, // 9
};
#undef _

#define ____ 0
static const uint32_t POSITIONS[4] =
{
    POS0|POS1|POS2|____, // 0
    POS0|POS1|____|POS3, // 1
    POS0|____|POS2|POS3, // 2
    ____|POS1|POS2|POS3, // 3
};
#undef ____

// Display state:
typedef struct
{
    uint32_t display;

    uint16_t number;
} SegDisplay;

// Set number to display.
void segSetNumber( SegDisplay* this, uint16_t number)
{
    this->number = number;
}

// Update display value.
void segUpdate( SegDisplay* this, uint32_t tick)
{
    uint32_t divisors[4] = {1, 10, 100, 1000};

    unsigned quarter = tick % 4;
    unsigned divisor = divisors[quarter];

    this->display = DIGITS[(this->number / divisor) % 10] | POSITIONS[quarter];
}

// Write changes to microcontroller.
void segShow( SegDisplay* this)
{
    uint32_t surrounding_state = ~PINS_USED & *GPIOx_ODR( GPIOA_BASE);
     //   seg7->display = F | POS3;
    uint32_t to_write = PINS_USED & this->display;

    *GPIOx_ODR( GPIOA_BASE) = surrounding_state | to_write;
}

#if 0
void set_GPIOx_AFRL( volatile uint32_t *GPIOx_addr, 
                     uint32_t afsel, 
                     uint32_t pin )
{
    *(GPIOx_addr + GPIOx_AFRL_offset) = (afsel << (pin * AFSEL_size_in_bits));
}
#endif

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
// 
//     // (2) Configure PA0 as button:
//     MODIFY_REG( GPIOx_MODER( GPIOA_BASE), GPIOx_MODE_bits( 0, GPIOx_MODE_mask), GPIOx_MODE_bits( 0, GPIOx_MODE_input));
// 
//     // Configure PA0 as pull-down pin:
//     MODIFY_REG( GPIOx_PUPDR( GPIOA_BASE), GPIOx_PUPD_bits( 0, GPIOx_PUPD_mask), GPIOx_PUPD_bits( 0, GPIOx_PUPD_pulldown));

//    set_GPIOx_AFRL( GPIOC_BASE, AFSEL_AF2, PIN0 );
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

    this->saturation = 0;

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

const uint32_t PAUSE_TICKS = 1000;
const uint32_t TICKRATE_WIN = 30;
const uint32_t TICKRATE_LOSS = 200;

// uint32_t numcat(uint16_t high, uint16_t low)
// {
//     return high << 16U | low;
// }

void ledOnButton()
{
    Button button_first = {
        .port = {
            .reg = GPIOC_BASE,
            .number = 0
        }
    };

    gpioPortInit( &button_first.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);

    Button button_second = {
        .port = {
            .reg = GPIOC_BASE,
            .number = 1
        }
    };
    gpioPortInit( &button_second.port, GPIOx_MODE_input, GPIOx_TYPE_pushpull, GPIOx_PUPD_pulldown);

    GpioPort led_first = {
        .reg = GPIOC_BASE,
        .number = 2
    };
    gpioPortInit( &led_first, GPIOx_MODE_gpout, GPIOx_TYPE_pushpull, GPIOx_PUPD_none);

    GpioPort led_second = {
        .reg = GPIOC_BASE,
        .number = 3
    };
    gpioPortInit( &led_second, GPIOx_MODE_gpout, GPIOx_TYPE_pushpull, GPIOx_PUPD_none);
 
    bool prev_press_first = false;
    bool prev_press_second = false;
    while (1)
    {
        bool curr_press_first = buttonPoll( &button_first);
        bool curr_press_second = buttonPoll( &button_second);

        if (prev_press_first && prev_press_second)
        {
            prev_press_first = curr_press_first;
            prev_press_second = curr_press_second;
            continue;
        }

        if (curr_press_first && curr_press_second)
        {
            if (prev_press_first)
            {
                for (uint32_t tick = 0; tick < PAUSE_TICKS; tick++)
                {
                    blinkingLed( &led_first, tick, TICKRATE_LOSS, true);
                    blinkingLed( &led_second, tick, TICKRATE_WIN, true);
                    to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
                }

                gpioPortEmit( &led_first, false);
                gpioPortEmit( &led_second, false);
            }
            else if (prev_press_second)
            {
                for (uint32_t tick = 0; tick < PAUSE_TICKS; tick++)
                {
                    blinkingLed( &led_first, tick, TICKRATE_WIN, true);
                    blinkingLed( &led_second, tick, TICKRATE_LOSS, true);
                    to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
                }

                gpioPortEmit( &led_first, false);
                gpioPortEmit( &led_second, false);
            }
        } 

        prev_press_first = curr_press_first;
        prev_press_second = curr_press_second;

        to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
    }
}


int main()
{
    board_clocking_init();

    board_gpio_init();
   
    ledOnButton();

    return 0;


    // Init display rendering:
//     SegDisplay seg =
//     {
//         .number = 0
//     };
// 
//     GpioPort port = {
//         .reg = GPIOA_BASE,
//         .number = 0
//     };
// 
//     Button button = {
//         .port = port
//     };
// 
//     uint32_t tick = 0;
//     bool active = false; 
//     while (1)
//     {
//         
//         bool status = buttonPoll( &button);
// 
//         if (status && !active)
//         {
//             if (seg.number < 9999U)
//                 seg.number++;
// 
//             active = true;
//         }
// 
//         if (!status)
//             active = false;
//        
//         segUpdate( &seg, tick);
//         segShow( &seg);
// 
//         // Adjust ticks every ms:
//         to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
//         tick += 1;
// 
//     }
// 

//     while (1)
//     {
//         // Update button state:
//         bool active = *GPIOA_IDR & (1U << 0U);
// 
//         if (active)
//         {
//             if (saturation < 5U)
//             {
//                 saturation += 1U;
//             }
//             else
//             {
//                 button_was_pressed = 1U;
//             }
//         }
//         else
//         {
//             saturation = 0U;
//         }
// 
//         // Update display state:
//         if (!button_was_pressed && (tick % 10U) == 0U)
//         {
//             if (seg7.number < 9999U)
//             {
//                 seg7.number = seg7.number + 1U;
//             }
//         }
// 
//         // Render display state:
//         SEG7_set_number_quarter(&seg7, tick);
// 
//         SEG7_push_display_state_to_mc(&seg7);
// 
//         // Adjust ticks every ms:
//         to_get_more_accuracy_pay_2202_2013_2410_3805_1ms();
//         tick += 1;
//     }

}
