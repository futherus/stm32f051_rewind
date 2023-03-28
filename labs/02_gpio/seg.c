#include "seg.h"

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

// TOTALLY CORRECT digit composition
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

void segSetNumber( SegDisplay* this, uint16_t number)
{
    this->number = number;
}

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
