#ifndef SEG_H
#define SEG_H

//-------------------
// 7-segment display
//-------------------

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"

// Display state
typedef struct
{
    uint32_t display;

    uint16_t number;
} SegDisplay;

// Set number
void segSetNumber( SegDisplay* this, uint16_t number);

// Update display value
void segUpdate( SegDisplay* this, uint32_t tick);

// Write changes to microcontroller
void segShow( SegDisplay* this);

#endif // SEG_H
