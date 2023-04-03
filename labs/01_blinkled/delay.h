#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

#define DELAY_CYCLES_PER_TICK 8

/* Converts interval in microseconds to ticks
 * with respect to frequency
 *
 * WARNING: USEC < 715 000 000
 */
#define delay_msec2ticks( USEC, FREQ) ((USEC) * ((FREQ) / 1000000U / DELAY_CYCLES_PER_TICK))

void delay( uint32_t ticks);

#endif // DELAY_H
