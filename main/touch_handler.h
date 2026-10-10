#ifndef TOUCH_HANDLER_H
#define TOUCH_HANDLER_H

#include <stdint.h>

#ifndef NUM_TOUCH_PADS
#define NUM_TOUCH_PADS 4
#endif

extern uint64_t pad_press_time[NUM_TOUCH_PADS];

#endif // TOUCH_HANDLER_H
