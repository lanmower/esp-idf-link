#ifndef TOUCH_HANDLER_H
#define TOUCH_HANDLER_H

#include <stdint.h> // For uint64_t, used by pad_press_time
// esp_timer.h is not strictly needed if get_time_held_us/ms are removed, but pad_press_time uses uint64_t
// Let's keep it for now, as pad_press_time might be related to timestamps.

// Define number of touch pads if not defined elsewhere
#ifndef NUM_TOUCH_PADS
#define NUM_TOUCH_PADS 4 // This should ideally come from a central config like main.h
#endif

// Extern declaration for the global array tracking press times
// Defined in state_machine.cpp
extern uint64_t pad_press_time[NUM_TOUCH_PADS]; // This is used by state_machine.cpp

// Declare global variables needed by other files (defined in touch_handler.cpp or io_helpers.cpp)
// Note: It's generally better practice to pass these as parameters or encapsulate them,
// but sticking to the existing structure for now.
// extern bool current_touch_state[4]; // REMOVED
// extern bool last_touch_state[4]; // REMOVED
// extern bool touch_held_state[4]; // REMOVED

#endif // TOUCH_HANDLER_H