#ifndef TOUCH_HANDLER_H
#define TOUCH_HANDLER_H

#include <stdint.h> // For uint64_t
#include "esp_timer.h" // For esp_timer_get_time

// Threshold for touch detection is defined in main.h
// #define TOUCH_THRESHOLD 40

// Function to initialize touch pads
// void touch_init(); // Removed - Logic in io_helpers.cpp

// Function to read touch pads
// void read_touch_pads(); // Removed - Logic in io_helpers.cpp

// Define number of touch pads if not defined elsewhere
#ifndef NUM_TOUCH_PADS
#define NUM_TOUCH_PADS 4
#endif

// Extern declaration for the global array tracking press times
// Defined in state_machine.cpp
extern uint64_t pad_press_time[NUM_TOUCH_PADS];

// Function to get the time a pad has been held in milliseconds
uint64_t get_time_held_ms(int pad_index, uint64_t current_time_us);

// Function to get the time a pad has been held in microseconds (alternative)
uint64_t get_time_held_us(int pad_index, uint64_t current_time_us);


// Declare global variables needed by other files (defined in touch_handler.cpp or io_helpers.cpp)
// Note: It's generally better practice to pass these as parameters or encapsulate them,
// but sticking to the existing structure for now.
extern bool current_touch_state[4];
extern bool last_touch_state[4];
extern bool touch_held_state[4]; // Added for consistency if used elsewhere

#endif // TOUCH_HANDLER_H