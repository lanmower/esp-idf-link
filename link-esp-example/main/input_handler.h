#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include <stdint.h> // For uint64_t
#include "main.h" // For MIDI_CC_THRESHOLD if needed
#include "state_machine.h" // Include for InputEvent definition

// Function to initialize input hardware (ADC, Touch)
void initialize_inputs();

// Modified function to read all inputs and populate the InputEvent struct
// Returns true if any input changed significantly, false otherwise.
bool read_inputs(InputEvent& current_event); // Changed return type and parameters


#endif // INPUT_HANDLER_H 