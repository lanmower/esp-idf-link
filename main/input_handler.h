#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include <stdint.h>
#include "main.h"
#include "state_machine.h"

void initialize_inputs();

bool read_inputs(InputEvent& current_event);

#endif
