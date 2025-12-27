#ifndef NETWORK_MIDI_H
#define NETWORK_MIDI_H

#include <stddef.h>
#include <stdint.h>

void network_midi_init();
void network_midi_start();
void network_midi_stop();
const char* network_midi_get_device_ip();

#endif
