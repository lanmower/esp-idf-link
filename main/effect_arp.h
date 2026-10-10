#ifndef EFFECT_ARP_H
#define EFFECT_ARP_H

#include "main.h"
#include <array>
#include <vector>
#include "synth_interface.h"
#include "ableton/Link.hpp"
#include <chrono>
#include "midi_file.h"
#include "touch_handler.h"

enum ArpMode { ARP_MODE_NOTE = 0, ARP_MODE_CHORD = 1 };
extern ArpMode g_current_arp_mode;

bool handle_arp_active(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, int note_index);
bool handle_arp_adjusting_pads(const ableton::Link::SessionState& state, const std::chrono::microseconds& time, const bool pad_pressed_this_tick[], std::array<bool, NUM_TOUCH_PADS>& pads_used);

#define SUBMENU_FILTER     0
#define SUBMENU_REVERSE    1
#define SUBMENU_SIDECHAIN  2

void reset_arp_to_midi_player();

void handle_arp_adjust_pots(int pot1_delta, int pot2_delta, bool& pot1_used, bool& pot2_used);

extern MidiFilePlayer g_midi_player;

extern int g_current_arp_transpose;
extern double g_current_arp_playback_rate;
extern bool g_midi_player_active;

#endif
