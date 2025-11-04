#ifndef MIDI_FILE_H
#define MIDI_FILE_H

#include <vector>
#include <string>
#include <cstdint>
#include <memory>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include "ableton/Link.hpp"

struct MidiNote {
    uint8_t note;
    uint8_t velocity;
    double startTime;  // In beats
    double duration;   // In beats
};

struct MidiCC {
    uint8_t controller;
    uint8_t value;
    double time;  // In beats
};

struct MidiTrack {
    std::vector<MidiNote> notes;
    std::vector<MidiCC> ccs;
    double lengthInBeats;
};

class MidiFile {
public:
    MidiFile(const std::string& filename);
    ~MidiFile();

    bool load();
    const MidiTrack& getTrack() const { return track; }
    
private:
    std::string filename;
    MidiTrack track;
    bool parseFile();
};

class MidiFilePlayer {
public:
    MidiFilePlayer();
    ~MidiFilePlayer();

    // Set the folder to scan for MIDI files 
    void setFolder(const std::string& folderPath);
    
    // Cycle to the next MIDI file in the folder
    void nextFile();
    
    // Start playback
    bool start();
    
    // Stop playback
    void stop();
    
    // Set the transpose amount in semitones
    void setTranspose(int semitones);
    
    // Set the playback rate multiplier
    void setPlaybackRate(double rate);
    
    // Set note length scale (0.0 to 2.0 = 0% to 200%)
    void setNoteLengthScale(double scale);
    
    // Set velocity scale (0.0 to 1.0)
    void setVelocityScale(double scale);
    
    // Process MIDI playback based on Link timing
    void process(const ableton::Link::SessionState& sessionState, 
                const std::chrono::microseconds& time);
    
    // Get the current file name
    std::string getCurrentFileName() const;
    
    // Get the current folder path
    const char* getCurrentFolder() const { return folderPath.c_str(); }
    
    // Get and set the current file index
    size_t getCurrentFileIndex() const { return currentFileIndex; }
    void setCurrentFileIndex(size_t index);
    
    // Store and retrieve pot1 value (for speed control)
    int getPot1Value() const { return pot1Value; }
    void setPot1Value(int value) { pot1Value = value; }
    
    // Store and retrieve pot2 value (for transpose)
    int getPot2Value() const { return pot2Value; }
    void setPot2Value(int value) { pot2Value = value; }
    
private:
    std::vector<std::string> midiFiles;
    size_t currentFileIndex;
    std::unique_ptr<MidiFile> currentFile;
    std::string folderPath;
    int transpose;
    double playbackRate;
    double noteLengthScale;
    double velocityScale;
    bool isPlaying;
    double lastBeat;
    
    // Pot value storage (for UI control)
    int pot1Value = 64; // Default to middle position
    int pot2Value = 64; // Default to middle position
    
    // Tempo synchronization
    double lastTempo;
    bool syncToBpm;
    bool quantizeLoops;
    double linkQuantum;
    
    // Track played notes to prevent retriggering
    struct NoteEvent {
        uint8_t note;
        double startTime;
        
        // Required for unordered_set
        bool operator==(const NoteEvent& other) const {
            return note == other.note && startTime == other.startTime;
        }
    };
    
    struct NoteEventHash {
        std::size_t operator()(const NoteEvent& event) const {
            return (std::hash<uint8_t>()(event.note) ^ 
                   (std::hash<double>()(event.startTime) << 1));
        }
    };
    
    // Store notes that have been played in the current loop
    std::unordered_set<NoteEvent, NoteEventHash> playedNotes;
    
    // Track CC events to prevent duplicate sends
    struct CCEvent {
        uint8_t controller;
        double time;
        
        bool operator==(const CCEvent& other) const {
            return controller == other.controller && time == other.time;
        }
    };
    
    struct CCEventHash {
        std::size_t operator()(const CCEvent& event) const {
            return (std::hash<uint8_t>()(event.controller) ^ 
                   (std::hash<double>()(event.time) << 1));
        }
    };
    
    std::unordered_set<CCEvent, CCEventHash> sentCCs;
    
    // Store active notes with their end times
    struct ActiveNote {
        uint8_t note;
        uint8_t velocity;
        double endTime;  // Beat time when note should be turned off
    };
    std::vector<ActiveNote> activeNotes;
    
    // Clear played notes when looping
    void clearPlayedNotes();
    
    // Process active notes to send note-offs
    void processNoteOffs(double currentBeat);
    
    // Update tempo based on Link session
    void updateTempo(const ableton::Link::SessionState& sessionState);
    
    // Calculate the correct loop point for quantization
    double calculateQuantizedLoopPoint(double currentBeat, const MidiTrack& track);
    
    // Pending file switch for quantum-aligned switching
    bool pendingFileSwitch = false;
    size_t pendingFileIndex = 0;
    
    // Internal method to load a file immediately
    void loadFileInternal(size_t fileIndex);
};

#endif // MIDI_FILE_H 