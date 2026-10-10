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
    double startBeat;
    double durationBeats;
};

struct MidiCC {
    uint8_t controller;
    uint8_t value;
    double timeInBeats;
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

    void setFolder(const std::string& folderPath);
    void nextFile();
    bool start();
    void stop();

    void setTranspose(int semitones);
    void setPlaybackRate(double rate);
    void setNoteLengthScale(double scale);
    void setVelocityScale(double scale);

    void process(const ableton::Link::SessionState& sessionState,
                const std::chrono::microseconds& time);

    std::string getCurrentFileName() const;
    const char* getCurrentFolder() const { return folderPath.c_str(); }

    size_t getCurrentFileIndex() const { return currentFileIndex; }
    void setCurrentFileIndex(size_t index);

    int getPot1Value() const { return noteLengthScalePotValue; }
    void setPot1Value(int value) { noteLengthScalePotValue = value; }

    int getPot2Value() const { return velocityScalePotValue; }
    void setPot2Value(int value) { velocityScalePotValue = value; }

private:
    static constexpr int kPotCentreValue = 64;

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

    int noteLengthScalePotValue = kPotCentreValue;
    int velocityScalePotValue   = kPotCentreValue;

    double lastTempo;
    bool syncToBpm;
    bool quantizeLoops;
    double linkQuantum;

    struct NoteEvent {
        uint8_t note;
        double startBeat;

        bool operator==(const NoteEvent& other) const {
            return note == other.note && startBeat == other.startBeat;
        }
    };

    struct NoteEventHash {
        std::size_t operator()(const NoteEvent& event) const {
            return (std::hash<uint8_t>()(event.note) ^
                   (std::hash<double>()(event.startBeat) << 1));
        }
    };

    std::unordered_set<NoteEvent, NoteEventHash> playedNotes;

    struct CCEvent {
        uint8_t controller;
        double timeInBeats;

        bool operator==(const CCEvent& other) const {
            return controller == other.controller && timeInBeats == other.timeInBeats;
        }
    };

    struct CCEventHash {
        std::size_t operator()(const CCEvent& event) const {
            return (std::hash<uint8_t>()(event.controller) ^
                   (std::hash<double>()(event.timeInBeats) << 1));
        }
    };

    std::unordered_set<CCEvent, CCEventHash> sentCCs;

    struct ActiveNote {
        uint8_t note;
        uint8_t velocity;
        double endBeat;
    };
    std::vector<ActiveNote> activeNotes;

    void clearPlayedNotes();
    void processNoteOffs(double currentBeat);
    void updateTempo(const ableton::Link::SessionState& sessionState);
    double calculateQuantizedLoopPoint(double currentBeat, const MidiTrack& track);
    void loadFileInternal(size_t fileIndex);

    bool pendingFileSwitch = false;
    size_t pendingFileIndex = 0;
};

#endif // MIDI_FILE_H
