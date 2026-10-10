#include "midi_file.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <errno>
#include <inttypes.h>
#include "synth_interface.h"
#include "state_machine.h"

static const char *TAG = "MIDI_FILE";

static constexpr int     kFileOpenMaxRetries       = 3;
static constexpr int     kFileOpenRetryDelayMs     = 10;
static constexpr int     kMaxLoadAttempts          = 3;
static constexpr size_t  kMaxTrackEvents           = 1024;

static constexpr size_t  kMidiHeaderSize           = 14;
static constexpr size_t  kTrackHeaderSize          = 8;

static constexpr uint8_t kStatusByteFlag           = 0x80;
static constexpr uint8_t kStatusChannelMask        = 0xF0;
static constexpr uint8_t kStatusNoteOn             = 0x90;
static constexpr uint8_t kStatusNoteOff            = 0x80;
static constexpr uint8_t kStatusControlChange      = 0xB0;
static constexpr uint8_t kStatusProgramChange      = 0xC0;
static constexpr uint8_t kStatusChannelPressure    = 0xD0;
static constexpr uint8_t kStatusMeta               = 0xFF;
static constexpr uint8_t kStatusSysEx              = 0xF0;
static constexpr uint8_t kStatusSysExEscape        = 0xF7;

static constexpr uint8_t kMetaTempo                = 0x51;
static constexpr uint8_t kMetaEndOfTrack           = 0x2F;
static constexpr size_t  kTempoMetaLength          = 3;

static constexpr int32_t kNoActiveNoteTick         = -1;
static constexpr int     kMidiNoteCount            = 128;
static constexpr int     kMidiValueMin             = 0;
static constexpr int     kMidiValueMax             = 127;

static constexpr uint8_t kMiddleCNote              = 60;
static constexpr uint8_t kDefaultNoteVelocity      = 100;
static constexpr double  kDefaultNoteStartBeat     = 0.0;
static constexpr double  kDefaultNoteDurationBeats = 1.0;
static constexpr double  kDefaultTrackLengthBeats  = 1.0;

static constexpr double  kInitialTempoBpm          = 120.0;
static constexpr double  kTempoChangeThresholdBpm  = 0.5;
static constexpr double  kReferenceTempoBpm        = 120.0;
static constexpr double  kQuantumRoundTolerance    = 0.1;
static constexpr int     kEmptyTrackLogInterval    = 100;
static constexpr double  kEventTriggerWindowBeats  = 0.03;

static constexpr const char* kSpiffsRootDir        = "/spiffs";
static constexpr const char* kSpiffsMountPrefix    = "/spiffs/";
static constexpr const char* kMidiFileExtension    = ".mid";

static constexpr uint8_t kDefaultMidiFileHeader[] = {
    'M', 'T', 'h', 'd',
    0, 0, 0, 6,
    0, 1,
    0, 1,
    0, 96
};

static constexpr uint8_t kDefaultMidiFileTrack[] = {
    'M', 'T', 'r', 'k',
    0, 0, 0, 19,
    0, 0x90, 60, 64,
    96, 0x80, 60, 0,
    0, 0xFF, 0x2F, 0
};

static bool isStatusByte(uint8_t byte) { return (byte & kStatusByteFlag) != 0; }
static bool isNoteOffVelocity(uint8_t velocity) { return velocity == 0; }
static bool isNoteHeld(int32_t startTick) { return startTick != kNoActiveNoteTick; }

static bool hasMidiFileExtension(const std::string& name) {
    return name.find(kMidiFileExtension) != std::string::npos;
}

static std::string baseNameOf(const std::string& path) {
    const size_t lastSlash = path.find_last_of('/');
    return (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
}

static void addFallbackNoteIfTrackHasNoNotes(MidiTrack& track) {
    if (!track.notes.empty()) return;
    ESP_LOGW(TAG, "No notes found in MIDI file, adding a default C4 note");
    track.notes.push_back(MidiNote{kMiddleCNote, kDefaultNoteVelocity,
                                   kDefaultNoteStartBeat, kDefaultNoteDurationBeats});
    track.lengthInBeats = kDefaultTrackLengthBeats;
}

MidiFile::MidiFile(const std::string& filename) :
    filename(filename) {
    ESP_LOGD(TAG, "MidiFile constructor called for file: %s", filename.c_str());
}

MidiFile::~MidiFile() {
    ESP_LOGD(TAG, "MidiFile destructor called for file: %s", filename.c_str());
}

bool MidiFile::load() {
    ESP_LOGI(TAG, "Loading MIDI file: %s", filename.c_str());

    track.notes.clear();
    track.ccs.clear();
    track.lengthInBeats = 0;

    FILE* file = nullptr;
    int retries = 0;
    while (!file && retries < kFileOpenMaxRetries) {
        file = fopen(filename.c_str(), "rb");
        if (!file) {
            retries++;
            ESP_LOGW(TAG, "Failed to open MIDI file (attempt %d/%d): %s (errno: %d, %s)",
                    retries, kFileOpenMaxRetries, filename.c_str(), errno, strerror(errno));

            if (retries < kFileOpenMaxRetries) {
                vTaskDelay(pdMS_TO_TICKS(kFileOpenRetryDelayMs));
            }
        }
    }

    if (!file) {
        ESP_LOGE(TAG, "Failed to open MIDI file after %d attempts: %s",
                kFileOpenMaxRetries, filename.c_str());
        return false;
    }

    struct FileGuard {
        FILE* f;
        explicit FileGuard(FILE* file) : f(file) {}
        ~FileGuard() {
            if (f) {
                fclose(f);
                ESP_LOGD(TAG, "File closed by guard");
            }
        }
    };
    FileGuard guard(file);

    bool result = false;
    try {
        result = parseFile();
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception while parsing MIDI file: %s", e.what());
        result = false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception while parsing MIDI file");
        result = false;
    }

    if (result) {
        ESP_LOGI(TAG, "Successfully loaded MIDI file: %s", filename.c_str());
        ESP_LOGI(TAG, "  Track length: %.2f beats", track.lengthInBeats);
        ESP_LOGI(TAG, "  Number of notes: %zu", track.notes.size());
        ESP_LOGI(TAG, "  Number of CCs: %zu", track.ccs.size());

        if (!track.notes.empty()) {
            const int maxNotesToLog = std::min(5, static_cast<int>(track.notes.size()));
            ESP_LOGI(TAG, "  First %d notes:", maxNotesToLog);
            for (int i = 0; i < maxNotesToLog; i++) {
                ESP_LOGI(TAG, "    Note %d: pitch=%d, vel=%d, start=%.2f, dur=%.2f",
                        i+1, track.notes[i].note, track.notes[i].velocity,
                        track.notes[i].startBeat, track.notes[i].durationBeats);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to parse MIDI file: %s", filename.c_str());
    }

    return result;
}

bool MidiFile::parseFile() {
    FILE* file = fopen(filename.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", filename.c_str());
        return false;
    }

    struct FileGuard {
        FILE* f;
        explicit FileGuard(FILE* file) : f(file) {}
        ~FileGuard() { if (f) fclose(f); }
    };
    FileGuard guard(file);

    uint8_t headerChunk[kMidiHeaderSize];
    if (fread(headerChunk, 1, kMidiHeaderSize, file) != kMidiHeaderSize) {
        ESP_LOGE(TAG, "Failed to read MIDI header");
        return false;
    }

    if (strncmp((char*)headerChunk, "MThd", 4) != 0) {
        ESP_LOGE(TAG, "Not a valid MIDI file");
        return false;
    }

    uint16_t ticksPerQuarterNote = (headerChunk[12] << 8) | headerChunk[13];
    ESP_LOGI(TAG, "MIDI file division");

    uint16_t format = (headerChunk[8] << 8) | headerChunk[9];
    uint16_t numTracks = (headerChunk[10] << 8) | headerChunk[11];
    ESP_LOGI(TAG, "MIDI format: %u, Number of tracks: %u", format, numTracks);

    for (uint16_t trackIdx = 0; trackIdx < numTracks; trackIdx++) {
        uint8_t trackHeader[kTrackHeaderSize];
        if (fread(trackHeader, 1, kTrackHeaderSize, file) != kTrackHeaderSize) {
            ESP_LOGE(TAG, "Failed to read track header %u", trackIdx);
            return false;
        }

        if (strncmp((char*)trackHeader, "MTrk", 4) != 0) {
            ESP_LOGE(TAG, "Invalid track chunk in track %u", trackIdx);
            return false;
        }

        uint32_t trackLength = (trackHeader[4] << 24) | (trackHeader[5] << 16) |
                              (trackHeader[6] << 8) | trackHeader[7];
        ESP_LOGI(TAG, "Track %u length: %" PRIu32 " bytes", trackIdx, trackLength);

        long trackStartPos = ftell(file);

        uint32_t absoluteTicks = 0;

        std::array<int32_t, kMidiNoteCount> noteStartTicks;
        noteStartTicks.fill(kNoActiveNoteTick);

        std::array<uint8_t, kMidiNoteCount> noteVelocities;
        noteVelocities.fill(0);

        uint8_t buffer[3];
        uint8_t status = 0;

        size_t eventCount = 0;

        while (ftell(file) < trackStartPos + trackLength && eventCount < kMaxTrackEvents) {
            eventCount++;
            uint32_t deltaTime = 0;
            uint8_t byte;

            do {
                if (fread(&byte, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read delta time byte");
                    break;
                }
                deltaTime = (deltaTime << 7) | (byte & 0x7F);
            } while (byte & kStatusByteFlag);

            absoluteTicks += deltaTime;

            if (fread(&byte, 1, 1, file) != 1) {
                ESP_LOGE(TAG, "Failed to read event type");
                break;
            }

            if (isStatusByte(byte)) {
                status = byte;
                if (status != kStatusMeta && status != kStatusSysEx && status != kStatusSysExEscape) {
                    if (fread(buffer, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read first data byte");
                        break;
                    }
                }
            } else {
                buffer[0] = byte;
            }

            if ((status & kStatusChannelMask) == kStatusNoteOn) {
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte for Note On");
                    break;
                }

                uint8_t note = buffer[0];
                uint8_t velocity = buffer[1];

                if (!isNoteOffVelocity(velocity)) {
                    noteStartTicks[note] = absoluteTicks;
                    noteVelocities[note] = velocity;
                    ESP_LOGD(TAG, "Note On: %u at tick %" PRIu32 " with velocity %u", note, absoluteTicks, velocity);
                } else {
                    if (isNoteHeld(noteStartTicks[note])) {
                        uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                        double startBeat = (double)noteStartTicks[note] / ticksPerQuarterNote;
                        double durationBeats = (double)durationTicks / ticksPerQuarterNote;

                        if (durationBeats > 0) {
                            MidiNote midiNote = {
                                note,
                                noteVelocities[note],
                                startBeat,
                                durationBeats
                            };
                            track.notes.push_back(midiNote);
                            ESP_LOGD(TAG, "Added note %u: start=%.2f, duration=%.2f",
                                    note, startBeat, durationBeats);
                        }

                        noteStartTicks[note] = kNoActiveNoteTick;
                        noteVelocities[note] = 0;
                    }
                }
            } else if ((status & kStatusChannelMask) == kStatusNoteOff) {
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    break;
                }

                uint8_t note = buffer[0];

                if (isNoteHeld(noteStartTicks[note])) {
                    uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                    double startBeat = (double)noteStartTicks[note] / ticksPerQuarterNote;
                    double durationBeats = (double)durationTicks / ticksPerQuarterNote;

                    if (durationBeats > 0) {
                        MidiNote midiNote = {
                            note,
                            noteVelocities[note],
                            startBeat,
                            durationBeats
                        };
                        track.notes.push_back(midiNote);
                    }

                    noteStartTicks[note] = kNoActiveNoteTick;
                    noteVelocities[note] = 0;
                }
            } else if (status == kStatusMeta) {
                uint8_t metaType;
                if (fread(&metaType, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read meta event type");
                    break;
                }

                uint32_t metaLength = 0;
                do {
                    if (fread(&byte, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read meta event length");
                        break;
                    }
                    metaLength = (metaLength << 7) | (byte & 0x7F);
                } while (byte & kStatusByteFlag);

                if (metaType == kMetaTempo) {
                    if (metaLength == kTempoMetaLength) {
                        uint8_t tempoBytes[kTempoMetaLength];
                        if (fread(tempoBytes, 1, kTempoMetaLength, file) != kTempoMetaLength) {
                            ESP_LOGE(TAG, "Failed to read tempo meta event data");
                            break;
                        }

                        uint32_t microsecondsPerQuarter = (tempoBytes[0] << 16) |
                                                        (tempoBytes[1] << 8) |
                                                        tempoBytes[2];

                        double bpm = 60000000.0 / microsecondsPerQuarter;
                        ESP_LOGI(TAG, "Tempo: %.1f BPM", bpm);
                    } else {
                        fseek(file, metaLength, SEEK_CUR);
                    }
                } else if (metaType == kMetaEndOfTrack) {
                    ESP_LOGI(TAG, "End of track marker found");
                    fseek(file, trackStartPos + trackLength, SEEK_SET);
                    break;
                } else {
                    fseek(file, metaLength, SEEK_CUR);
                }
            } else if (status == kStatusSysEx || status == kStatusSysExEscape) {
                uint32_t sysexLength = 0;
                do {
                    if (fread(&byte, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read SysEx length");
                        break;
                    }
                    sysexLength = (sysexLength << 7) | (byte & 0x7F);
                } while (byte & kStatusByteFlag);

                fseek(file, sysexLength, SEEK_CUR);
            } else if ((status & kStatusChannelMask) == kStatusControlChange) {
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte for Control Change");
                    break;
                }

                uint8_t controller = buffer[0];
                uint8_t value = buffer[1];
                double timeInBeats = (double)absoluteTicks / ticksPerQuarterNote;

                MidiCC cc = {controller, value, timeInBeats};
                track.ccs.push_back(cc);
                ESP_LOGD(TAG, "CC: controller=%u value=%u at beat %.2f", controller, value, timeInBeats);
            } else if ((status & kStatusChannelMask) == kStatusProgramChange ||
                       (status & kStatusChannelMask) == kStatusChannelPressure) {
            } else {
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte");
                    break;
                }
            }
        }

        for (uint8_t note = 0; note < kMidiNoteCount; note++) {
            if (isNoteHeld(noteStartTicks[note])) {
                uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                double startBeat = (double)noteStartTicks[note] / ticksPerQuarterNote;
                double durationBeats = (double)durationTicks / ticksPerQuarterNote;

                if (durationBeats > 0) {
                    MidiNote midiNote = {
                        note,
                        noteVelocities[note],
                        startBeat,
                        durationBeats
                    };
                    track.notes.push_back(midiNote);
                    ESP_LOGD(TAG, "Added note %u at end of track: start=%.2f, duration=%.2f",
                            note, startBeat, durationBeats);
                }
            }
        }
    }

    track.lengthInBeats = 0;
    for (const auto& note : track.notes) {
        double noteEndBeat = note.startBeat + note.durationBeats;
        if (noteEndBeat > track.lengthInBeats) {
            track.lengthInBeats = noteEndBeat;
        }
    }

    addFallbackNoteIfTrackHasNoNotes(track);

    std::sort(track.notes.begin(), track.notes.end(), [](const MidiNote& a, const MidiNote& b) {
        return a.startBeat < b.startBeat;
    });

    ESP_LOGI(TAG, "Loaded MIDI file with %zu notes, %zu CCs, length: %.2f beats",
             track.notes.size(), track.ccs.size(), track.lengthInBeats);

    fclose(file);
    return true;
}

MidiFilePlayer::MidiFilePlayer() :
    currentFileIndex(0),
    currentFile(nullptr),
    transpose(0),
    playbackRate(1.0),
    noteLengthScale(1.0),
    velocityScale(1.0),
    isPlaying(false),
    lastBeat(0),
    lastTempo(kInitialTempoBpm),
    syncToBpm(false),
    quantizeLoops(true),
    linkQuantum(LINK_QUANTUM) {
    ESP_LOGI(TAG, "MidiFilePlayer initialized with Link quantum: %.1f beats", linkQuantum);
}

MidiFilePlayer::~MidiFilePlayer() {
}

void MidiFilePlayer::clearPlayedNotes() {
    playedNotes.clear();
    sentCCs.clear();
}

void MidiFilePlayer::setFolder(const std::string& folderPath) {
    this->folderPath = folderPath;

    ESP_LOGI(TAG, "Setting MIDI folder: %s", folderPath.c_str());

    midiFiles.clear();

    DIR* dir = opendir(folderPath.c_str());
    if (dir) {
        struct dirent* entry;
        int fileCount = 0;
        ESP_LOGI(TAG, "Scanning for MIDI files in: %s", folderPath.c_str());
        while ((entry = readdir(dir)) != NULL) {
            std::string filename = entry->d_name;

            if (hasMidiFileExtension(filename)) {
                midiFiles.push_back(folderPath + "/" + filename);
                fileCount++;
                ESP_LOGI(TAG, "Found MIDI file: %s", filename.c_str());
            }
        }
        closedir(dir);
        ESP_LOGI(TAG, "Found %d MIDI files in %s", fileCount, folderPath.c_str());
    } else {
        std::string prefix = folderPath;
        if (prefix.compare(0, std::strlen(kSpiffsMountPrefix), kSpiffsMountPrefix) == 0) {
            prefix = prefix.substr(std::strlen(kSpiffsMountPrefix));
        }

        if (prefix.back() != '/') {
            prefix += '/';
        }

        ESP_LOGI(TAG, "Opening root directory to search for prefix: %s", prefix.c_str());

        DIR* rootDir = opendir(kSpiffsRootDir);
        if (rootDir) {
            struct dirent* entry;
            int fileCount = 0;

            ESP_LOGI(TAG, "Scanning SPIFFS root for files with prefix: %s", prefix.c_str());

            while ((entry = readdir(rootDir)) != NULL) {
                std::string entryPath = entry->d_name;

                ESP_LOGD(TAG, "Checking entry: %s", entryPath.c_str());

                if (entryPath.find(prefix) == 0 && hasMidiFileExtension(entryPath)) {
                    std::string fullPath = std::string(kSpiffsRootDir) + "/" + entryPath;
                    midiFiles.push_back(fullPath);
                    fileCount++;

                    ESP_LOGI(TAG, "Found MIDI file: %s (full path: %s)",
                             baseNameOf(entryPath).c_str(), fullPath.c_str());
                }
            }
            closedir(rootDir);

            if (fileCount > 0) {
                ESP_LOGI(TAG, "Found %d MIDI files with prefix %s", fileCount, prefix.c_str());
            } else {
                ESP_LOGW(TAG, "No MIDI files found with prefix %s", prefix.c_str());

                rootDir = opendir(kSpiffsRootDir);
                if (rootDir) {
                    ESP_LOGI(TAG, "Listing all MIDI files in SPIFFS:");
                    struct dirent* entry;
                    while ((entry = readdir(rootDir)) != NULL) {
                        std::string path = entry->d_name;
                        if (hasMidiFileExtension(path)) {
                            ESP_LOGI(TAG, "  %s", path.c_str());
                        }
                    }
                    closedir(rootDir);
                }
            }
        } else {
            ESP_LOGE(TAG, "Could not open SPIFFS root directory (errno: %d, %s)",
                    errno, strerror(errno));
        }
    }

    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "No MIDI files found in %s, creating a default one", folderPath.c_str());

        std::string defaultFilePath = folderPath + "/default.mid";

        FILE* file = fopen(defaultFilePath.c_str(), "wb");
        if (file) {
            fwrite(kDefaultMidiFileHeader, 1, sizeof(kDefaultMidiFileHeader), file);
            fwrite(kDefaultMidiFileTrack, 1, sizeof(kDefaultMidiFileTrack), file);
            fclose(file);

            midiFiles.push_back(defaultFilePath);
            ESP_LOGI(TAG, "Created default MIDI file: %s", defaultFilePath.c_str());
        } else {
            ESP_LOGE(TAG, "Failed to create default MIDI file (errno: %d, %s)",
                    errno, strerror(errno));
        }
    }

    std::sort(midiFiles.begin(), midiFiles.end());

    if (!midiFiles.empty()) {
        currentFileIndex = 0;
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
        ESP_LOGI(TAG, "Loading MIDI file: %s", midiFiles[currentFileIndex].c_str());
        if (!currentFile->load()) {
            ESP_LOGE(TAG, "Failed to load MIDI file: %s", midiFiles[currentFileIndex].c_str());
            currentFile.reset();
        } else {
            ESP_LOGI(TAG, "Successfully loaded MIDI file with %zu notes",
                    currentFile->getTrack().notes.size());
        }
    } else {
        ESP_LOGE(TAG, "No MIDI files available in folder: %s", folderPath.c_str());
    }
}

void MidiFilePlayer::nextFile() {
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "No MIDI files available to cycle through");
        return;
    }

    size_t nextIndex = (currentFileIndex + 1) % midiFiles.size();

    pendingFileSwitch = true;
    pendingFileIndex = nextIndex;

    ESP_LOGI(TAG, "Scheduled file switch from %zu to %zu at next quantum boundary",
             currentFileIndex, nextIndex);

    ESP_LOGI(TAG, "Available MIDI files in folder %s:", folderPath.c_str());
    for (size_t i = 0; i < midiFiles.size(); i++) {
        ESP_LOGI(TAG, "  %s%zu: %s",
                (i == currentFileIndex) ? "-> " : (i == nextIndex) ? "=> " : "   ",
                i + 1,
                midiFiles[i].c_str());
    }
}

void MidiFilePlayer::loadFileInternal(size_t fileIndex) {
    if (fileIndex >= midiFiles.size()) {
        ESP_LOGE(TAG, "Invalid file index: %zu (max: %zu)", fileIndex, midiFiles.size() - 1);
        return;
    }

    currentFileIndex = fileIndex;

    ESP_LOGI(TAG, "Loading MIDI file %zu at quantum boundary: %s",
            currentFileIndex + 1, midiFiles[currentFileIndex].c_str());

    struct stat st;
    if (stat(midiFiles[currentFileIndex].c_str(), &st) != 0) {
        ESP_LOGE(TAG, "File doesn't exist or can't be accessed: %s (errno: %d, %s)",
                midiFiles[currentFileIndex].c_str(), errno, strerror(errno));

        setFolder(folderPath);

        if (!midiFiles.empty()) {
            currentFileIndex = 0;
        } else {
            return;
        }
    }

    try {
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
        if (!currentFile->load()) {
            ESP_LOGE(TAG, "Failed to load MIDI file: %s", midiFiles[currentFileIndex].c_str());
            currentFile.reset();

            if (midiFiles.size() > 1) {
                currentFileIndex = (currentFileIndex + 1) % midiFiles.size();

                currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
                if (!currentFile->load()) {
                    ESP_LOGE(TAG, "Failed to load next MIDI file as well");
                    currentFile.reset();
                    return;
                }
            } else {
                return;
            }
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception while loading MIDI file: %s", e.what());
        currentFile.reset();
        return;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception while loading MIDI file");
        currentFile.reset();
        return;
    }

    ESP_LOGI(TAG, "Loaded MIDI file: %s at quantum boundary",
             baseNameOf(midiFiles[currentFileIndex]).c_str());

    clearPlayedNotes();
    activeNotes.clear();
}

bool MidiFilePlayer::start() {
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "No MIDI files available to play");
        return false;
    }

    bool loaded = false;
    int attempts = 0;

    while (!loaded && attempts < kMaxLoadAttempts && attempts < midiFiles.size()) {
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);

        loaded = currentFile->load();
        if (!loaded) {
            ESP_LOGW(TAG, "Failed to load MIDI file %s, trying next file",
                    midiFiles[currentFileIndex].c_str());

            currentFileIndex = (currentFileIndex + 1) % midiFiles.size();
            attempts++;
        } else {
            const MidiTrack& track = currentFile->getTrack();

            double lengthInQuantums = track.lengthInBeats / linkQuantum;
            double remainder = lengthInQuantums - std::floor(lengthInQuantums);

            if (remainder > 0.01 && remainder < 0.99) {
                double adjustedLength = calculateQuantizedLoopPoint(0, track);
                ESP_LOGI(TAG, "MIDI file length (%.2f beats) is not a multiple of quantum (%.1f). "
                         "Will quantize to %.2f beats for sync.",
                         track.lengthInBeats, linkQuantum, adjustedLength);
            } else {
                ESP_LOGI(TAG, "MIDI file loaded with length %.2f beats (%.2f quantums)",
                         track.lengthInBeats, lengthInQuantums);
            }
        }
    }

    if (!loaded) {
        ESP_LOGE(TAG, "Failed to load any MIDI file after %d attempts", attempts);
        currentFile.reset();
        return false;
    }

    playedNotes.clear();
    activeNotes.clear();

    isPlaying = true;
    lastBeat = 0;

    return true;
}

void MidiFilePlayer::stop() {
    isPlaying = false;

    if (g_current_synth) {
        g_current_synth->sendAllNotesOff();
    }

    ESP_LOGI(TAG, "Stopped MIDI playback");
}

void MidiFilePlayer::setTranspose(int semitones) {
    transpose = semitones;
    ESP_LOGI(TAG, "Set transpose: %d semitones", transpose);
}

void MidiFilePlayer::setPlaybackRate(double rate) {
    playbackRate = rate;
    ESP_LOGI(TAG, "Set playback rate: %.2fx", playbackRate);
}

void MidiFilePlayer::setNoteLengthScale(double scale) {
    noteLengthScale = scale;
    ESP_LOGI(TAG, "Set note length scale: %.0f%%", noteLengthScale * 100);
}

void MidiFilePlayer::setVelocityScale(double scale) {
    velocityScale = scale;
    ESP_LOGI(TAG, "Set velocity scale: %.0f%%", velocityScale * 100);
}

void MidiFilePlayer::processNoteOffs(double currentBeat) {
    if (activeNotes.empty()) {
        return;
    }

    auto it = activeNotes.begin();
    while (it != activeNotes.end()) {
        if (it->endBeat <= currentBeat) {
            if (g_current_synth) {
                g_current_synth->sendNoteOff(it->note, 0);
            }

            it = activeNotes.erase(it);
        } else {
            ++it;
        }
    }
}

void MidiFilePlayer::updateTempo(const ableton::Link::SessionState& sessionState) {
    if (!syncToBpm) return;

    double currentTempo = sessionState.tempo();

    if (std::abs(currentTempo - lastTempo) > kTempoChangeThresholdBpm) {
        playbackRate = currentTempo / kReferenceTempoBpm;

        ESP_LOGI(TAG, "Adjusted playback rate to %.2fx (Link tempo: %.1f BPM)",
                 playbackRate, currentTempo);

        lastTempo = currentTempo;
    }
}

double MidiFilePlayer::calculateQuantizedLoopPoint(double currentBeat, const MidiTrack& track) {
    if (!quantizeLoops) {
        return track.lengthInBeats;
    }

    if (track.lengthInBeats <= linkQuantum) {
        return linkQuantum;
    }

    double numQuantums = track.lengthInBeats / linkQuantum;

    double roundedQuantums = std::round(numQuantums);

    if (std::abs(numQuantums - roundedQuantums) < kQuantumRoundTolerance) {
        return roundedQuantums * linkQuantum;
    }

    double ceilingQuantums = std::ceil(numQuantums);

    ESP_LOGI(TAG, "Quantizing track length: %.2f beats to %.2f quantums (%.2f beats)",
             track.lengthInBeats, ceilingQuantums, ceilingQuantums * linkQuantum);

    return ceilingQuantums * linkQuantum;
}

void MidiFilePlayer::process(const ableton::Link::SessionState& sessionState,
                           const std::chrono::microseconds& time) {
    if (!isPlaying) {
        ESP_LOGD(TAG, "MIDI player not playing, skipping process");
        return;
    }

    if (!currentFile) {
        ESP_LOGW(TAG, "No MIDI file loaded, but player is active");
        return;
    }

    const auto beats = sessionState.beatAtTime(time, linkQuantum);

    const MidiTrack& track = currentFile->getTrack();

    updateTempo(sessionState);

    processNoteOffs(beats * playbackRate);

    double effectiveTrackLength = calculateQuantizedLoopPoint(0.0, track);
    if (effectiveTrackLength <= 0.0) {
        effectiveTrackLength = linkQuantum;
    }

    const double scaledBeats = beats * playbackRate;
    double loopedBeat = fmod(scaledBeats, effectiveTrackLength);
    if (loopedBeat < 0) loopedBeat += effectiveTrackLength;

    bool loopBoundaryCrossed = loopedBeat < lastBeat;
    if (loopBoundaryCrossed) {
        double currentPhase = sessionState.phaseAtTime(time, linkQuantum);
        int currentQuantumNumber = static_cast<int>(std::floor(beats / linkQuantum));

        ESP_LOGD(TAG, "MIDI loop restart at quantum boundary %d (beat %.2f, phase: %.2f)",
                 currentQuantumNumber, beats, currentPhase);

        if (g_current_synth) {
            g_current_synth->sendAllNotesOff();
        }
        activeNotes.clear();

        clearPlayedNotes();

        if (pendingFileSwitch && currentPhase < 0.1) {
            loadFileInternal(pendingFileIndex);
            pendingFileSwitch = false;
            ESP_LOGI(TAG, "Switched to new file at quantum boundary");
        }
    }

    lastBeat = loopedBeat;

    if (track.notes.empty()) {
        static int emptyLogCounter = 0;
        if (++emptyLogCounter > kEmptyTrackLogInterval) {
            ESP_LOGW(TAG, "No notes in the MIDI track! Current file: %s",
                     midiFiles.empty() ? "None" : midiFiles[currentFileIndex].c_str());
            emptyLogCounter = 0;
        }
        return;
    }

    for (const auto& note : track.notes) {
        if (note.startBeat > loopedBeat + kEventTriggerWindowBeats) {
            break;
        }

        if (loopedBeat >= note.startBeat &&
            loopedBeat < note.startBeat + kEventTriggerWindowBeats) {

            NoteEvent event{note.note, note.startBeat};

            if (playedNotes.find(event) != playedNotes.end()) {
                continue;
            }

            int transposedNote = note.note + transpose;

            if (transposedNote >= kMidiValueMin && transposedNote <= kMidiValueMax) {
                if (g_current_synth) {
                    int scaledVelocity = static_cast<int>(note.velocity * velocityScale);
                    scaledVelocity = std::min(kMidiValueMax, std::max(kMidiValueMin, scaledVelocity));

                    g_current_synth->sendNoteOn(transposedNote, scaledVelocity);

                    playedNotes.insert(event);

                    double scaledDuration = note.durationBeats * noteLengthScale;
                    double endBeatAbsolute = scaledBeats + scaledDuration;

                    activeNotes.push_back({
                        static_cast<uint8_t>(transposedNote),
                        static_cast<uint8_t>(scaledVelocity),
                        endBeatAbsolute
                    });
                } else {
                    ESP_LOGW(TAG, "Cannot play note - synth interface is null");
                }
            }
        }
    }

    for (const auto& cc : track.ccs) {
        if (loopedBeat >= cc.timeInBeats &&
            loopedBeat < cc.timeInBeats + kEventTriggerWindowBeats) {

            CCEvent event{cc.controller, cc.timeInBeats};

            if (sentCCs.find(event) != sentCCs.end()) {
                continue;
            }

            if (g_current_synth) {
                g_current_synth->sendControlChange(cc.controller, cc.value);
                ESP_LOGD(TAG, "Sent CC: controller=%u value=%u at beat %.2f",
                        cc.controller, cc.value, loopedBeat);

                sentCCs.insert(event);
            }
        }
    }
}

std::string MidiFilePlayer::getCurrentFileName() const {
    if (midiFiles.empty()) return "";

    return baseNameOf(midiFiles[currentFileIndex]);
}

void MidiFilePlayer::setCurrentFileIndex(size_t index) {
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "Cannot set file index - no files available");
        return;
    }

    if (index >= midiFiles.size()) {
        index = midiFiles.size() - 1;
        ESP_LOGW(TAG, "Requested file index %zu out of range, clamping to %zu", index, index);
    }

    currentFileIndex = index;
    ESP_LOGI(TAG, "Set current file index to %zu", currentFileIndex);
}
