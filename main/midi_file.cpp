#include "midi_file.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <errno.h>
#include <inttypes.h> // Add this for PRIu32 format specifier
#include "synth_interface.h"
#include "state_machine.h" // For access to g_current_synth

static const char *TAG = "MIDI_FILE";

// Simple MIDI file parser for single-track files
MidiFile::MidiFile(const std::string& filename) : 
    filename(filename) {
    ESP_LOGD(TAG, "MidiFile constructor called for file: %s", filename.c_str());
}

MidiFile::~MidiFile() {
    ESP_LOGD(TAG, "MidiFile destructor called for file: %s", filename.c_str());
}

bool MidiFile::load() {
    ESP_LOGI(TAG, "Loading MIDI file: %s", filename.c_str());
    
    // Clear any existing track data
    track.notes.clear();
    track.ccs.clear();
    track.lengthInBeats = 0;
    
    // Try to open the MIDI file
    FILE* file = nullptr;
    
    // Retry opening the file a few times in case of filesystem issues
    const int MAX_RETRIES = 3;
    int retries = 0;
    while (!file && retries < MAX_RETRIES) {
        file = fopen(filename.c_str(), "rb");
        if (!file) {
            retries++;
            ESP_LOGW(TAG, "Failed to open MIDI file (attempt %d/%d): %s (errno: %d, %s)", 
                    retries, MAX_RETRIES, filename.c_str(), errno, strerror(errno));
            
            if (retries < MAX_RETRIES) {
                // Brief delay before retrying
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
    
    if (!file) {
        ESP_LOGE(TAG, "Failed to open MIDI file after %d attempts: %s", 
                MAX_RETRIES, filename.c_str());
        return false;
    }
    
    // Use a scope guard to ensure file is closed even on exceptions
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
    
    // Log the results of parsing
    if (result) {
        ESP_LOGI(TAG, "Successfully loaded MIDI file: %s", filename.c_str());
        ESP_LOGI(TAG, "  Track length: %.2f beats", track.lengthInBeats);
        ESP_LOGI(TAG, "  Number of notes: %zu", track.notes.size());
        ESP_LOGI(TAG, "  Number of CCs: %zu", track.ccs.size());
        
        // Log some info about the first few notes if available
        if (!track.notes.empty()) {
            const int maxNotesToLog = std::min(5, static_cast<int>(track.notes.size()));
            ESP_LOGI(TAG, "  First %d notes:", maxNotesToLog);
            for (int i = 0; i < maxNotesToLog; i++) {
                ESP_LOGI(TAG, "    Note %d: pitch=%d, vel=%d, start=%.2f, dur=%.2f",
                        i+1, track.notes[i].note, track.notes[i].velocity,
                        track.notes[i].startTime, track.notes[i].duration);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to parse MIDI file: %s", filename.c_str());
    }
    
    return result;
}

// Simplified MIDI file parser that extracts just the note data
bool MidiFile::parseFile() {
    FILE* file = fopen(filename.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", filename.c_str());
        return false;
    }
    
    // Use scope guard to ensure file is properly closed
    struct FileGuard {
        FILE* f;
        explicit FileGuard(FILE* file) : f(file) {}
        ~FileGuard() { if (f) fclose(f); }
    };
    FileGuard guard(file);
    
    // Read MIDI header
    uint8_t headerChunk[14];
    if (fread(headerChunk, 1, 14, file) != 14) {
        ESP_LOGE(TAG, "Failed to read MIDI header");
        return false;
    }
    
    // Check if it's a valid MIDI file (MThd)
    if (strncmp((char*)headerChunk, "MThd", 4) != 0) {
        ESP_LOGE(TAG, "Not a valid MIDI file");
        return false;
    }
    
    // Get PPQ (ticks per quarter note)
    uint16_t division = (headerChunk[12] << 8) | headerChunk[13];
    ESP_LOGI(TAG, "MIDI file division");
    
    // Get format (0, 1, or 2) and number of tracks
    uint16_t format = (headerChunk[8] << 8) | headerChunk[9];
    uint16_t numTracks = (headerChunk[10] << 8) | headerChunk[11];
    ESP_LOGI(TAG, "MIDI format: %u, Number of tracks: %u", format, numTracks);
    
    // Process each track (in format 0 there is only one track)
    for (uint16_t trackIdx = 0; trackIdx < numTracks; trackIdx++) {
        // Process track data
        uint8_t trackHeader[8];
        if (fread(trackHeader, 1, 8, file) != 8) {
            ESP_LOGE(TAG, "Failed to read track header %u", trackIdx);
            return false;
        }
        
        // Check if it's a track chunk (MTrk)
        if (strncmp((char*)trackHeader, "MTrk", 4) != 0) {
            ESP_LOGE(TAG, "Invalid track chunk in track %u", trackIdx);
            return false;
        }
        
        // Get track length
        uint32_t trackLength = (trackHeader[4] << 24) | (trackHeader[5] << 16) | 
                              (trackHeader[6] << 8) | trackHeader[7];
        ESP_LOGI(TAG, "Track %u length: %" PRIu32 " bytes", trackIdx, trackLength);
        
        long trackStartPos = ftell(file);
        
        // Parse track events
        uint32_t absoluteTicks = 0;
        
        // Store the start time (in ticks) of each active note
        std::array<int32_t, 128> noteStartTicks;
        noteStartTicks.fill(-1);  // -1 indicates note is not active
        
        // Store the velocity of each active note
        std::array<uint8_t, 128> noteVelocities;
        noteVelocities.fill(0);
        
        uint8_t buffer[3]; // For reading event data
        uint8_t status = 0;
        
        // Read track events - PERFORMANCE: Reduce buffer size for faster parsing
        const size_t MAX_TRACK_EVENTS = 1024;
        size_t eventCount = 0;
        
        // Read track events
        while (ftell(file) < trackStartPos + trackLength && eventCount < MAX_TRACK_EVENTS) {
            eventCount++;
            // Read delta time
            uint32_t deltaTime = 0;
            uint8_t byte;
            
            do {
                if (fread(&byte, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read delta time byte");
                    break;
                }
                deltaTime = (deltaTime << 7) | (byte & 0x7F);
            } while (byte & 0x80);
            
            absoluteTicks += deltaTime;
            
            // Read event type
            if (fread(&byte, 1, 1, file) != 1) {
                ESP_LOGE(TAG, "Failed to read event type");
                break;
            }
            
            // Handle running status
            if (byte & 0x80) {
                status = byte;
                // Read first data byte for most message types
                if (status != 0xFF && status != 0xF0 && status != 0xF7) {
                    if (fread(buffer, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read first data byte");
                        break;
                    }
                }
            } else {
                // Using running status, first byte is actually first data byte
                buffer[0] = byte;
            }
            
            // Process event based on status
            if ((status & 0xF0) == 0x90) {
                // Note On
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte for Note On");
                    break;
                }
                
                uint8_t note = buffer[0];
                uint8_t velocity = buffer[1];
                
                if (velocity > 0) {
                    // Note On - store start time and velocity
                    noteStartTicks[note] = absoluteTicks;
                    noteVelocities[note] = velocity;
                    ESP_LOGD(TAG, "Note On: %u at tick %" PRIu32 " with velocity %u", note, absoluteTicks, velocity);
                } else {
                    // Note On with velocity 0 is equivalent to Note Off
                    if (noteStartTicks[note] >= 0) {
                        // Calculate duration in ticks and convert to beats
                        uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                        double startTime = (double)noteStartTicks[note] / division;
                        double duration = (double)durationTicks / division;
                        
                        // Only add notes with positive duration
                        if (duration > 0) {
                            MidiNote midiNote = {
                                note,
                                noteVelocities[note],
                                startTime,
                                duration
                            };
                            track.notes.push_back(midiNote);
                            ESP_LOGD(TAG, "Added note %u: start=%.2f, duration=%.2f", 
                                    note, startTime, duration);
                        }
                        
                        // Mark note as inactive
                        noteStartTicks[note] = -1;
                        noteVelocities[note] = 0;
                    }
                }
            } else if ((status & 0xF0) == 0x80) {
                // Note Off
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    //ESP_LOGE(TAG, "Failed to read second data byte for Note Off");
                    break;
                }
                
                uint8_t note = buffer[0];
                
                if (noteStartTicks[note] >= 0) {
                    // Calculate duration in ticks and convert to beats
                    uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                    double startTime = (double)noteStartTicks[note] / division;
                    double duration = (double)durationTicks / division;
                    
                    // Only add notes with positive duration
                    if (duration > 0) {
                        MidiNote midiNote = {
                            note,
                            noteVelocities[note],
                            startTime,
                            duration
                        };
                        track.notes.push_back(midiNote);
                        //ESP_LOGD(TAG, "Added note %u (from Note Off): start=%.2f, duration=%.2f", 
                        //        note, startTime, duration);
                    }
                    
                    // Mark note as inactive
                    noteStartTicks[note] = -1;
                    noteVelocities[note] = 0;
                }
            } else if (status == 0xFF) {
                // Meta event
                uint8_t metaType;
                if (fread(&metaType, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read meta event type");
                    break;
                }
                
                // Read length
                uint32_t metaLength = 0;
                do {
                    if (fread(&byte, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read meta event length");
                        break;
                    }
                    metaLength = (metaLength << 7) | (byte & 0x7F);
                } while (byte & 0x80);
                
                // Handle specific meta events
                if (metaType == 0x51) { // Tempo
                    if (metaLength == 3) {
                        uint8_t tempoBytes[3];
                        if (fread(tempoBytes, 1, 3, file) != 3) {
                            ESP_LOGE(TAG, "Failed to read tempo meta event data");
                            break;
                        }
                        
                        uint32_t microsecondsPerQuarter = (tempoBytes[0] << 16) | 
                                                        (tempoBytes[1] << 8) | 
                                                        tempoBytes[2];
                        
                        double bpm = 60000000.0 / microsecondsPerQuarter;
                        ESP_LOGI(TAG, "Tempo: %.1f BPM", bpm);
                    } else {
                        // Skip meta event data that doesn't match expected length
                        fseek(file, metaLength, SEEK_CUR);
                    }
                } else if (metaType == 0x2F) { // End of track
                    ESP_LOGI(TAG, "End of track marker found");
                    // Skip rest of the track data
                    fseek(file, trackStartPos + trackLength, SEEK_SET);
                    break;
                } else {
                    // Skip other meta event data
                    fseek(file, metaLength, SEEK_CUR);
                }
            } else if (status == 0xF0 || status == 0xF7) {
                // SysEx message
                uint32_t sysexLength = 0;
                do {
                    if (fread(&byte, 1, 1, file) != 1) {
                        ESP_LOGE(TAG, "Failed to read SysEx length");
                        break;
                    }
                    sysexLength = (sysexLength << 7) | (byte & 0x7F);
                } while (byte & 0x80);
                
                // Skip SysEx data
                fseek(file, sysexLength, SEEK_CUR);
            } else if ((status & 0xF0) == 0xB0) {
                // Control Change
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte for Control Change");
                    break;
                }
                
                uint8_t controller = buffer[0];
                uint8_t value = buffer[1];
                double time = (double)absoluteTicks / division;
                
                MidiCC cc = {controller, value, time};
                track.ccs.push_back(cc);
                ESP_LOGD(TAG, "CC: controller=%u value=%u at beat %.2f", controller, value, time);
            } else if ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) {
                // Program Change or Channel Pressure - one data byte, already read
                // No need to read more bytes
            } else {
                // Other channel message types (Pitch Bend, etc.) have 2 data bytes
                // We already read the first data byte for these message types
                if (fread(buffer + 1, 1, 1, file) != 1) {
                    ESP_LOGE(TAG, "Failed to read second data byte");
                    break;
                }
            }
        }
        
        // Add any still-active notes with implicit Note Off at end of track
        for (uint8_t note = 0; note < 128; note++) {
            if (noteStartTicks[note] >= 0) {
                // Use track end as implicit Note Off
                uint32_t durationTicks = absoluteTicks - noteStartTicks[note];
                double startTime = (double)noteStartTicks[note] / division;
                double duration = (double)durationTicks / division;
                
                if (duration > 0) {
                    MidiNote midiNote = {
                        note,
                        noteVelocities[note],
                        startTime,
                        duration
                    };
                    track.notes.push_back(midiNote);
                    ESP_LOGD(TAG, "Added note %u at end of track: start=%.2f, duration=%.2f", 
                            note, startTime, duration);
                }
            }
        }
    }
    
    // Set track length to the last note's end time
    track.lengthInBeats = 0;
    for (const auto& note : track.notes) {
        double endTime = note.startTime + note.duration;
        if (endTime > track.lengthInBeats) {
            track.lengthInBeats = endTime;
        }
    }
    
    // If no notes were found but we've read a valid MIDI file, 
    // create a default note to avoid empty tracks
    if (track.notes.empty()) {
        ESP_LOGW(TAG, "No notes found in MIDI file, adding a default C4 note");
        MidiNote defaultNote = {
            60,  // C4
            100, // Velocity
            0.0, // Start at beginning
            1.0  // Duration of 1 beat
        };
        track.notes.push_back(defaultNote);
        track.lengthInBeats = 1.0;
    }
    
    // Sort notes by start time
    std::sort(track.notes.begin(), track.notes.end(), [](const MidiNote& a, const MidiNote& b) {
        return a.startTime < b.startTime;
    });
    
    ESP_LOGI(TAG, "Loaded MIDI file with %zu notes, %zu CCs, length: %.2f beats", 
             track.notes.size(), track.ccs.size(), track.lengthInBeats);
    
    fclose(file);
    return true;
}

// MidiFilePlayer implementation
MidiFilePlayer::MidiFilePlayer() : 
    currentFileIndex(0),
    currentFile(nullptr),
    transpose(0),
    playbackRate(1.0),
    noteLengthScale(1.0),
    velocityScale(1.0),
    isPlaying(false),
    lastBeat(0),
    lastTempo(120.0),
    syncToBpm(false),
    quantizeLoops(true),
    linkQuantum(LINK_QUANTUM) {  // Initialize with LINK_QUANTUM from main.h
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
    
    // Clear existing files
    midiFiles.clear();
    
    // In SPIFFS, directories don't really exist, so we don't try to create them
    // Instead, we scan for files that have the prefix of our path
    
    // First try to open as a directory - this works in some implementations
    DIR* dir = opendir(folderPath.c_str());
    if (dir) {
        struct dirent* entry;
        int fileCount = 0;
        ESP_LOGI(TAG, "Scanning for MIDI files in: %s", folderPath.c_str());
        while ((entry = readdir(dir)) != NULL) {
            std::string filename = entry->d_name;
            
            // Check if it's a MIDI file
            if (filename.size() > 4 && 
                filename.substr(filename.size() - 4) == ".mid") {
                midiFiles.push_back(folderPath + "/" + filename);
                fileCount++;
                ESP_LOGI(TAG, "Found MIDI file: %s", filename.c_str());
            }
        }
        closedir(dir);
        ESP_LOGI(TAG, "Found %d MIDI files in %s", fileCount, folderPath.c_str());
    } else {
        // Directory open failed, so use alternate approach for SPIFFS
        // We'll scan the root and filter for files with our prefix
        
        // Extract folder name without /spiffs/ prefix for comparison
        std::string prefix = folderPath;
        if (prefix.compare(0, 8, "/spiffs/") == 0) {
            prefix = prefix.substr(8); // Remove "/spiffs/" prefix for comparison
        }
        
        // Make sure prefix ends with /
        if (prefix.back() != '/') {
            prefix += '/';
        }
        
        ESP_LOGI(TAG, "Opening root directory to search for prefix: %s", prefix.c_str());
        
        // Try with the root directory
        DIR* rootDir = opendir("/spiffs");
        if (rootDir) {
            struct dirent* entry;
            int fileCount = 0;
            
            ESP_LOGI(TAG, "Scanning SPIFFS root for files with prefix: %s", prefix.c_str());
            
            while ((entry = readdir(rootDir)) != NULL) {
                std::string entryPath = entry->d_name;
                
                ESP_LOGD(TAG, "Checking entry: %s", entryPath.c_str());
                
                // Check if entry starts with our prefix and ends with .mid
                if (entryPath.find(prefix) == 0 && 
                    entryPath.size() > 4 && 
                    entryPath.substr(entryPath.size() - 4) == ".mid") {
                    
                    std::string fullPath = "/spiffs/" + entryPath;
                    midiFiles.push_back(fullPath);
                    fileCount++;
                    
                    // Extract just the filename part for logging
                    size_t lastSlash = entryPath.find_last_of('/');
                    std::string filename = (lastSlash != std::string::npos) ? 
                                        entryPath.substr(lastSlash + 1) : entryPath;
                    
                    ESP_LOGI(TAG, "Found MIDI file: %s (full path: %s)", filename.c_str(), fullPath.c_str());
                }
            }
            closedir(rootDir);
            
            if (fileCount > 0) {
                ESP_LOGI(TAG, "Found %d MIDI files with prefix %s", fileCount, prefix.c_str());
            } else {
                ESP_LOGW(TAG, "No MIDI files found with prefix %s", prefix.c_str());
                
                // List all MIDI files to help debug the issue
                rootDir = opendir("/spiffs");
                if (rootDir) {
                    ESP_LOGI(TAG, "Listing all MIDI files in SPIFFS:");
                    struct dirent* entry;
                    while ((entry = readdir(rootDir)) != NULL) {
                        std::string path = entry->d_name;
                        if (path.size() > 4 && path.substr(path.size() - 4) == ".mid") {
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
    
    // If no MIDI files were found, try to create a simple one
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "No MIDI files found in %s, creating a default one", folderPath.c_str());
        
        // Create a default MIDI file in the folder
        std::string defaultFilePath = folderPath + "/default.mid";
        
        FILE* file = fopen(defaultFilePath.c_str(), "wb");
        if (file) {
            // MThd header
            fputs("MThd", file);
            fputc(0, file);            // Length high byte
            fputc(0, file);            // Length
            fputc(0, file);            // Length
            fputc(6, file);            // Length low byte
            
            fputc(0, file);            // Format (0)
            fputc(1, file);            // Format
            
            fputc(0, file);            // Tracks high byte
            fputc(1, file);            // Tracks low byte (1 track)
            
            fputc(0, file);            // Division high byte
            fputc(96, file);           // Division low byte (96 ticks per quarter note)
            
            // MTrk header
            fputs("MTrk", file);
            fputc(0, file);            // Length high byte
            fputc(0, file);            // Length
            fputc(0, file);            // Length
            fputc(19, file);           // Length low byte (bytes in track chunk)
            
            // Delta time
            fputc(0, file);            // Delta time
            
            // Note On
            fputc(0x90, file);         // Note On, channel 1
            fputc(60, file);           // Note number (middle C)
            fputc(64, file);           // Velocity
            
            // Delta time
            fputc(96, file);           // Delta time (96 ticks = 1 quarter note)
            
            // Note Off
            fputc(0x80, file);         // Note Off, channel 1
            fputc(60, file);           // Note number
            fputc(0, file);            // Velocity
            
            // End of track
            fputc(0, file);            // Delta time
            fputc(0xFF, file);         // Meta event
            fputc(0x2F, file);         // End of track
            fputc(0, file);            // Length
            
            fclose(file);
            
            // Add the created file to our list
            midiFiles.push_back(defaultFilePath);
            ESP_LOGI(TAG, "Created default MIDI file: %s", defaultFilePath.c_str());
        } else {
            ESP_LOGE(TAG, "Failed to create default MIDI file (errno: %d, %s)", 
                    errno, strerror(errno));
        }
    }
    
    // Sort filenames alphabetically
    std::sort(midiFiles.begin(), midiFiles.end());
    
    // Load the first file if available
    if (!midiFiles.empty()) {
        currentFileIndex = 0;
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
        ESP_LOGI(TAG, "Loading MIDI file: %s", midiFiles[currentFileIndex].c_str());
        if (!currentFile->load()) {
            ESP_LOGE(TAG, "Failed to load MIDI file: %s", midiFiles[currentFileIndex].c_str());
            currentFile.reset();  // Clear the invalid file
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
    
    // Calculate the next file index
    size_t nextIndex = (currentFileIndex + 1) % midiFiles.size();
    
    // Schedule the file switch for the next quantum boundary
    pendingFileSwitch = true;
    pendingFileIndex = nextIndex;
    
    ESP_LOGI(TAG, "Scheduled file switch from %zu to %zu at next quantum boundary", 
             currentFileIndex, nextIndex);
    
    // Log available files for debugging
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
    
    // Verify file exists before attempting to load
    struct stat st;
    if (stat(midiFiles[currentFileIndex].c_str(), &st) != 0) {
        ESP_LOGE(TAG, "File doesn't exist or can't be accessed: %s (errno: %d, %s)",
                midiFiles[currentFileIndex].c_str(), errno, strerror(errno));
        
        // Try reloading the file list as the filesystem might have changed
        setFolder(folderPath);
        
        // If we still have files after reloading, try the first one
        if (!midiFiles.empty()) {
            currentFileIndex = 0;
        } else {
            return; // No files available
        }
    }
    
    // Try to load the file
    try {
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
        if (!currentFile->load()) {
            ESP_LOGE(TAG, "Failed to load MIDI file: %s", midiFiles[currentFileIndex].c_str());
            currentFile.reset();  // Clear the invalid file
            
            // Try the next file if this one fails
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
    
    // Extract just the filename part for logging
    std::string fullPath = midiFiles[currentFileIndex];
    size_t lastSlash = fullPath.find_last_of('/');
    std::string filename = (lastSlash != std::string::npos) ? 
                           fullPath.substr(lastSlash + 1) : fullPath;
    
    ESP_LOGI(TAG, "Loaded MIDI file: %s at quantum boundary", filename.c_str());
    
    // Clear any played notes from the previous file
    clearPlayedNotes();
    activeNotes.clear();
    
    // The file is now loaded and will start playing seamlessly at the quantum boundary
}

bool MidiFilePlayer::start() {
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "No MIDI files available to play");
        return false;
    }
    
    // Attempt to load the current file
    bool loaded = false;
    int attempts = 0;
    const int MAX_ATTEMPTS = 3;
    
    // Try a few files in case one fails to load
    while (!loaded && attempts < MAX_ATTEMPTS && attempts < midiFiles.size()) {
        // Create a new MIDI file object
        currentFile = std::make_unique<MidiFile>(midiFiles[currentFileIndex]);
        
        // Try to load the file
        loaded = currentFile->load();
        if (!loaded) {
            ESP_LOGW(TAG, "Failed to load MIDI file %s, trying next file", 
                    midiFiles[currentFileIndex].c_str());
            
            // Move to next file
            currentFileIndex = (currentFileIndex + 1) % midiFiles.size();
            attempts++;
        } else {
            // Successfully loaded
            const MidiTrack& track = currentFile->getTrack();
            
            // If track length isn't a multiple of linkQuantum, log the info
            double lengthInQuantums = track.lengthInBeats / linkQuantum;
            double remainder = lengthInQuantums - std::floor(lengthInQuantums);
            
            if (remainder > 0.01 && remainder < 0.99) {
                // Track length doesn't align perfectly with quantum
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
    
    // Clear any previously played notes
    playedNotes.clear();
    activeNotes.clear();
    
    // Start playback
    isPlaying = true;
    lastBeat = 0;
    
    return true;
}

void MidiFilePlayer::stop() {
    isPlaying = false;
    
    // Turn off any playing notes
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
    // PERFORMANCE: Early exit if no active notes
    if (activeNotes.empty()) {
        return;
    }
    
    // Use absolute beat time for comparison
    double absoluteBeat = currentBeat;
    
    // Remove notes that need to be turned off
    auto it = activeNotes.begin();
    while (it != activeNotes.end()) {
        if (it->endTime <= absoluteBeat) {
            // Send note off
            if (g_current_synth) {
                g_current_synth->sendNoteOff(it->note, 0);
            }
            
            // Remove this note
            it = activeNotes.erase(it);
        } else {
            ++it;
        }
    }
}

void MidiFilePlayer::updateTempo(const ableton::Link::SessionState& sessionState) {
    if (!syncToBpm) return;
    
    // Get current tempo from Link
    double currentTempo = sessionState.tempo();
    
    // Only update if the tempo has changed significantly
    if (std::abs(currentTempo - lastTempo) > 0.5) {
        // We want to scale playback rate based on tempo
        // Assuming MIDI files are typically designed for 120 BPM
        constexpr double REFERENCE_TEMPO = 120.0;
        
        // Calculate adjusted playback rate (overrides manual setting)
        double tempoFactor = currentTempo / REFERENCE_TEMPO;
        playbackRate = tempoFactor;
        
        ESP_LOGI(TAG, "Adjusted playback rate to %.2fx (Link tempo: %.1f BPM)", 
                 playbackRate, currentTempo);
        
        lastTempo = currentTempo;
    }
}

double MidiFilePlayer::calculateQuantizedLoopPoint(double currentBeat, const MidiTrack& track) {
    if (!quantizeLoops) {
        return track.lengthInBeats;
    }
    
    // Find the best multiple of the quantum that matches the track length
    // This ensures longer phrases are properly synchronized
    
    // If track is shorter than the quantum, just use the quantum
    if (track.lengthInBeats <= linkQuantum) {
        return linkQuantum;
    }
    
    // Find the closest multiple of linkQuantum to the track length
    // For tracks close to a quantum multiple, we round to that multiple
    // For tracks not close, we find the best quantum multiple that preserves the phrase
    
    // Calculate how many quantums would fit in the track
    double numQuantums = track.lengthInBeats / linkQuantum;
    
    // Round to the nearest whole number of quantums
    double roundedQuantums = std::round(numQuantums);
    
    // If we're very close to a whole number of quantums, use that
    if (std::abs(numQuantums - roundedQuantums) < 0.1) {
        return roundedQuantums * linkQuantum;
    }
    
    // For tracks with lengths not close to quantum multiples,
    // round up to ensure we accommodate the entire phrase
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
    
    // Get current beat from Link - always use 16-beat quantum
    const auto beats = sessionState.beatAtTime(time, linkQuantum);
    
    // Get the current track
    const MidiTrack& track = currentFile->getTrack();
    
    // Update tempo based on Link session
    updateTempo(sessionState);
    
    // Process any notes that need to be turned off
    processNoteOffs(beats * playbackRate);
    
    // Loop at the track's quantized phrase length, NOT a forced 16-beat quantum.
    // calculateQuantizedLoopPoint rounds the track length to the nearest/ceil multiple
    // of the quantum so a longer phrase (e.g. a 32- or 64-beat loop) is preserved while
    // still aligning to Link's grid. This keeps the longer phrase intact.
    double effectiveTrackLength = calculateQuantizedLoopPoint(0.0, track);
    if (effectiveTrackLength <= 0.0) {
        effectiveTrackLength = linkQuantum; // degenerate guard: empty/zero-length track
    }
    
    // Calculate the beat position in the MIDI file with quantum-aligned looping
    const double scaledBeats = beats * playbackRate;
    double loopedBeat = fmod(scaledBeats, effectiveTrackLength);
    if (loopedBeat < 0) loopedBeat += effectiveTrackLength;
    
    // Check if we've crossed the loop boundary (quantum boundary)
    bool loopBoundaryCrossed = loopedBeat < lastBeat;
    if (loopBoundaryCrossed) {
        // Get the quantum information from Link
        double currentPhase = sessionState.phaseAtTime(time, linkQuantum);
        int currentQuantumNumber = static_cast<int>(std::floor(beats / linkQuantum));
        
        ESP_LOGD(TAG, "MIDI loop restart at quantum boundary %d (beat %.2f, phase: %.2f)", 
                 currentQuantumNumber, beats, currentPhase);
        
        if (g_current_synth) {
            g_current_synth->sendAllNotesOff();
        }
        activeNotes.clear();

        // Clear tracking for played notes
        clearPlayedNotes();
        
        // Check if file switching is pending and we're at a quantum boundary
        if (pendingFileSwitch && currentPhase < 0.1) {
            // Switch to the pending file at the quantum boundary
            loadFileInternal(pendingFileIndex);
            pendingFileSwitch = false;
            ESP_LOGI(TAG, "Switched to new file at quantum boundary");
        }
    }
    
    // Store the current beat for next time
    lastBeat = loopedBeat;
    
    // Debug: check how many notes are in the track
    if (track.notes.empty()) {
        static int emptyLogCounter = 0;
        if (++emptyLogCounter > 100) {  // Don't spam logs
            ESP_LOGW(TAG, "No notes in the MIDI track! Current file: %s", 
                     midiFiles.empty() ? "None" : midiFiles[currentFileIndex].c_str());
            emptyLogCounter = 0;
        }
        return;
    }
    
    // Process notes at the current beat
    // Removed notes per tick limit for unrestricted playback
    
    // Smaller window to catch the note, reduces but doesn't eliminate multiple triggers
    constexpr double NOTE_TRIGGER_WINDOW = 0.03;
    
    // PERFORMANCE: Use early termination to reduce loop iterations
    for (const auto& note : track.notes) {
        // Skip notes that are too far in the future
        if (note.startTime > loopedBeat + NOTE_TRIGGER_WINDOW) {
            break; // Notes are sorted by start time, so we can break early
        }
        
        // Calculate if the note should be played at this beat
        if (loopedBeat >= note.startTime && 
            loopedBeat < note.startTime + NOTE_TRIGGER_WINDOW) { 
            
            // Create a note event to check if we've already played this note
            NoteEvent event{note.note, note.startTime};
            
            // Check if we've already played this note
            if (playedNotes.find(event) != playedNotes.end()) {
                // Already played this note, skip it
                continue;
            }
            
            // Apply transpose and play the note
            int transposedNote = note.note + transpose;
            
            // Ensure the note is in the valid MIDI range
            if (transposedNote >= 0 && transposedNote <= 127) {
                // Play the note via the synth interface
                if (g_current_synth) {
                    // Apply velocity scaling
                    int scaledVelocity = static_cast<int>(note.velocity * velocityScale);
                    scaledVelocity = std::min(127, std::max(0, scaledVelocity));
                    
                    g_current_synth->sendNoteOn(transposedNote, scaledVelocity);
                    
                    // Record that we've played this note
                    playedNotes.insert(event);
                    
                    // Calculate the absolute beat where the note should end with length scaling
                    double scaledDuration = note.duration * noteLengthScale;
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
    
    // Process CC messages at the current beat
    for (const auto& cc : track.ccs) {
        // Check if the CC should be sent at this beat
        if (loopedBeat >= cc.time && 
            loopedBeat < cc.time + NOTE_TRIGGER_WINDOW) {
            
            // Create a CC event to check if we've already sent this CC
            CCEvent event{cc.controller, cc.time};
            
            // Check if we've already sent this CC
            if (sentCCs.find(event) != sentCCs.end()) {
                // Already sent this CC, skip it
                continue;
            }
            
            // Send the CC message via the synth interface
            if (g_current_synth) {
                g_current_synth->sendControlChange(cc.controller, cc.value);
                ESP_LOGD(TAG, "Sent CC: controller=%u value=%u at beat %.2f", 
                        cc.controller, cc.value, loopedBeat);
                
                // Record that we've sent this CC
                sentCCs.insert(event);
            }
        }
    }
}

std::string MidiFilePlayer::getCurrentFileName() const {
    if (midiFiles.empty()) return "";
    
    // Extract just the filename without the path
    std::string fullPath = midiFiles[currentFileIndex];
    size_t lastSlash = fullPath.find_last_of('/');
    if (lastSlash != std::string::npos) {
        return fullPath.substr(lastSlash + 1);
    }
    return fullPath;
}

void MidiFilePlayer::setCurrentFileIndex(size_t index) {
    if (midiFiles.empty()) {
        ESP_LOGW(TAG, "Cannot set file index - no files available");
        return;
    }
    
    // Clamp the index to valid range
    if (index >= midiFiles.size()) {
        index = midiFiles.size() - 1;
        ESP_LOGW(TAG, "Requested file index %zu out of range, clamping to %zu", index, index);
    }
    
    currentFileIndex = index;
    ESP_LOGI(TAG, "Set current file index to %zu", currentFileIndex);
} 