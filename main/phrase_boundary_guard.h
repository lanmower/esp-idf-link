#pragma once

struct PhraseBoundaryStep {
    bool crossed;
    int lastPhraseNumber;
};

inline PhraseBoundaryStep stepPhraseBoundary(bool baselineCaptured, int lastPhraseNumber,
                                            int currentPhraseNumber)
{
    if (!baselineCaptured) {
        return {false, currentPhraseNumber};
    }
    if (currentPhraseNumber > lastPhraseNumber) {
        return {true, currentPhraseNumber};
    }
    if (currentPhraseNumber < lastPhraseNumber) {
        return {false, currentPhraseNumber};
    }
    return {false, lastPhraseNumber};
}
