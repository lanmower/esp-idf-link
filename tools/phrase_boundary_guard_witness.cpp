#include "phrase_boundary_guard.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
int g_checks = 0;
int g_fail = 0;

void check(const char* label, bool condition)
{
    ++g_checks;
    if (!condition) {
        ++g_fail;
        std::printf("FAIL: %s\n", label);
    }
}

std::string readSource(const char* path)
{
    std::ifstream in(path);
    if (!in) {
        std::printf("FAIL: cannot read %s\n", path);
        ++g_fail;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool contains(const std::string& haystack, const char* needle)
{
    return haystack.find(needle) != std::string::npos;
}

PhraseBoundaryStep feed(bool& baseline, int& last, int current)
{
    const PhraseBoundaryStep step = stepPhraseBoundary(baseline, last, current);
    baseline = true;
    last = step.lastPhraseNumber;
    return step;
}

void stepChecks()
{
    bool baseline = false;
    int last = 0;

    const PhraseBoundaryStep first = feed(baseline, last, 7);
    check("baseline captures current phrase number", !first.crossed && last == 7);

    const PhraseBoundaryStep forward = feed(baseline, last, 8);
    check("forward step emits a boundary", forward.crossed && last == 8);

    const PhraseBoundaryStep forwardJump = feed(baseline, last, 11);
    check("forward jump emits one boundary", forwardJump.crossed && last == 11);

    const PhraseBoundaryStep noChange = feed(baseline, last, 11);
    check("no-change control emits no boundary", !noChange.crossed && last == 11);

    const PhraseBoundaryStep backwards = feed(baseline, last, 6);
    check("backwards rebase emits no boundary", !backwards.crossed);
    check("backwards rebase adopts the new lower phrase number", last == 6);

    const PhraseBoundaryStep recovered = feed(baseline, last, 7);
    check("forward boundary after a rebase is NOT held back", recovered.crossed && last == 7);

    const PhraseBoundaryStep recoveredFurther = feed(baseline, last, 8);
    check("later forward boundaries keep emitting", recoveredFurther.crossed && last == 8);

    bool deepBaseline = true;
    int deepLast = 12;
    const PhraseBoundaryStep deep = feed(deepBaseline, deepLast, -5);
    check("deep backwards rebase emits no boundary", !deep.crossed && deepLast == -5);
    const PhraseBoundaryStep deepRecovered = feed(deepBaseline, deepLast, -4);
    check("deep rebase recovery emits the next forward boundary",
          deepRecovered.crossed && deepLast == -4);
    check("a second backwards step still emits no boundary",
          !feed(deepBaseline, deepLast, -9).crossed);
    check("repeated stale phrase number stays quiet",
          !feed(deepBaseline, deepLast, -9).crossed);
}

void parityChecks()
{
    const std::string sync = readSource("main/link_sync.cpp");
    check("link_sync.cpp includes the shipped guard header",
          contains(sync, "#include \"phrase_boundary_guard.h\""));
    check("link_sync.cpp calls stepPhraseBoundary",
          contains(sync, "stepPhraseBoundary("));
    check("link_sync.cpp no longer uses the direction-blind comparison",
          !contains(sync, "info.currentPhraseNumber != s_last_phrase_number"));
    check("link_sync.cpp keeps the SPP call tied to the boundary flag",
          contains(sync, "if (crossedPhraseBoundary) {"));
    check("link_sync.cpp keeps send_song_position(quantumInfo.phraseBeat)",
          contains(sync, "send_song_position(quantumInfo.phraseBeat);"));
    check("shipped guard has a backwards-rebase branch",
          contains(readSource("main/phrase_boundary_guard.h"),
                   "if (currentPhraseNumber < lastPhraseNumber)"));
}
}

int main()
{
    stepChecks();
    parityChecks();
    std::printf("phrase boundary guard witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
