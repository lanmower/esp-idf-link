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

void stepChecks()
{
    const PhraseBoundaryStep baseline = stepPhraseBoundary(false, 0, 7);
    check("baseline captures current phrase number", !baseline.crossed && baseline.lastPhraseNumber == 7);

    const PhraseBoundaryStep forward = stepPhraseBoundary(true, 7, 8);
    check("forward step emits a boundary", forward.crossed);
    check("forward step advances the last number", forward.lastPhraseNumber == 8);

    const PhraseBoundaryStep forwardJump = stepPhraseBoundary(true, 7, 11);
    check("forward jump emits one boundary", forwardJump.crossed && forwardJump.lastPhraseNumber == 11);

    const PhraseBoundaryStep noChange = stepPhraseBoundary(true, 7, 7);
    check("no-change control emits no boundary", !noChange.crossed);
    check("no-change control keeps the last number", noChange.lastPhraseNumber == 7);

    const PhraseBoundaryStep backwards = stepPhraseBoundary(true, 7, 6);
    check("backwards step emits no boundary", !backwards.crossed);
    check("backwards step does not overwrite the last number", backwards.lastPhraseNumber == 7);

    const PhraseBoundaryStep deepRebase = stepPhraseBoundary(true, 12, -5);
    check("backwards rebase emits no boundary", !deepRebase.crossed);
    check("backwards rebase keeps the old last number", deepRebase.lastPhraseNumber == 12);

    const PhraseBoundaryStep recovered = stepPhraseBoundary(true, 12, 13);
    check("forward step after an ignored rebase still emits a boundary",
          recovered.crossed && recovered.lastPhraseNumber == 13);

    const PhraseBoundaryStep staleThenForward = stepPhraseBoundary(true, 12, 12);
    check("repeated stale phrase number stays quiet", !staleThenForward.crossed && staleThenForward.lastPhraseNumber == 12);
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
}
}

int main()
{
    stepChecks();
    parityChecks();
    std::printf("phrase boundary guard witness: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
