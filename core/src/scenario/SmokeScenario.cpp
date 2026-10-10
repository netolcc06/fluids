#include "fluids/scenario/SmokeScenario.h"

namespace fluids {

SmokeScenario makePlumeScenario() {
    // A tall-ish domain (2m tall, 1.2m wide) with a modest source rectangle
    // centered near the bottom — a chimney/plume-base setup that gives the
    // rising column room to develop, curl, and dissipate before reaching
    // the top boundary. Comparable scale to the water presets' ~2x1.5m /
    // ~2x2m domains (see fluids::makeDamBreakScenario /
    // makeBlockDropScenario), just narrower and taller since a plume
    // benefits from vertical room more than horizontal.
    SmokeScenario s;
    s.domainWidth = 1.2f;
    s.domainHeight = 2.0f;
    s.sourceWidth = 0.3f;
    s.sourceHeight = 0.15f;
    s.sourceX = 0.5f * (s.domainWidth - s.sourceWidth); // centered
    s.sourceY = 0.0f;                                    // sits on the floor, like a chimney base
    s.emissionRate = 2.0f;
    return s;
}

} // namespace fluids
