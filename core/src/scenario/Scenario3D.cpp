#include "fluids/scenario/Scenario3D.h"

namespace fluids {

Scenario3D makeDamBreak3DScenario() {
    // A column of water held in one corner of the domain, released under
    // gravity to collapse and spread across the floor in both x and z (the
    // 3D generalization of the 2D dam break's "spreads across the floor in
    // x" — here it spreads in the whole x-z plane).
    Scenario3D s;
    s.domainWidth = 1.2f;
    s.domainHeight = 1.0f;
    s.domainDepth = 0.6f;
    s.spacing = 0.06f;
    s.regionX = 0.06f;
    s.regionY = 0.06f;
    s.regionZ = 0.06f;
    s.regionWidth = 0.42f;
    s.regionHeight = 0.72f;
    s.regionDepth = 0.48f;
    s.velocityX = 0.0f;
    s.velocityY = 0.0f;
    s.velocityZ = 0.0f;
    s.hasRegion2 = false;
    return s;
}

Scenario3D makeBlockDrop3DScenario() {
    // A cubic block of water seeded mid-air with empty space below it, so it
    // free-falls and splashes on impact.
    Scenario3D s;
    s.domainWidth = 1.2f;
    s.domainHeight = 1.2f;
    s.domainDepth = 0.6f;
    s.spacing = 0.06f;
    s.regionX = 0.42f;
    s.regionY = 0.7f;
    s.regionZ = 0.18f;
    s.regionWidth = 0.36f;
    s.regionHeight = 0.36f;
    s.regionDepth = 0.24f;
    s.velocityX = 0.0f;
    s.velocityY = 0.0f;
    s.velocityZ = 0.0f;
    s.hasRegion2 = false;
    return s;
}

Scenario3D makeTwoColumnCollision3DScenario() {
    // Two blocks near opposite ends of the domain along x, launched toward
    // each other.
    Scenario3D s;
    s.domainWidth = 1.4f;
    s.domainHeight = 0.7f;
    s.domainDepth = 0.5f;
    s.spacing = 0.06f;

    s.regionX = 0.06f;
    s.regionY = 0.06f;
    s.regionZ = 0.06f;
    s.regionWidth = 0.3f;
    s.regionHeight = 0.36f;
    s.regionDepth = 0.38f;
    s.velocityX = 1.2f;
    s.velocityY = 0.0f;
    s.velocityZ = 0.0f;

    s.hasRegion2 = true;
    s.region2X = 1.04f;
    s.region2Y = 0.06f;
    s.region2Z = 0.06f;
    s.region2Width = 0.3f;
    s.region2Height = 0.36f;
    s.region2Depth = 0.38f;
    s.region2VelocityX = -1.2f;
    s.region2VelocityY = 0.0f;
    s.region2VelocityZ = 0.0f;

    return s;
}

} // namespace fluids
