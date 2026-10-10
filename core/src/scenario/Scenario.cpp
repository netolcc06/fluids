#include "fluids/scenario/Scenario.h"

namespace fluids {

Scenario makeDamBreakScenario() {
    // A tall column of water held against the left wall, released under
    // gravity to collapse across the floor.
    Scenario s;
    s.domainWidth = 2.0f;
    s.domainHeight = 1.5f;
    s.spacing = 0.035f;
    s.regionX = 0.05f;
    s.regionY = 0.05f;
    s.regionWidth = 0.5f;
    s.regionHeight = 1.0f;
    s.velocityX = 0.0f;
    s.velocityY = 0.0f;
    s.hasRegion2 = false;
    return s;
}

Scenario makeBlockDropScenario() {
    // A square block of water seeded mid-air with empty space below it, so
    // it free-falls and splashes on impact.
    Scenario s;
    s.domainWidth = 2.0f;
    s.domainHeight = 2.0f;
    s.spacing = 0.035f;
    s.regionX = 0.75f;
    s.regionY = 1.2f;
    s.regionWidth = 0.5f;
    s.regionHeight = 0.5f;
    s.velocityX = 0.0f;
    s.velocityY = 0.0f;
    s.hasRegion2 = false;
    return s;
}

Scenario makeTwoColumnCollisionScenario() {
    // Two blocks near opposite walls, launched toward each other.
    Scenario s;
    s.domainWidth = 2.0f;
    s.domainHeight = 1.2f;
    s.spacing = 0.035f;

    s.regionX = 0.05f;
    s.regionY = 0.05f;
    s.regionWidth = 0.4f;
    s.regionHeight = 0.6f;
    s.velocityX = 1.5f;
    s.velocityY = 0.0f;

    s.hasRegion2 = true;
    s.region2X = 1.55f;
    s.region2Y = 0.05f;
    s.region2Width = 0.4f;
    s.region2Height = 0.6f;
    s.region2VelocityX = -1.5f;
    s.region2VelocityY = 0.0f;

    return s;
}

} // namespace fluids
