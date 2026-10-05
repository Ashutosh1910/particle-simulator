#pragma once
#include <vector>

#include "../physics/world.h"

struct SceneInfo {
    const char* name;
    PhysicsModel model;
    const char* description;  // shown under the scene selector; '\n' separated lines
    int defaultCount;         // particle count; 0 = fixed layout (no count slider)
    int minCount, maxCount;
};

const std::vector<SceneInfo>& sceneList();

// Clears the world, resets the physics parameters to the scene's defaults
// (broad phase and thread count are left alone) and populates it.
void loadScene(World& world, int scene, int count, unsigned seed);

// Colour palette shared by scenes and spawn tools.
uint32_t paletteColor(int i);

// Builders shared by scenes and edit tools.
// Rope of particles from a to b (optionally pinned at a, with a heavier ball at b).
void buildRope(World& w, Vec2 a, Vec2 b, float radius, int group, bool pinFirst, float endRadius);
// Pressurised ring: particles on a circle, edge links and an area constraint.
void buildSoftBody(World& w, Vec2 center, float radius, uint32_t color, int group, Vec2 velocity);
