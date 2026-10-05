#pragma once
#include <cstdint>
#include <vector>

#include "../core/math2.h"

// Convex polygon rigid body. Static bodies (invMass == 0) are used for
// obstacles and user-drawn walls.
struct Body {
    Vec2 pos;            // centre of mass
    Vec2 vel;
    float angle = 0;
    float angVel = 0;
    float invMass = 0;
    float invInertia = 0;
    std::vector<Vec2> local;    // vertices relative to the centre of mass, counter-clockwise (positive area)
    std::vector<Vec2> world;    // cached world-space vertices
    std::vector<Vec2> normals;  // cached outward edge normals; normals[i] belongs to edge world[i] -> world[i+1]
    float boundRadius = 0;
    uint32_t color = 0;

    bool isStatic() const { return invMass == 0; }
    float mass() const { return invMass > 0 ? 1.0f / invMass : 0.0f; }
    void updateWorld();
    AABB aabb() const;
};

// Builds a body from a convex polygon given in world coordinates (any winding).
// density is mass per square pixel; density <= 0 makes a static body.
Body makePolygonBody(const std::vector<Vec2>& worldVerts, float density, uint32_t color);
Body makeBoxBody(Vec2 center, float width, float height, float angle, float density, uint32_t color);
Body makeRegularPolygonBody(Vec2 center, float radius, int sides, float angle, float density, uint32_t color);
// A static wall segment from a to b with the given thickness.
Body makeWallBody(Vec2 a, Vec2 b, float thickness, uint32_t color);

// Mass per square pixel giving the same mass/area ratio as particles (radius 10 -> mass 1).
constexpr float kDefaultDensity = 1.0f / (3.14159265f * 100.0f);

struct ContactPoint {
    Vec2 point;   // world position
    float depth;  // penetration (> 0)
};

// SAT test with reference-face clipping. On overlap returns 1 or 2 contact
// points and the normal pointing from a to b; returns 0 when separated.
int collidePolygons(const Body& a, const Body& b, Vec2& normal, ContactPoint out[2]);

// Circle (centre c, radius r) against polygon a. Normal points from a to the circle.
bool collidePolygonCircle(const Body& a, Vec2 c, float r, Vec2& normal, ContactPoint& out);

// True if the world point p lies inside (or on) the polygon.
bool pointInBody(const Body& b, Vec2 p);
