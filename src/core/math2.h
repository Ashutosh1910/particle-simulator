#pragma once
// Minimal 2D vector math used by the simulation core. The core does not depend
// on raylib so it can be built into headless tests and benchmarks.
#include <algorithm>
#include <cmath>

struct Vec2 {
    float x = 0, y = 0;
    constexpr Vec2() = default;
    constexpr Vec2(float x, float y) : x(x), y(y) {}
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(float s, Vec2 a) { return {a.x * s, a.y * s}; }
inline Vec2 operator/(Vec2 a, float s) { return {a.x / s, a.y / s}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { a.x += b.x; a.y += b.y; return a; }
inline Vec2& operator-=(Vec2& a, Vec2 b) { a.x -= b.x; a.y -= b.y; return a; }
inline Vec2& operator*=(Vec2& a, float s) { a.x *= s; a.y *= s; return a; }

inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
// z component of the 3D cross product
inline float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// angular velocity (scalar) cross vector: w x r
inline Vec2 cross(float w, Vec2 r) { return {-w * r.y, w * r.x}; }
inline float lengthSq(Vec2 a) { return dot(a, a); }
inline float length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline Vec2 normalize(Vec2 a) {
    float l = length(a);
    return l > 1e-12f ? a / l : Vec2{0, 0};
}
inline Vec2 rotate(Vec2 v, float c, float s) { return {c * v.x - s * v.y, s * v.x + c * v.y}; }

struct AABB {
    float minX = 0, minY = 0, maxX = 0, maxY = 0;
    float width() const { return maxX - minX; }
    float height() const { return maxY - minY; }
    bool contains(Vec2 p) const { return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY; }
};

inline bool overlaps(const AABB& a, const AABB& b) {
    return a.minX <= b.maxX && b.minX <= a.maxX && a.minY <= b.maxY && b.minY <= a.maxY;
}

// Packed RGBA colour, byte order r,g,b,a in memory (matches raylib's Color).
inline unsigned int packRGBA(unsigned r, unsigned g, unsigned b, unsigned a = 255) {
    return (r & 255u) | ((g & 255u) << 8) | ((b & 255u) << 16) | ((a & 255u) << 24);
}
