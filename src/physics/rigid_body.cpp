#include "rigid_body.h"

#include <cfloat>
#include <cmath>

void Body::updateWorld() {
    float c = std::cos(angle), s = std::sin(angle);
    size_t n = local.size();
    world.resize(n);
    normals.resize(n);
    for (size_t i = 0; i < n; i++) world[i] = pos + rotate(local[i], c, s);
    for (size_t i = 0; i < n; i++) {
        Vec2 e = world[(i + 1) % n] - world[i];
        normals[i] = normalize(Vec2{e.y, -e.x});
    }
}

AABB Body::aabb() const {
    AABB b{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (Vec2 v : world) {
        b.minX = std::min(b.minX, v.x); b.maxX = std::max(b.maxX, v.x);
        b.minY = std::min(b.minY, v.y); b.maxY = std::max(b.maxY, v.y);
    }
    return b;
}

Body makePolygonBody(const std::vector<Vec2>& worldVerts, float density, uint32_t color) {
    Body body;
    body.color = color;
    std::vector<Vec2> v = worldVerts;
    size_t n = v.size();
    // signed area decides the winding; we store counter-clockwise (positive area),
    // for which (e.y, -e.x) is the outward normal of edge e
    float signedArea = 0;
    for (size_t i = 0; i < n; i++) signedArea += cross(v[i], v[(i + 1) % n]);
    if (signedArea < 0) std::reverse(v.begin(), v.end());

    // area, centroid and inertia by triangle fan around the vertex average
    Vec2 ref{0, 0};
    for (Vec2 p : v) ref += p;
    ref = ref / (float)n;
    float area = 0, inertia = 0;
    Vec2 centroid{0, 0};
    for (size_t i = 0; i < n; i++) {
        Vec2 e1 = v[i] - ref, e2 = v[(i + 1) % n] - ref;
        float d = cross(e1, e2);
        float triArea = 0.5f * d;
        area += triArea;
        centroid += (e1 + e2) * (triArea / 3.0f);
        float intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
        float inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
        inertia += (0.25f / 3.0f * d) * (intx2 + inty2);
    }
    area = std::max(area, 1e-6f);
    centroid = centroid / area;
    body.pos = ref + centroid;
    for (Vec2 p : v) body.local.push_back(p - body.pos);
    for (Vec2 p : body.local) body.boundRadius = std::max(body.boundRadius, length(p));
    if (density > 0) {
        float mass = density * area;
        // parallel axis theorem: inertia about the centroid
        float iCentroid = density * inertia - mass * dot(centroid, centroid);
        body.invMass = 1.0f / mass;
        body.invInertia = iCentroid > 0 ? 1.0f / iCentroid : 0.0f;
    }
    body.updateWorld();
    return body;
}

Body makeBoxBody(Vec2 center, float width, float height, float angle, float density, uint32_t color) {
    float c = std::cos(angle), s = std::sin(angle);
    float hw = width / 2, hh = height / 2;
    std::vector<Vec2> v = {center + rotate({-hw, -hh}, c, s), center + rotate({hw, -hh}, c, s),
                           center + rotate({hw, hh}, c, s), center + rotate({-hw, hh}, c, s)};
    Body b = makePolygonBody(v, density, color);
    return b;
}

Body makeRegularPolygonBody(Vec2 center, float radius, int sides, float angle, float density, uint32_t color) {
    std::vector<Vec2> v;
    for (int i = 0; i < sides; i++) {
        float a = angle + 2.0f * 3.14159265f * i / sides;
        v.push_back(center + Vec2{std::cos(a), std::sin(a)} * radius);
    }
    return makePolygonBody(v, density, color);
}

Body makeWallBody(Vec2 a, Vec2 b, float thickness, uint32_t color) {
    Vec2 d = b - a;
    float len = std::max(length(d), 1.0f);
    float angle = std::atan2(d.y, d.x);
    return makeBoxBody((a + b) * 0.5f, len, thickness, angle, 0.0f, color);
}

namespace {

// Largest separation of b from any face of a (negative = overlapping on all faces).
float maxSeparation(const Body& a, const Body& b, int& edge) {
    float best = -FLT_MAX;
    edge = 0;
    for (size_t i = 0; i < a.world.size(); i++) {
        Vec2 n = a.normals[i], v = a.world[i];
        float s = FLT_MAX;
        for (Vec2 w : b.world) s = std::min(s, dot(n, w - v));
        if (s > best) {
            best = s;
            edge = (int)i;
        }
    }
    return best;
}

// Keeps the part of segment in[0..1] where dot(n, p) <= offset.
int clipSegment(const Vec2 in[2], Vec2 out[2], Vec2 n, float offset) {
    int count = 0;
    float d0 = dot(n, in[0]) - offset, d1 = dot(n, in[1]) - offset;
    if (d0 <= 0) out[count++] = in[0];
    if (d1 <= 0) out[count++] = in[1];
    if (d0 * d1 < 0 && count < 2) out[count++] = in[0] + (in[1] - in[0]) * (d0 / (d0 - d1));
    return count;
}

}  // namespace

int collidePolygons(const Body& a, const Body& b, Vec2& normal, ContactPoint out[2]) {
    int edgeA, edgeB;
    float sepA = maxSeparation(a, b, edgeA);
    if (sepA > 0) return 0;
    float sepB = maxSeparation(b, a, edgeB);
    if (sepB > 0) return 0;

    // prefer a's face unless b's is clearly better (avoids flip-flopping)
    const Body *ref = &a, *inc = &b;
    int refEdge = edgeA;
    bool flip = false;
    if (sepB > sepA + 0.05f) {
        ref = &b; inc = &a; refEdge = edgeB; flip = true;
    }
    Vec2 refNormal = ref->normals[refEdge];

    // incident edge: the face of inc most anti-parallel to the reference normal
    int incEdge = 0;
    float minDot = FLT_MAX;
    for (size_t i = 0; i < inc->normals.size(); i++) {
        float d = dot(inc->normals[i], refNormal);
        if (d < minDot) {
            minDot = d;
            incEdge = (int)i;
        }
    }
    size_t ni = inc->world.size(), nr = ref->world.size();
    Vec2 incident[2] = {inc->world[incEdge], inc->world[(incEdge + 1) % ni]};
    Vec2 r1 = ref->world[refEdge], r2 = ref->world[(refEdge + 1) % nr];
    Vec2 tangent = normalize(r2 - r1);

    Vec2 clip1[2], clip2[2];
    if (clipSegment(incident, clip1, -tangent, -dot(tangent, r1)) < 2) return 0;
    if (clipSegment(clip1, clip2, tangent, dot(tangent, r2)) < 2) return 0;

    int count = 0;
    for (int i = 0; i < 2; i++) {
        float sep = dot(refNormal, clip2[i] - r1);
        if (sep <= 0) {
            // contact point halfway between the two surfaces
            out[count++] = {clip2[i] - refNormal * (0.5f * sep), -sep};
        }
    }
    normal = flip ? -refNormal : refNormal;
    return count;
}

bool collidePolygonCircle(const Body& a, Vec2 c, float r, Vec2& normal, ContactPoint& out) {
    size_t n = a.world.size();
    float best = -FLT_MAX;
    int edge = 0;
    for (size_t i = 0; i < n; i++) {
        float s = dot(a.normals[i], c - a.world[i]);
        if (s > r) return false;
        if (s > best) {
            best = s;
            edge = (int)i;
        }
    }
    Vec2 v1 = a.world[edge], v2 = a.world[(edge + 1) % n];
    if (best < 1e-6f) {  // centre inside the polygon: push out through the closest face
        normal = a.normals[edge];
        out = {c - normal * best, r - best};
        return true;
    }
    float u1 = dot(c - v1, v2 - v1), u2 = dot(c - v2, v1 - v2);
    Vec2 corner;
    if (u1 <= 0) corner = v1;
    else if (u2 <= 0) corner = v2;
    else {
        normal = a.normals[edge];
        out = {c - normal * best, r - best};
        return true;
    }
    Vec2 d = c - corner;
    float dist2 = lengthSq(d);
    if (dist2 > r * r) return false;
    float dist = std::sqrt(dist2);
    normal = dist > 1e-6f ? d / dist : a.normals[edge];
    out = {corner, r - dist};
    return true;
}

bool pointInBody(const Body& b, Vec2 p) {
    for (size_t i = 0; i < b.world.size(); i++)
        if (dot(b.normals[i], p - b.world[i]) > 0) return false;
    return !b.world.empty();
}
