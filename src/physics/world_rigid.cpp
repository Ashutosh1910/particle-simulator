// Convex polygon bodies: integration, contact generation (SAT against other
// bodies, circles against bodies, vertices against the walls) and a
// sequential-impulse solver with Coulomb friction and restitution.
#include <chrono>
#include <cmath>

#include "world.h"

namespace {

enum class Other : unsigned char { Body, Particle, World };

struct RigidContact {
    int a;        // body index
    int b;        // body index, particle index, or -1 for the world
    Other type;
    Vec2 n;       // from a to b
    Vec2 point;
    float depth;
    Vec2 rA, rB;
    float massN, massT, bias, jn, jt;
};

}  // namespace

void World::stepBodies(float h) {
    if (bodies.empty()) return;
    if (params.model != PhysicsModel::Rigid && params.model != PhysicsModel::SPH) return;
    auto t0 = std::chrono::steady_clock::now();
    const Vec2 g = params.gravity;
    const float damp = 1.0f / (1.0f + params.drag * h);

    // grab spring: critically damped, mass-independent pull towards the cursor
    if (grab_.kind == GrabKind::Body && grab_.index < (int)bodies.size()) {
        Body& b = bodies[grab_.index];
        Vec2 r = rotate(grab_.local, std::cos(b.angle), std::sin(b.angle));
        Vec2 vp = b.vel + cross(b.angVel, r);
        const float k = 900.0f, c = 2.0f * std::sqrt(k);
        Vec2 impulse = ((grab_.target - (b.pos + r)) * k - vp * c) * (b.mass() * h);
        b.vel += impulse * b.invMass;
        b.angVel += b.invInertia * cross(r, impulse);
    }

    for (Body& b : bodies) {
        if (b.isStatic()) continue;
        Vec2 acc = g;
        if (mouse.active) {
            Vec2 d = mouse.pos - b.pos;
            float dist = length(d);
            if (dist < mouse.radius && dist > 1) acc += d * (mouse.strength * (1 - dist / mouse.radius) / dist);
        }
        b.vel = (b.vel + acc * h) * damp;
        b.angVel *= damp;
        b.pos += b.vel * h;
        b.angle += b.angVel * h;
        b.updateWorld();
    }

    // ---- contact generation
    std::vector<RigidContact> contacts;
    auto add = [&](int a, int b, Other type, Vec2 n, const ContactPoint& cp) {
        contacts.push_back({a, b, type, n, cp.point, cp.depth, {}, {}, 0, 0, 0, 0, 0});
    };
    int nb = (int)bodies.size();
    for (int i = 0; i < nb; i++) {
        for (int j = i + 1; j < nb; j++) {
            const Body &A = bodies[i], &B = bodies[j];
            if (A.isStatic() && B.isStatic()) continue;
            float rr = A.boundRadius + B.boundRadius;
            if (lengthSq(A.pos - B.pos) > rr * rr) continue;
            Vec2 n;
            ContactPoint cps[2];
            int count = collidePolygons(A, B, n, cps);
            for (int k = 0; k < count; k++) add(i, j, Other::Body, n, cps[k]);
        }
    }
    if (params.walls) {
        for (int i = 0; i < nb; i++) {
            const Body& A = bodies[i];
            if (A.isStatic()) continue;
            for (Vec2 v : A.world) {
                if (v.x < bounds.minX) add(i, -1, Other::World, {-1, 0}, {v, bounds.minX - v.x});
                if (v.x > bounds.maxX) add(i, -1, Other::World, {1, 0}, {v, v.x - bounds.maxX});
                if (v.y < bounds.minY) add(i, -1, Other::World, {0, -1}, {v, bounds.minY - v.y});
                if (v.y > bounds.maxY) add(i, -1, Other::World, {0, 1}, {v, v.y - bounds.maxY});
            }
        }
    }
    if (p.size() > 0 && broadphase_->count() == p.size()) {
        // the broad phase was built earlier in this substep; particles may have
        // moved since, so widen the query by the largest radius plus a margin
        float maxR = 0, maxV = 0;
        for (int i = 0; i < p.size(); i++) {
            maxR = std::max(maxR, p.radius[i]);
            maxV = std::max(maxV, std::fabs(p.vx[i]) + std::fabs(p.vy[i]));
        }
        float margin = maxR + maxV * h + 1.0f;
        std::vector<int> found;
        for (int i = 0; i < nb; i++) {
            const Body& A = bodies[i];
            AABB box = A.aabb();
            box = {box.minX - margin, box.minY - margin, box.maxX + margin, box.maxY + margin};
            found.clear();
            broadphase_->queryAABB(box, found);
            for (int j : found) {
                if (A.isStatic() && p.invMass[j] == 0) continue;
                Vec2 n;
                ContactPoint cp;
                if (collidePolygonCircle(A, {p.x[j], p.y[j]}, p.radius[j], n, cp)) add(i, j, Other::Particle, n, cp);
            }
        }
    }
    stats_.bodyContacts = (int)contacts.size();
    if (contacts.empty()) {
        stats_.rigidMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return;
    }

    // ---- helpers to treat the three kinds of "b" uniformly
    auto velAt = [&](const RigidContact& c, bool isA) -> Vec2 {
        if (isA) {
            const Body& A = bodies[c.a];
            return A.vel + cross(A.angVel, c.rA);
        }
        switch (c.type) {
            case Other::Body: return bodies[c.b].vel + cross(bodies[c.b].angVel, c.rB);
            case Other::Particle: return {p.vx[c.b], p.vy[c.b]};
            default: return {0, 0};
        }
    };
    auto applyImpulse = [&](const RigidContact& c, Vec2 P) {  // +P on b, -P on a
        Body& A = bodies[c.a];
        A.vel -= P * A.invMass;
        A.angVel -= A.invInertia * cross(c.rA, P);
        if (c.type == Other::Body) {
            Body& B = bodies[c.b];
            B.vel += P * B.invMass;
            B.angVel += B.invInertia * cross(c.rB, P);
        } else if (c.type == Other::Particle) {
            p.vx[c.b] += P.x * p.invMass[c.b];
            p.vy[c.b] += P.y * p.invMass[c.b];
        }
    };
    auto invMassB = [&](const RigidContact& c) {
        if (c.type == Other::Body) return bodies[c.b].invMass;
        if (c.type == Other::Particle) return p.invMass[c.b];
        return 0.0f;
    };
    auto invInertiaB = [&](const RigidContact& c) { return c.type == Other::Body ? bodies[c.b].invInertia : 0.0f; };

    const float e = params.restitution, mu = params.friction;
    const float restSpeed = 2.0f * length(g) * h + 1e-3f;
    for (RigidContact& c : contacts) {
        const Body& A = bodies[c.a];
        c.rA = c.point - A.pos;
        c.rB = c.type == Other::Body ? c.point - bodies[c.b].pos : Vec2{0, 0};
        Vec2 t{c.n.y, -c.n.x};
        float wA = A.invMass, iA = A.invInertia, wB = invMassB(c), iB = invInertiaB(c);
        float rnA = cross(c.rA, c.n), rnB = cross(c.rB, c.n);
        float rtA = cross(c.rA, t), rtB = cross(c.rB, t);
        float kN = wA + wB + iA * rnA * rnA + iB * rnB * rnB;
        float kT = wA + wB + iA * rtA * rtA + iB * rtB * rtB;
        c.massN = kN > 0 ? 1.0f / kN : 0.0f;
        c.massT = kT > 0 ? 1.0f / kT : 0.0f;
        float vn = dot(velAt(c, false) - velAt(c, true), c.n);
        c.bias = vn < -restSpeed ? -e * vn : 0.0f;
    }

    for (int it = 0; it < params.rigidIterations; it++) {
        for (RigidContact& c : contacts) {
            if (c.massN == 0) continue;
            Vec2 vrel = velAt(c, false) - velAt(c, true);
            float vn = dot(vrel, c.n);
            float jnNew = std::max(c.jn + c.massN * (-vn + c.bias), 0.0f);
            float dj = jnNew - c.jn;
            c.jn = jnNew;
            applyImpulse(c, c.n * dj);

            Vec2 t{c.n.y, -c.n.x};
            vrel = velAt(c, false) - velAt(c, true);
            float vt = dot(vrel, t);
            float maxF = mu * c.jn;
            float jtNew = std::clamp(c.jt - c.massT * vt, -maxF, maxF);
            float djt = jtNew - c.jt;
            c.jt = jtNew;
            applyImpulse(c, t * djt);
        }
    }

    // ---- position correction: push overlapping shapes apart (split by mass)
    const float slop = 0.3f, beta = 0.5f;
    for (const RigidContact& c : contacts) {
        float C = std::max(c.depth - slop, 0.0f) * beta;
        if (C == 0 || c.massN == 0) continue;
        Vec2 P = c.n * (C * c.massN);
        Body& A = bodies[c.a];
        A.pos -= P * A.invMass;
        A.angle -= A.invInertia * cross(c.rA, P);
        if (c.type == Other::Body) {
            Body& B = bodies[c.b];
            B.pos += P * B.invMass;
            B.angle += B.invInertia * cross(c.rB, P);
        } else if (c.type == Other::Particle) {
            p.x[c.b] += P.x * p.invMass[c.b];
            p.y[c.b] += P.y * p.invMass[c.b];
        }
    }
    for (Body& b : bodies)
        if (!b.isStatic()) b.updateWorld();
    stats_.rigidMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
