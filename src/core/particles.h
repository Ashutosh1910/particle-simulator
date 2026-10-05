#pragma once
#include <cstdint>
#include <vector>

// Struct-of-arrays particle storage: each attribute lives in its own
// contiguous array so hot loops (integration, broad phase) touch only the
// data they need and the compiler can vectorise them.
struct Particles {
    std::vector<float> x, y;      // position
    std::vector<float> vx, vy;    // velocity
    std::vector<float> px, py;    // position at the start of the substep (position-based solvers)
    std::vector<float> ax, ay;    // acceleration from the last force evaluation (velocity Verlet)
    std::vector<float> radius;
    std::vector<float> invMass;   // 0 = pinned / immovable
    std::vector<uint32_t> color;  // packed RGBA
    std::vector<int32_t> group;   // particles sharing a non-zero group never collide with each other

    int size() const { return (int)x.size(); }

    int add(float x0, float y0, float vx0, float vy0, float r, float mass, uint32_t rgba, int grp = 0) {
        x.push_back(x0); y.push_back(y0);
        vx.push_back(vx0); vy.push_back(vy0);
        px.push_back(x0); py.push_back(y0);
        ax.push_back(0); ay.push_back(0);
        radius.push_back(r);
        invMass.push_back(mass > 0 ? 1.0f / mass : 0.0f);
        color.push_back(rgba);
        group.push_back(grp);
        return size() - 1;
    }

    // Removes particle i by moving the last particle into its slot.
    void swapRemove(int i) {
        int last = size() - 1;
        auto mv = [&](auto& v) { v[i] = v[last]; v.pop_back(); };
        mv(x); mv(y); mv(vx); mv(vy); mv(px); mv(py); mv(ax); mv(ay);
        mv(radius); mv(invMass); mv(color); mv(group);
    }

    void clear() {
        for (auto* v : {&x, &y, &vx, &vy, &px, &py, &ax, &ay, &radius, &invMass}) v->clear();
        color.clear();
        group.clear();
    }

    float mass(int i) const { return invMass[i] > 0 ? 1.0f / invMass[i] : 0.0f; }
};

// Mass grows with area; a radius-10 particle has mass 1 (the original simulator's default).
inline float massForRadius(float r) { return r * r / 100.0f; }
