//
// Created by brice on 12/1/23.
//

#ifndef VISUAL_SPRINGGROUP_H
#define VISUAL_SPRINGGROUP_H

#include <cstdint>
#include <vector>
#include <cmath>

struct DampedSpringParams {
    float k;
    float a;
    float l0;
    float rDist;
    float maxT;
    float l0Mult = 1.;
};

struct DampedSpring {

    bool enabled = true;

    uint32_t i;
    uint32_t j;

    uint32_t n;

    DampedSpring(uint32_t i, uint32_t j, uint32_t n) : i(i), j(j), n(n) {}

    [[nodiscard]]
    static float ComputeForce(float l, float rel_speed, const DampedSpringParams& p) {
        float spring = -p.k*std::max(0.f, std::abs(l-p.l0*p.l0Mult)-p.rDist); // TODO ? Could be optimized if relaxation is not used
        if (l-p.l0 < 0) { spring *= -1; }
        float damping = p.a * rel_speed;
        return spring - damping;
    }
};


class SpringGroup {
public:

    SpringGroup(const std::vector<DampedSpring> &springs, const DampedSpringParams &params) : mSprings(springs), params(params) {}

    std::vector<DampedSpring> &GetSprings() { return mSprings; }


    DampedSpringParams params;

private:
    std::vector<DampedSpring> mSprings;

};


#endif //VISUAL_SPRINGGROUP_H
