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
    bool enableBreak = false;
};

struct DampedSpring {

    bool enabled = true;

    uint32_t i;
    uint32_t j;

    uint32_t n;
    uint8_t i_sp; // Force spot
    uint8_t j_sp; // Force spot

    DampedSpring(uint32_t i, uint32_t j, uint32_t n, uint8_t i_sp, uint8_t j_sp) : i(i), j(j), n(n), i_sp(i_sp), j_sp(j_sp) {}

    [[nodiscard]]
    static float ComputeForce(float l, float rel_speed, const DampedSpringParams& p) {
        float spring = -p.k*std::max(0.f, std::abs(l-p.l0*p.l0Mult)-p.rDist); // TODO ? Could be optimized if relaxation is not used
        if (l-p.l0*p.l0Mult < 0) { spring *= -1; }
        float damping = p.a * rel_speed;
        return spring - damping;
    }

    [[nodiscard]]
    static float ComputePotentialEnergy(float l, const DampedSpringParams& p) {
        float d = std::max(0.f, std::abs(l-p.l0*p.l0Mult)-p.rDist);
        return d*d;
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
