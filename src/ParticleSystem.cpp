//
// Created by brice on 11/18/23.
//

#include "ParticleSystem.h"
#include <random>

ParticleSystem::ParticleSystem(glm::vec3 position, glm::vec3 bounds, double radius, uint32_t n)
    : mPosition(position), mBounds(bounds), mBoundingBox(position, bounds, glm::vec3(1., 1., 0.8))
{
    for (uint32_t i = 0; i < n; ++i) {
        float rx = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float ry = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float rz = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        glm::vec3 spPosition = position + (glm::vec3(rx, ry, rz) * 2.f - 1.f) * glm::max(glm::vec3(0.), bounds-glm::vec3(radius));
        mSpheres.emplace_back(radius, 3, true);
        mSpheres[mSpheres.size()-1].SetPosition(spPosition);
    }
}

void ParticleSystem::Draw(const PerspectiveCamera &camera, Shader& shader) {
    mBoundingBox.Draw(camera, shader);
    for (auto &sphere: mSpheres) {
        sphere.Draw(camera, shader);
    }
}
