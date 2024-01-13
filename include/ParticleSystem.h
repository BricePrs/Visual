//
// Created by brice on 11/18/23.
//

#ifndef VISUAL_PARTICLESYSTEM_H
#define VISUAL_PARTICLESYSTEM_H

#include <vector>
#include <Mesh.h>

class ParticleSystem : public Drawable {
public:

    ParticleSystem(glm::vec3 position, glm::vec3 bounds, double radius, uint32_t n);

    void Draw(const PerspectiveCamera &camera, Shader &shader) override;

private:
    glm::vec3 mPosition;
    glm::vec3 mBounds;
    std::vector<Sphere> mSpheres;
    WireframeBox mBoundingBox;
};


#endif //VISUAL_PARTICLESYSTEM_H
