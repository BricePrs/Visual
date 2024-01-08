//
// Created by brice on 1/8/24.
//

#ifndef VISUAL_ANIMATEDJOINT_H
#define VISUAL_ANIMATEDJOINT_H


#include "Drawable.h"
#include "Mesh.h"

class AnimatedJoint : public Drawable {
public:

    AnimatedJoint(const std::string& fileName, glm::vec3 position);

    void Update(double dt);
    void Draw(const PerspectiveCamera &camera) override;

private:

    uint32_t mFrameCount;
    double mFrameTime;

    std::vector<glm::quat> mOrientations;

    Arrow3D mArrowX;
    Arrow3D mArrowY;
    Arrow3D mArrowZ;
};


#endif //VISUAL_ANIMATEDJOINT_H
