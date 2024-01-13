//
// Created by brice on 12/5/23.
//

#ifndef VISUAL_ANIMATEDMESH_H
#define VISUAL_ANIMATEDMESH_H


#include "joint.h"

class AnimatedMesh : public Drawable {
public:

    AnimatedMesh(const std::string &skeletonFileName, const std::string &skinFileName, const std::string &weightsFileName);

    void Update(double dt);
    void Draw(const PerspectiveCamera &camera, Shader &shader) override;

private:

    uint32_t mFrameCount;
    double mFrameTime;
    Joint* mRootJoint;


    Mesh<SimpleVertex> mSkeletonMesh;
    Mesh<SimpleColorVertex> mSkinMesh;
};


#endif //VISUAL_ANIMATEDMESH_H
