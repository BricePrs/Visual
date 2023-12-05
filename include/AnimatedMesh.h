//
// Created by brice on 12/5/23.
//

#ifndef VISUAL_ANIMATEDMESH_H
#define VISUAL_ANIMATEDMESH_H


#include "joint.h"

class AnimatedMesh : public Mesh<SimpleVertex> {
public:

    AnimatedMesh(Joint &joint, std::string fileName);

private:

    uint32_t mFrameCount;
    Joint* mRootJoint;

};


#endif //VISUAL_ANIMATEDMESH_H
