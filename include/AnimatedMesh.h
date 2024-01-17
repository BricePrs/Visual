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
    void Draw(const PerspectiveCamera& camera) override;

private:

    uint32_t mFrameCount;
    double mFrameTime;
    Joint* mRootJoint;


    Mesh<SimpleVertex> mSkeletonMesh;

    // For Skinning
    Mesh<SimpleColorVertex> mSkinMesh;
    Mesh<SimpleColorVertex> mSkinMeshTransformed;
    void ParseWeights(const std::string &weightsFileName, std::unordered_map<std::string, Joint *> &jointMap);
    std::vector<std::vector<std::pair<double, Joint *>>> weight; // Size = nb a vertex in mSkinMesh
    std::vector<Joint *> jointArray; // Need to find joints in array which exist in the bvh
    std::unordered_map<Joint *, glm::mat4> B_MJ;
    std::unordered_map<Joint *, glm::mat4> C_JM;
};


#endif //VISUAL_ANIMATEDMESH_H
