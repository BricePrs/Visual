//
// Created by brice on 12/5/23.
//

#include "AnimatedMesh.h"

AnimatedMesh::AnimatedMesh(const std::string &skeletonFileName, const std::string &skinFileName, const std::string &weightsFileName)
{

    // -- Skeleton Set-up -- //

    mRootJoint = Joint::createFromFile(skeletonFileName, mFrameCount, mFrameTime);
    
    std::vector<SimpleVertex> skeletonVertices;
    std::vector<uint32_t> skeletonIndices;
    mRootJoint->buildSkeletonMatrices(skeletonVertices, skeletonIndices, glm::mat4(1.));

    mSkeletonMesh = Mesh<SimpleVertex>(skeletonVertices, skeletonIndices);
    mSkeletonMesh.SetPrimitiveMode(GL_LINES);
    mSkeletonMesh.SetScale(glm::vec3(0.01));
    mSkeletonMesh.SetPosition(glm::vec3(-10, -1., 10.));
    mSkeletonMesh.SetColor(glm::vec3(1., 0.3, 0.2));

    // -- Skin Set-up -- //

    mSkinMesh = ParseOFF(skinFileName);
    mSkinMesh.SetScale(glm::vec3(0.01));
    mSkinMesh.SetPosition({-3, 1., 0.});
    mSkinMesh.SetDrawMode(GL_LINE);

    // -- Weights Set-up -- //


}

void AnimatedMesh::Update(double dt) {

    // -- Skeleton Update -- //
    
    double animationTime = fmod(dt, mFrameTime*mFrameCount) / mFrameTime;
    uint32_t frameNumber = static_cast<uint32_t>(trunc(animationTime));
    double framePercent = animationTime-frameNumber;

    mRootJoint->animateLerp(frameNumber, framePercent);
    std::vector<SimpleVertex> skeletonVertices;
    std::vector<uint32_t> skeletonIndices; // Not used

    mRootJoint->buildSkeletonMatrices(skeletonVertices, skeletonIndices, glm::mat4(1.));
    mSkeletonMesh.ChangeVertices(skeletonVertices);

    // -- Skin Update -- //


}

void AnimatedMesh::Draw(const PerspectiveCamera &camera) {
    mSkeletonMesh.Draw(camera);
    mSkinMesh.Draw(camera);
}
