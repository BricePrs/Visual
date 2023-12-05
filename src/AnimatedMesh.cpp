//
// Created by brice on 12/5/23.
//

#include "AnimatedMesh.h"

AnimatedMesh::AnimatedMesh(const std::string &skeletonFileName, const std::string &skinFileName, const std::string &weightsFileName)
    : Mesh<SimpleVertex>()
{
    Joint *rootJoint = Joint::createFromFile(skeletonFileName, frameCount, frameTime);
    std::vector<SimpleVertex> squeletonVertices;
    std::vector<uint32_t> squeletonIndices;
    rootJoint->buildSqueleton(squeletonVertices, squeletonIndices, glm::vec3(0.), glm::vec3(1., 0., 0.), glm::vec3(0., 1., 0.), glm::vec3(0., 0., 1.));

    Mesh<SimpleVertex> squeletonMesh = {squeletonVertices, squeletonIndices};
    squeletonMesh.SetPrimitiveMode(GL_LINES);
    squeletonMesh.SetScale(glm::vec3(0.01));
    squeletonMesh.SetPosition(glm::vec3(-10, -1., 10.));
}
