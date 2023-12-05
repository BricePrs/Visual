//
// Created by brice on 11/28/23.
//

#ifndef VISUAL_SOFTBODY2D_H
#define VISUAL_SOFTBODY2D_H

#include "Mesh.h"
#include "ConstraintSet.h"
#include "SpringGroup.h"




struct PhysicsParam {
    float g;
    float dt;
};

class SoftBody2D : public InteractiveObject {
public:

    enum DisplayMode {
        Hull,
        Structure,
    };

    static SoftBody2D Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params, DisplayMode displayMode);
    static SoftBody2D Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode);

    void PhysicsStep(const std::vector<Collider*> &colliders);
    void Draw(const PerspectiveCamera &camera) override;

    void DrawWindow();


private:

    SoftBody2D(
            const std::vector<glm::vec3> &positions,
            const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
            const std::vector<ConstraintSet> &constraintSets,
            const std::vector<uint32_t> &hullIndices,
            PhysicsParam params,
            DisplayMode displayMode
       );


    void InitMesh();
    void BuildMesh();

    void ResetForces();
    void ComputeForces();
    void ApplyConstraints();
    void MoveMasses();
    void SolveCollisions(const std::vector<Collider*> &colliders);

    void InitMeshData(std::vector<uint32_t> &hullMeshIndices);
    void UpdateMeshVertices();



    // -- Simulation -- //

    PhysicsParam                        mPhysicsParams;
    float                               mVertexMass;

    std::vector<glm::vec3>              mPositions;     // Per Mass
    std::vector<glm::vec3>              mVelocities;    // Per Mass
    std::vector<glm::vec3>              mColors;        // Per Mass

    std::vector<glm::vec3>              mForces;        // Per Mass
    std::vector<SpringGroup>            mSpringGroups;

    std::vector<ConstraintSet>          mConstraintSets;

    glm::vec3                           mAABBLower;
    glm::vec3                           mAABBUpper;


    // -- Rendering -- //

    std::vector<SimpleColorVertex>      mStructureMeshVertices;
    std::vector<uint32_t>               mStructureMeshIndices;

    std::vector<SimpleNormalVertex>     mHullMeshVertices;
    std::vector<uint32_t>               mHullIndices;

    Mesh<SimpleColorVertex>*            mStructureMesh;
    Mesh<SimpleNormalVertex>*           mHullMesh;
    WireframeBox*                       mBoundingBox;

    DisplayMode                         mDisplayMode;
};


#endif //VISUAL_SOFTBODY2D_H
