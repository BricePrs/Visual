//
// Created by brice on 11/28/23.
//

#ifndef VISUAL_SOFTBODY2D_H
#define VISUAL_SOFTBODY2D_H

#include <unordered_map>
#include "Mesh.h"
#include "ConstraintSet.h"
#include "SpringGroup.h"

#define ENABLE_CUDA


struct PhysicsParam {
    float g;
    float dt;
    float t;
};

struct CudaSpringGroup {
    DampedSpringParams params;
    DampedSpring* springs;
    uint32_t springCount;
    CudaSpringGroup(DampedSpringParams params, DampedSpring* springs, uint32_t springCount) : params(params), springs(springs), springCount(springCount) {}
};


class CudaConstraintSet {
public:

    CudaConstraintSet(const std::vector<uint32_t>& constrainedVertices) : size(constrainedVertices.size()) {
        cudaMalloc(reinterpret_cast<void**>(&vertices), constrainedVertices.size() * sizeof(uint32_t));
        cudaMemcpy(vertices, constrainedVertices.data(), constrainedVertices.size() * sizeof(uint32_t), cudaMemcpyHostToDevice);
    }

    float freq = 1.; // TODO move
    uint32_t* vertices;
    uint32_t size;

};

struct CudaResources {
    uint32_t                        MassCount;
    uint32_t                        SpringCount;
    uint32_t                        ForcesPerMass;
    glm::vec3*                      Positions;     // Per Mass
    glm::vec3*                      Velocities;    // Per Mass
    glm::vec3*                      Colors;        // Per Mass

    glm::vec3*                      Forces;        // Per Mass
    glm::vec3*                      TotalForces;   // Per Mass
    std::vector<CudaSpringGroup>    SpringGroups;

    std::vector<CudaConstraintSet>  ConstraintSets; // TODO
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
    void RkPhysicsStep(const std::vector<Collider*> &colliders);
    void CudaPhysicsStep(const std::vector<Collider*> &colliders);
    void Draw(const PerspectiveCamera &camera) override;

    void DrawWindow();


private:

    SoftBody2D(
            const std::vector<glm::vec3> &positions,
            const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
            const std::vector<ConstraintSet> &constraintSets,
            const std::vector<std::vector<uint32_t>> &hullIndices,
            PhysicsParam params,
            DisplayMode mode
       );

    void InitCudaResources();
    void InitMesh();
    void BuildMesh();
    void SolveODE(const std::vector<Collider*> &colliders);

    void ResetForces();
    void ComputeForces();
    void ApplyConstraints();
    void MoveMasses();
    void ComputeRKCoefs(const std::vector<Collider*> &colliders);
    void BackupSystem();
    void ApplyBackup();
    void SolveCollisions(const std::vector<Collider*> &colliders);
    void ResetRkCoefs();
    void OffsetODESystem(int i, float offsetCoef);

    void CudaResetForces();
    void CudaComputeForces();
    void CudaApplyConstraints();
    void CudaMoveMasses();
    void CudaSolveCollisions(const std::vector<Collider*> &colliders);
    void CudaBuildMesh();

    void InitMeshData(std::vector<uint32_t> &hullMeshIndices);
    void UpdateMeshVertices();
    void CudaUpdateMeshVertices();




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

    // -- RK4 Integration Scheme -- //

    std::vector<std::vector<glm::vec3>> mRK_kForces;
    std::vector<std::vector<glm::vec3>> mRK_kVelocities;
    std::vector<glm::vec3> mPositionsBackup;
    std::vector<glm::vec3> mVelocitiesBackup;


    // -- Rendering -- //

    std::vector<SimpleColorVertex>      mStructureMeshVertices;
    std::vector<uint32_t>               mStructureMeshIndices;

    std::vector<std::unordered_map<uint32_t, uint32_t>> mHullVerticesLUT;
    std::vector<SimpleNormalVertex>     mHullMeshVertices;
    std::vector<std::vector<uint32_t>>  mHullIndices;

    Mesh<SimpleColorVertex>*            mStructureMesh;
    Mesh<SimpleNormalVertex>*           mHullMesh;
    WireframeBox*                       mBoundingBox;

    DisplayMode                         mDisplayMode;


    // -- CUDA -- //

    std::vector<uint8_t>                mVertexForcesSpots;
    CudaResources                       mCuda_Resources;

};


#endif //VISUAL_SOFTBODY2D_H
