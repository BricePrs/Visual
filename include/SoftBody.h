//
// Created by brice on 11/28/23.
//

#ifndef VISUAL_SOFTBODY_H
#define VISUAL_SOFTBODY_H

#include <unordered_map>
#include <fstream>
#include "Mesh.h"
#include "ConstraintSet.h"
#include "SpringGroup.h"
#include "gnuplot-iostream.h"

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

struct CudaODESystem {
    glm::vec3*                      Positions;     // Per Mass
    glm::vec3*                      Velocities;    // Per Mass
    glm::vec3*                      Forces;        // Per Mass
    glm::vec3*                      TotalForces;   // Per Mass

    CudaODESystem() = default;

    CudaODESystem(uint32_t massCount, uint32_t forcesPerMass)
    {
        cudaMalloc(reinterpret_cast<void **>(&Forces), forcesPerMass * massCount * sizeof (glm::vec3)); // 1D: 6, 2D: 12, 3D: 8, Total 26 / 100 000 vertex / force 12 Bytes
        cudaMalloc(reinterpret_cast<void **>(&TotalForces), massCount * sizeof (glm::vec3));
        cudaMalloc(reinterpret_cast<void **>(&Velocities), massCount * sizeof (glm::vec3));
        cudaMalloc(reinterpret_cast<void **>(&Positions), massCount * sizeof (glm::vec3));
    }
};

struct CudaEnergyResources {
    uint32_t                kineticEnergySize;
    uint32_t                potentialEnergySize;
    std::vector<uint32_t>   springGroupsEnergySize;
    std::vector<float *>    springGroupsEnergy;
    float *                 potentialEnergy;
    float *                 kineticEnergy;
};

struct CudaResources {
    uint32_t                        MassCount;
    uint32_t                        ForcesPerMass;
    uint32_t                        SpringCount;

    CudaODESystem                   mainOdeSystem;

    glm::vec3*                      Colors;        // Per Mass

    std::vector<CudaSpringGroup>    SpringGroups;
    std::vector<CudaConstraintSet>  ConstraintSets; // TODO

    std::vector<CudaODESystem>      RKOdeSystems;
    CudaEnergyResources             EnergyResources;
};

struct SimulationParams;

class SoftBody : public InteractiveObject {
public:

    enum DisplayMode {
        Hull,
        Structure,
    };

    static SoftBody Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params, DisplayMode displayMode);
    static std::shared_ptr<SoftBody> Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode, std::shared_ptr<SimulationParams> parameters);
    static std::shared_ptr<SoftBody> Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode);
    static std::shared_ptr<SoftBody> Torus(uint32_t longRes, uint32_t shortRes, uint32_t depthRes, PhysicsParam physicsParams, SoftBody::DisplayMode displayMode, std::shared_ptr<SimulationParams> simulationParameters = nullptr);

    void PhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders);
    void RkPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders);
    void CudaPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders);
    void CudaRkPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders);
    void Draw(const PerspectiveCamera &camera, Shader &shader) override;

    void DrawWindow();
    void PlotEnergy();
    void TogglePause() { mIsSimulationPaused = !mIsSimulationPaused; };

    std::shared_ptr<SimulationParams>   GetParams();


    SoftBody(
            const std::vector<glm::vec3> &positions,
            const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
            const std::vector<ConstraintSet> &constraintSets,
            const std::vector<std::vector<uint32_t>> &hullIndices,
            PhysicsParam params,
            DisplayMode mode,
            std::shared_ptr<SimulationParams> parameters = nullptr
    );


    ~SoftBody();

private:


    void InitMesh();
    void BuildMesh();

    void InitCudaResources();
    void InitCudaEnergyResources();
    void StartSimulation();

    void UpdateIC();

    void SolveODE(const std::vector<std::shared_ptr<Collider>> &colliders, bool solveCollisions);
    void CudaSolveODE(CudaODESystem &odeSystem, const std::vector<std::shared_ptr<Collider>> &colliders, bool solveCollisions);

    void ResetForces();
    void ComputeForces();
    void ApplyConstraints();
    void MoveMasses();
    void BackupSystem();
    void ApplyBackup();
    void SolveCollisions(const std::vector<std::shared_ptr<Collider>> &colliders);
    void UpdateBoundingBox();
    bool IsSpeedExploding();

    // -- RK4 functions
    void ComputeRKCoefs(const std::vector<std::shared_ptr<Collider>> &colliders);
    void ResetRkCoefs();
    void OffsetODESystem(int i, float offsetCoef);


    // -- Cuda simulation -- //
    void CudaResetForces(CudaODESystem &odeSystem);
    void CudaComputeForces(CudaODESystem &odeSystem);
    void CudaApplyConstraints(CudaODESystem &odeSystem);
    void CudaMoveMasses(CudaODESystem &odeSystem);
    void CudaSolveCollisions(CudaODESystem &odeSystem, const std::vector<std::shared_ptr<Collider>> &colliders);
    void CudaBuildMesh();


    // -- Cuda RK4 simulation -- //

    void CudaComputeRKCoefs(const std::vector<std::shared_ptr<Collider>> &colliders);
    void CudaApplyRkCoefs();
    void CudaInitRkIter();
    void CudaOffsetODESystems();


    void InitMeshData(std::vector<uint32_t> &hullMeshIndices);
    void UpdateMeshVertices();
    void CudaUpdateMeshVertices();


    // -- IC Set up -- //

    bool                                mHasSimulationStarted    = false;
    bool                                mIsSimulationPaused      = false;

    void CudaFreeResources();


    // -- Simulation analysis -- //

    void InitEnergyPlot();
    void ComputeSystemEnergy();
    void CudaComputeSystemEnergy();



    std::shared_ptr<SimulationParams>   mSimulationParams;

    // -- Simulation -- //

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
    std::vector<glm::vec3>              mPositionsBackup;
    std::vector<glm::vec3>              mVelocitiesBackup;


    // -- Rendering -- //

    std::vector<SimpleColorVertex>      mStructureMeshVertices;
    std::vector<uint32_t>               mStructureMeshIndices;

    std::vector<std::unordered_map<uint32_t, uint32_t>> mHullVerticesLUT;
    std::vector<SimpleNormalVertex>     mHullMeshVertices;
    std::vector<std::vector<uint32_t>>  mHullIndices;

    Mesh<SimpleColorVertex>*            mStructureMesh;
    Mesh<SimpleNormalVertex>*           mHullMesh;
    WireframeBox*                       mBoundingBox;


    // -- CUDA -- //

    std::vector<uint8_t>                mVertexForcesSpots;
    CudaResources                       mCuda_Resources;
    bool                                mIsCudaInit = false;

    // -- Simulation analysis -- //

    std::vector<float>                  mPltTime;
    std::vector<float>                  mPltKineticEnergy;
    std::vector<float>                  mPltGravityPotentialEnergy;
    std::vector<std::vector<float>>     mPltSpringsPotentialEnergy;
    std::vector<float>                  mPltMechanicalEnergy;

};

struct SimulationParams {
    uint32_t                            subPhysicStep           = 1;
    bool                                stopSimIfExploding      = false;
    glm::vec3                           centerPosition          = glm::vec3(0., 10., 3.);
    float                               scale                   = 1.f;
    SoftBody::DisplayMode               displayMode             = SoftBody::Hull;
    float                               vertexMass              = 1.f;
    float                               boundarySpringK         = 50.f;
    PhysicsParam                        physicsParams;
    std::vector<uint8_t>                enableConstraintSet;
    std::vector<glm::vec3>              positionsICBackup;     // Per Mass
    std::vector<DampedSpringParams>     springParams;

    std::shared_ptr<WireframeBox>       boundaryBox;

    SimulationParams() = default;
};

#endif //VISUAL_SOFTBODY_H
