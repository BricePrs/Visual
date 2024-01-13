//
// Created by brice on 11/28/23.
//

#include <chrono>
#include <softbody2D.cuh>
#include <unordered_map>
#include "SoftBody.h"
#include "imgui_dock/imgui.h"
#include "implot/implot.h"
#include <glm/gtc/type_ptr.hpp>

SoftBody::SoftBody(
        const std::vector<glm::vec3> &positions,
        const std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> &springGroupsIndices,
        const std::vector<ConstraintSet> &constraintSets,
        const std::vector<std::vector<uint32_t>> &hullIndices,
        PhysicsParam params,
        DisplayMode mode,
        std::shared_ptr<SimulationParams> parameters
) : InteractiveObject(false),
        mPositions(positions), mConstraintSets(constraintSets), mStructureMesh(nullptr),
        mHullIndices(hullIndices), mVertexForcesSpots(mPositions.size()), mHullVerticesLUT(hullIndices.size()),
        mSimulationParams(parameters)
{

    std::vector<DampedSpringParams> springParams;
    uint32_t meshVertexIndex = 0;
    for (int i = 0; i < springGroupsIndices.size(); ++i) {
        std::vector<DampedSpring> springs; springs.reserve(std::get<0>(springGroupsIndices[i]).size()/2);
        for (auto index = std::get<0>(springGroupsIndices[i]).begin(); index < std::get<0>(springGroupsIndices[i]).end(); index+=2) {
            springs.emplace_back(*index, *(index+1), meshVertexIndex, mVertexForcesSpots[*index], mVertexForcesSpots[*(index+1)]);
            ++mVertexForcesSpots[*index];
            ++mVertexForcesSpots[*(index+1)];
            meshVertexIndex+=2;
        }
        if (parameters != nullptr) {
            mSpringGroups.emplace_back(springs, parameters->springParams[i]);
        } else {
            mSpringGroups.emplace_back(springs, std::get<1>(springGroupsIndices[i]));
        }
    }



    mColors.reserve(mPositions.size());
    mForces.reserve(mPositions.size());
    mVelocities.reserve(mPositions.size());
    for ([[maybe_unused]] auto &vertex: mPositions) {
        mColors.emplace_back(1);
        mForces.emplace_back(0);
        mVelocities.emplace_back(0.);
    }

    InitMesh();

    if (parameters == nullptr) {
        mSimulationParams = std::make_shared<SimulationParams>();
        mSimulationParams->displayMode = mode;
        mSimulationParams->physicsParams = params;
        mSimulationParams->enableConstraintSet = std::vector<uint8_t>(mConstraintSets.size(), false);
        mSimulationParams->positionsICBackup = mPositions;
    }

    UpdateIC();
    UpdateMeshVertices();

    for (int i = 0; i < 4; ++i) {
        mRK_kForces.emplace_back();
        mRK_kForces[i].reserve(mPositions.size());
        mRK_kVelocities.emplace_back();
        mRK_kVelocities[i].reserve(mPositions.size());
    }


    InitEnergyPlot();
}

void SoftBody::UpdateIC() {
    for (uint32_t i = 0; i < mSimulationParams->positionsICBackup.size(); ++i) {
        mPositions[i] = mSimulationParams->scale * mSimulationParams->positionsICBackup[i] + mSimulationParams->centerPosition;
    }
    for (auto &springGroup: mSpringGroups) {
        springGroup.params.l0Mult = mSimulationParams->scale;
    }
    BuildMesh();
}

void SoftBody::StartSimulation() {
    for (auto &constraintSet: mConstraintSets) {
        constraintSet.SetStartingPositions(mPositions);
    }
#ifdef ENABLE_CUDA
    InitCudaResources();
#endif
}

void SoftBody::InitCudaResources() {
    mIsCudaInit = true;

    mCuda_Resources.MassCount = mPositions.size();
    for (auto& springGroup: mSpringGroups) {
        mCuda_Resources.SpringCount += springGroup.GetSprings().size();
        mCuda_Resources.SpringGroups.emplace_back(springGroup.params, nullptr, 0);
        mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springCount = springGroup.GetSprings().size();
        cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springs), springGroup.GetSprings().size() * sizeof (DampedSpring));
        cudaMemcpy(mCuda_Resources.SpringGroups[mCuda_Resources.SpringGroups.size() - 1].springs, springGroup.GetSprings().data(), springGroup.GetSprings().size() * sizeof (DampedSpring), cudaMemcpyHostToDevice);
    }
    mCuda_Resources.ForcesPerMass = 26;
    cudaMalloc(reinterpret_cast<void **>(&mCuda_Resources.Colors), mCuda_Resources.MassCount * sizeof (glm::vec3));

    mCuda_Resources.mainOdeSystem = CudaODESystem(mCuda_Resources.MassCount, mCuda_Resources.ForcesPerMass);


    cudaMemcpy(mCuda_Resources.mainOdeSystem.Positions, mPositions.data(), mCuda_Resources.MassCount * sizeof (glm::vec3), cudaMemcpyHostToDevice);
    cudaMemcpy(mCuda_Resources.Colors, mColors.data(), mCuda_Resources.MassCount * sizeof (glm::vec3), cudaMemcpyHostToDevice);

    for (auto &constraintSet: mConstraintSets) {
        mCuda_Resources.ConstraintSets.emplace_back(constraintSet.GetConstraints());
    }

    for (int i = 0; i < 4; ++i) {
        mCuda_Resources.RKOdeSystems.emplace_back(mCuda_Resources.MassCount, mCuda_Resources.ForcesPerMass);
    }

    InitCudaEnergyResources();
}

void SoftBody::CudaFreeResources() {
    if (!mIsCudaInit) { return; }
    for (auto& springGroup: mCuda_Resources.SpringGroups) {
        cudaFree(springGroup.springs);
    }
    cudaFree(mCuda_Resources.Colors);

    cudaFree(mCuda_Resources.EnergyResources.kineticEnergy);
    cudaFree(mCuda_Resources.EnergyResources.potentialEnergy);
    for (int i = 0; i < mSpringGroups.size(); ++i) {
        cudaFree(mCuda_Resources.EnergyResources.springGroupsEnergy[i]);
    }
}

SoftBody::~SoftBody() {
    CudaFreeResources();
}



uint32_t nextPowerOfTwo(uint32_t n) {
    return 1 << static_cast<uint32_t>(std::ceil(std::log2(n)));
}

void SoftBody::InitCudaEnergyResources() {
    mCuda_Resources.EnergyResources.kineticEnergySize = nextPowerOfTwo(mCuda_Resources.MassCount);
    mCuda_Resources.EnergyResources.potentialEnergySize = nextPowerOfTwo(mCuda_Resources.MassCount);
    cudaMalloc(reinterpret_cast<void**>(&mCuda_Resources.EnergyResources.kineticEnergy), mCuda_Resources.EnergyResources.kineticEnergySize * sizeof (float));
    cudaMalloc(reinterpret_cast<void**>(&mCuda_Resources.EnergyResources.potentialEnergy), mCuda_Resources.EnergyResources.potentialEnergySize * sizeof (float));
    for (int i = 0; i < mSpringGroups.size(); ++i) {
        mCuda_Resources.EnergyResources.springGroupsEnergySize.push_back(nextPowerOfTwo(mSpringGroups[i].GetSprings().size()));
        mCuda_Resources.EnergyResources.springGroupsEnergy.emplace_back();
        cudaMalloc(reinterpret_cast<void**>(&mCuda_Resources.EnergyResources.springGroupsEnergy[i]), mCuda_Resources.EnergyResources.springGroupsEnergySize[i] * sizeof (float));
    }
}



std::vector<uint32_t> BuildFaceIndices(
        uint32_t strideA,
        uint32_t strideB,
        uint32_t maxA,
        uint32_t maxB,
        uint32_t fixedC
    )
{
    std::vector<uint32_t> indices; indices.reserve(maxA/strideA*maxB/strideB);

    for (uint32_t i = 0; i < maxA-strideA; i+=strideA) {
        for (uint32_t j = 0; j < maxB-strideB; j+=strideB) {
            indices.push_back(i+j+fixedC);
            indices.push_back(i+j+fixedC+strideA);
            indices.push_back(i+j+fixedC+strideA+strideB);

            indices.push_back(i+j+fixedC);
            indices.push_back(i+j+fixedC+strideA+strideB);
            indices.push_back(i+j+fixedC+strideB);
        }
    }
    return indices;
}

SoftBody SoftBody::Rectangle(uint32_t l1, uint32_t l2, PhysicsParam params, DisplayMode displayMode) {

    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices1D;
    std::vector<uint32_t> indices2D;
    std::vector<uint32_t> constraints;

    uint32_t vertexCount = l1*l2;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            vertices.emplace_back((float)i-l1*.5, (float)j-l2*.5, 0);
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices1D.push_back(i*l2+j);
            indices1D.push_back((i+1)*l2+j);

            indices2D.push_back(i*l2+j);
            indices2D.push_back((i+1)*l2+(j+1));

            indices1D.push_back(i*l2+j);
            indices1D.push_back(i*l2+(j+1));

            indices2D.push_back((i+1)*l2+j);
            indices2D.push_back(i*l2+(j+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        indices1D.push_back(i*l2+l2-1);
        indices1D.push_back((i+1)*l2+l2-1);
    }

    for (int j = 0; j < l2-1; j++) {
        indices1D.push_back((l1-1)*l2+j);
        indices1D.push_back((l1-1)*l2+j+1);
    }

#if 0
    for (int j = 0; j < l2; j++) {
        constraints.push_back(j);
    }
#elif 0
    for (int j = 0; j < l1; j++) {
        constraints.push_back(j*l2-1+l2);
    }
#elif 0
    constraints.push_back(0*l2-1+l2);
    constraints.push_back((l1-1)*l2-1+l2);
#endif

    DampedSpringParams sp1D = {1., 0., 1., 0.0, 10.};
    DampedSpringParams sp2D = {1., 0., 1.414, 0., 10.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indices1D, sp1D}, {indices2D, sp2D}};
    std::vector<ConstraintSet> constraintSets = {  };

    std::vector<std::vector<uint32_t>> hullIndices = { BuildFaceIndices(l2, 1, l1 * l2, l2, 0)};

    return { vertices, springGroups, constraintSets, hullIndices, params, displayMode };
}

std::shared_ptr<SoftBody> SoftBody::Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode, std::shared_ptr<SimulationParams> parameters) {
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices1D;
    std::vector<uint32_t> indices2D;
    std::vector<uint32_t> indices3D;
    std::vector<uint32_t> constraints;
    uint32_t vertexCount = l1*l2*l3;

    vertices.reserve(vertexCount);
    for (int i = 0; i < l1; i++) {
        for (int j = 0; j < l2; j++) {
            for (int k = 0; k < l3; k++) {
                vertices.emplace_back(i-l1*.5, j-l2*.5, k-l3*.5);
            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            for (int k = 0; k < l3-1; k++) {

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back((i+1)*l3*l2+j*l3+k);

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back(i*l3*l2+(j+1)*l3+k);

                indices1D.push_back(i*l3*l2+j*l3+k);
                indices1D.push_back(i*l3*l2+j*l3+(k+1));


                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back((i+1)*l3*l2+j*l3+(k+1));

                indices2D.push_back(i*l3*l2+j*l3+k);
                indices2D.push_back((i+1)*l3*l2+(j+1)*l3+k);


                indices2D.push_back(i*l3*l2+j*l3+(k+1));
                indices2D.push_back(i*l3*l2+(j+1)*l3+k);

                indices2D.push_back((i+1)*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+(j+1)*l3+k);

                indices2D.push_back((i+1)*l3*l2+j*l3+k);
                indices2D.push_back(i*l3*l2+j*l3+(k+1));



                // TODO: Try without

                indices3D.push_back(i*l3*l2+j*l3+k);
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+j*l3+(k+1));
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+k);

                indices3D.push_back((i+1)*l3*l2+j*l3+k);
                indices3D.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+(j+1)*l3+k);
                indices3D.push_back((i+1)*l3*l2+j*l3+(k+1));

            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indices1D.push_back(i*l2*l3 + j*l3 + l3-1);
            indices1D.push_back((i+1)*l2*l3 + j*l3 + l3-1);

            indices1D.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices1D.push_back(i*l2*l3 + j*l3 + l3-1);

            indices2D.push_back(i*l2*l3 + j*l3 + l3-1);
            indices2D.push_back((i+1)*l2*l3 + (j+1)*l3 + l3-1);

            indices2D.push_back(i*l2*l3 + (j+1)*l3 + l3-1);
            indices2D.push_back((i+1)*l2*l3 + j*l3 + l3-1);
        }
    }

    for (int j = 0; j < l2-1; j++) {
        for (int k = 0; k < l3-1; k++) {

            indices1D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices1D.push_back((l1-1)*l2*l3 + j*l3 + (k+1));

            indices1D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices1D.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);

            indices2D.push_back((l1-1)*l2*l3 + j*l3 + k);
            indices2D.push_back((l1-1)*l2*l3 + (j+1)*l3 + (k+1));

            indices2D.push_back((l1-1)*l2*l3 + (j+1)*l3 + k);
            indices2D.push_back((l1-1)*l2*l3 + j*l3 + (k+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int k = 0; k < l3-1; k++) {

            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));

            indices1D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices1D.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);

            indices2D.push_back(i*l2*l3 + (l2-1)*l3 + k);
            indices2D.push_back((i+1)*l2*l3 + (l2-1)*l3 + (k+1));

            indices2D.push_back((i+1)*l2*l3 + (l2-1)*l3 + k);
            indices2D.push_back(i*l2*l3 + (l2-1)*l3 + (k+1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i*l2*l3);
        indices1D.push_back(l1*l2*l3-1-(i+1)*l2*l3);
    }
    for (int i = 0; i < l2-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i*l3);
        indices1D.push_back(l1*l2*l3-1-(i+1)*l3);
    }
    for (int i = 0; i < l3-1; i++) {
        indices1D.push_back(l1*l2*l3-1-i);
        indices1D.push_back(l1*l2*l3-1-(i+1));
    }


#if 0
    for (int i = 0; i < l1; ++i) {
        for (int k = 0; k < l3; ++k) {
            constraints.push_back(i*l2*l3+k);
        }
    }
#endif

    std::vector<uint32_t> constraintsBottomIndices = BuildFaceIndices(l3, l2 * l3, l2 * l3, l1 * l2 * l3, 0);
    std::vector<uint32_t> constraintsTopIndices = BuildFaceIndices(l2*l3, l3, l1*l2*l3, l2*l3, l3-1);

    auto * bottomCallbackInfo = new SBFaceCenterInfo( {0, static_cast<unsigned int>(l1*l2*l3-1) }, 2.f );
    auto * topCallbackInfo = new SBFaceCenterInfo( {0, static_cast<unsigned int>(l1*l2*l3-1) }, -2.f );

    DampedSpringParams sp1D = {20., 0., 1., 0.0, 100.};
    DampedSpringParams sp2D = {20., 0., std::sqrt(2.f), 0., 100.};
    DampedSpringParams sp3D = {20., 0., std::sqrt(3.f), 0., 100.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indices1D, sp1D}, {indices2D, sp2D}, {indices3D, sp3D}};

    ConstraintSet::Callback callback = SBRotationCallback;
    std::vector<ConstraintSet> constraintSets = {
            {BuildFaceIndices(l3, 1, l2*l3, l3, (l1-1)*l2*l3), callback, bottomCallbackInfo},
    };

    std::vector<std::vector<uint32_t>> hullIndices = {
            BuildFaceIndices(l3, l2 * l3, l2 * l3, l1 * l2 * l3, 0),
            BuildFaceIndices(l2*l3, l3, l1*l2*l3, l2*l3, l3-1),
            BuildFaceIndices(1, l3, l3, l2*l3, 0),
            BuildFaceIndices(l3, 1, l2*l3, l3, (l1-1)*l2*l3),
            BuildFaceIndices(l2*l3, 1, l1*l2*l3, l3, 0),
            BuildFaceIndices(1, l2*l3, l3, l1*l2*l3, (l2-1)*l3)
    };


    return std::make_shared<SoftBody>(vertices, springGroups, constraintSets, hullIndices, params, displayMode, parameters);
}

std::shared_ptr<SoftBody> SoftBody::Cube(uint32_t l1, uint32_t l2, uint32_t l3, PhysicsParam params, DisplayMode displayMode) {
    return Cube(l1, l2, l3, params, displayMode, nullptr);
}

std::shared_ptr<SoftBody> SoftBody::Torus(uint32_t longRes, uint32_t shortRes, uint32_t depthRes, PhysicsParam physicsParams, SoftBody::DisplayMode displayMode,
                                          std::shared_ptr<SimulationParams> simulationParameters) {
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indicesLong;
    std::vector<uint32_t> indicesShort;
    std::vector<uint32_t> indicesDepth;
    std::vector<uint32_t> constraints;
    std::vector<std::vector<uint32_t>> hullIndices = {{}};
    uint32_t vertexCount = longRes*shortRes*depthRes;

    vertices.reserve(vertexCount);
    for (uint32_t i = 0; i < longRes; i++) {
        static float LONG_RADIUS = 3.f;
        float th = 2.f * 3.141593f * (static_cast<float>(i)) / static_cast<float>(longRes);
        glm::vec3 planeP = LONG_RADIUS * glm::vec3(glm::cos(th), 0, glm::sin(th));
        for (uint32_t j = 0; j < shortRes; j++) {
            static float SHORT_RADIUS = 1.f;
            float phi = 2.f * 3.141593f * (static_cast<float>(j)) / (static_cast<float>(shortRes));
            glm::vec3 radialP = glm::vec3(glm::cos(phi), glm::sin(phi), 0.f)*SHORT_RADIUS;
            glm::vec3 rotRadialP = glm::vec3(glm::cos(th)*radialP.x, radialP.y, glm::sin(th)*radialP.x);
            for (uint32_t k = 1; k < depthRes+1; k++) {
                float r = static_cast<float>(k) / static_cast<float>(depthRes);
                vertices.emplace_back(r * rotRadialP + planeP);
            }
        }
    }
    for (uint32_t i = 0; i < longRes; i++) {
        for (uint32_t j = 0; j < shortRes; j++) {
            for (uint32_t k = 0; k < depthRes; k++) {
                if (k < depthRes-1) {
                    indicesDepth.emplace_back(i * depthRes * shortRes + j * depthRes + k);
                    indicesDepth.emplace_back(i * depthRes * shortRes + j * depthRes + k + 1);
                }
                indicesLong.emplace_back(i * depthRes * shortRes + j * depthRes + k);
                indicesLong.emplace_back(((i+1)%longRes) * depthRes * shortRes + j * depthRes + k);

                indicesShort.emplace_back(i * depthRes * shortRes + j * depthRes + k);
                indicesShort.emplace_back(i * depthRes * shortRes + ((j+1)%shortRes) * depthRes + k);
            }
        }
    }

    for (uint32_t i = 0; i < longRes; i++) {
        for (uint32_t j = 0; j < shortRes; j++) {
            hullIndices[0].emplace_back(i * depthRes * shortRes + j * depthRes + (depthRes-1));
            if (j+1 != shortRes) {
                hullIndices[0].emplace_back(((i+1)%longRes) * depthRes * shortRes + ((j+1)%shortRes) * depthRes + (depthRes-1));
            }
            hullIndices[0].emplace_back(((i+1)%longRes) * depthRes * shortRes + j * depthRes + (depthRes-1));
            if (j+1 == shortRes) {
                hullIndices[0].emplace_back(((i+1)%longRes) * depthRes * shortRes + ((j+1)%shortRes) * depthRes + (depthRes-1));
            }

            hullIndices[0].emplace_back(i * depthRes * shortRes + j * depthRes + (depthRes-1));
            if (i+1 != shortRes) {
                hullIndices[0].emplace_back(i * depthRes * shortRes + ((j+1)%shortRes) * depthRes + (depthRes-1));
            }
            hullIndices[0].emplace_back(((i+1)%longRes) * depthRes * shortRes + ((j+1)%shortRes) * depthRes + (depthRes-1));
            if (i+1 == shortRes) {
                hullIndices[0].emplace_back(i * depthRes * shortRes + ((j+1)%shortRes) * depthRes + (depthRes-1));
            }
        }
    }
/*
    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            for (int k = 0; k < l3-1; k++) {

                indicesLong.push_back(i * l3 * l2 + j * l3 + k);
                indicesLong.push_back((i + 1) * l3 * l2 + j * l3 + k);

                indicesLong.push_back(i * l3 * l2 + j * l3 + k);
                indicesLong.push_back(i * l3 * l2 + (j + 1) * l3 + k);

                indicesLong.push_back(i * l3 * l2 + j * l3 + k);
                indicesLong.push_back(i * l3 * l2 + j * l3 + (k + 1));


                indicesShort.push_back(i * l3 * l2 + j * l3 + k);
                indicesShort.push_back(i * l3 * l2 + (j + 1) * l3 + (k + 1));

                indicesShort.push_back(i * l3 * l2 + j * l3 + k);
                indicesShort.push_back((i + 1) * l3 * l2 + j * l3 + (k + 1));

                indicesShort.push_back(i * l3 * l2 + j * l3 + k);
                indicesShort.push_back((i + 1) * l3 * l2 + (j + 1) * l3 + k);


                indicesShort.push_back(i * l3 * l2 + j * l3 + (k + 1));
                indicesShort.push_back(i * l3 * l2 + (j + 1) * l3 + k);

                indicesShort.push_back((i + 1) * l3 * l2 + j * l3 + k);
                indicesShort.push_back(i * l3 * l2 + (j + 1) * l3 + k);

                indicesShort.push_back((i + 1) * l3 * l2 + j * l3 + k);
                indicesShort.push_back(i * l3 * l2 + j * l3 + (k + 1));



                // TODO: Try without

                indices3D.push_back(i*l3*l2+j*l3+k);
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+j*l3+(k+1));
                indices3D.push_back((i+1)*l3*l2+(j+1)*l3+k);

                indices3D.push_back((i+1)*l3*l2+j*l3+k);
                indices3D.push_back(i*l3*l2+(j+1)*l3+(k+1));

                indices3D.push_back(i*l3*l2+(j+1)*l3+k);
                indices3D.push_back((i+1)*l3*l2+j*l3+(k+1));

            }
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int j = 0; j < l2-1; j++) {
            indicesLong.push_back(i * l2 * l3 + j * l3 + l3 - 1);
            indicesLong.push_back((i + 1) * l2 * l3 + j * l3 + l3 - 1);

            indicesLong.push_back(i * l2 * l3 + (j + 1) * l3 + l3 - 1);
            indicesLong.push_back(i * l2 * l3 + j * l3 + l3 - 1);

            indicesShort.push_back(i * l2 * l3 + j * l3 + l3 - 1);
            indicesShort.push_back((i + 1) * l2 * l3 + (j + 1) * l3 + l3 - 1);

            indicesShort.push_back(i * l2 * l3 + (j + 1) * l3 + l3 - 1);
            indicesShort.push_back((i + 1) * l2 * l3 + j * l3 + l3 - 1);
        }
    }

    for (int j = 0; j < l2-1; j++) {
        for (int k = 0; k < l3-1; k++) {

            indicesLong.push_back((l1 - 1) * l2 * l3 + j * l3 + k);
            indicesLong.push_back((l1 - 1) * l2 * l3 + j * l3 + (k + 1));

            indicesLong.push_back((l1 - 1) * l2 * l3 + j * l3 + k);
            indicesLong.push_back((l1 - 1) * l2 * l3 + (j + 1) * l3 + k);

            indicesShort.push_back((l1 - 1) * l2 * l3 + j * l3 + k);
            indicesShort.push_back((l1 - 1) * l2 * l3 + (j + 1) * l3 + (k + 1));

            indicesShort.push_back((l1 - 1) * l2 * l3 + (j + 1) * l3 + k);
            indicesShort.push_back((l1 - 1) * l2 * l3 + j * l3 + (k + 1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        for (int k = 0; k < l3-1; k++) {

            indicesLong.push_back(i * l2 * l3 + (l2 - 1) * l3 + k);
            indicesLong.push_back(i * l2 * l3 + (l2 - 1) * l3 + (k + 1));

            indicesLong.push_back(i * l2 * l3 + (l2 - 1) * l3 + k);
            indicesLong.push_back((i + 1) * l2 * l3 + (l2 - 1) * l3 + k);

            indicesShort.push_back(i * l2 * l3 + (l2 - 1) * l3 + k);
            indicesShort.push_back((i + 1) * l2 * l3 + (l2 - 1) * l3 + (k + 1));

            indicesShort.push_back((i + 1) * l2 * l3 + (l2 - 1) * l3 + k);
            indicesShort.push_back(i * l2 * l3 + (l2 - 1) * l3 + (k + 1));
        }
    }

    for (int i = 0; i < l1-1; i++) {
        indicesLong.push_back(l1 * l2 * l3 - 1 - i * l2 * l3);
        indicesLong.push_back(l1 * l2 * l3 - 1 - (i + 1) * l2 * l3);
    }
    for (int i = 0; i < l2-1; i++) {
        indicesLong.push_back(l1 * l2 * l3 - 1 - i * l3);
        indicesLong.push_back(l1 * l2 * l3 - 1 - (i + 1) * l3);
    }
    for (int i = 0; i < l3-1; i++) {
        indicesLong.push_back(l1 * l2 * l3 - 1 - i);
        indicesLong.push_back(l1 * l2 * l3 - 1 - (i + 1));
    }*/


#if 0
    for (int i = 0; i < l1; ++i) {
        for (int k = 0; k < l3; ++k) {
            constraints.push_back(i*l2*l3+k);
        }
    }
#endif

    std::vector<uint32_t> constraintsBottomIndices = {}; // BuildFaceIndices(l3, l2 * l3, l2 * l3, l1 * l2 * l3, 0);
    std::vector<uint32_t> constraintsTopIndices = {}; // BuildFaceIndices(l2*l3, l3, l1*l2*l3, l2*l3, l3-1);

    auto * bottomCallbackInfo = new SBFaceCenterInfo( {0, static_cast<unsigned int>(1) }, 2.f );
    auto * topCallbackInfo = new SBFaceCenterInfo( {0, static_cast<unsigned int>(1) }, -2.f );

    DampedSpringParams sp1D = {1., 0., glm::length(vertices[indicesLong[1]]-vertices[indicesLong[0]]), 0.0, 100.};
    DampedSpringParams sp2D = {1., 0., glm::length(vertices[indicesShort[1]]-vertices[indicesShort[0]]), 0., 100.};
    DampedSpringParams sp3D = {1., 0., glm::length(vertices[indicesDepth[1]]-vertices[indicesDepth[0]]), 0., 100.};
    std::vector<std::tuple<std::vector<uint32_t>, DampedSpringParams>> springGroups = {{indicesLong, sp1D},
                                                                                       {indicesShort, sp2D},
                                                                                       {indicesDepth, sp3D}};

    ConstraintSet::Callback callback = SBRotationCallback;
    std::vector<ConstraintSet> constraintSets = {
            //{constraintsBottomIndices, callback, bottomCallbackInfo},
            //{constraintsTopIndices, callback, topCallbackInfo}
    };


    return std::make_shared<SoftBody>(vertices, springGroups, constraintSets, hullIndices, physicsParams, displayMode, simulationParameters);
}


void SoftBody::InitMesh() {

    std::vector<uint32_t> hullMeshIndices; hullMeshIndices.reserve(mHullIndices.size());
    InitMeshData(hullMeshIndices);

    mHullMesh = new Mesh<SimpleNormalVertex>(mHullMeshVertices, hullMeshIndices);
    mHullMesh->SetPrimitiveMode(GL_TRIANGLES);
    mHullMesh->SetDrawMode(GL_FILL);

    mStructureMesh = new Mesh<SimpleColorVertex>(mStructureMeshVertices, mStructureMeshIndices);
    mStructureMesh->SetPrimitiveMode(GL_LINES);
    mStructureMesh->SetDrawMode(GL_LINE);

    mBoundingBox = new WireframeBox(glm::vec3(0.), glm::vec3(0.), glm::vec3(1, 1, .5));

}

void SoftBody::BuildMesh() {
    if (!mStructureMesh) {
        InitMesh();
        return;
    }
    UpdateMeshVertices();
    mStructureMesh->ChangeVertices(mStructureMeshVertices);
    mHullMesh->ChangeVertices(mHullMeshVertices);
    mHullMesh->RecomputeVerticesAttributes();
}

void SoftBody::CudaBuildMesh() {
    if (!mStructureMesh) {
        InitMesh();
        return;
    }
    CudaUpdateMeshVertices();
    mStructureMesh->ChangeVertices(mStructureMeshVertices);
    mHullMesh->ChangeVertices(mHullMeshVertices);
    mHullMesh->RecomputeVerticesAttributes();
}


void SoftBody::ApplyConstraints() {
    for (int i = 0; i < mConstraintSets.size(); ++i) {
        if (mSimulationParams->enableConstraintSet[i]) {
            mConstraintSets[i].ApplyConstraint(mForces, mVelocities, mPositions, mSimulationParams->physicsParams.t);
        }
    }
}

void SoftBody::CudaApplyConstraints(CudaODESystem &odeSystem) {
    //Parallel_CudaApplyConstraints(mCuda_Resources, odeSystem, mSimulationParams->physicsParams.t);
}

bool SoftBody::IsSpeedExploding() {
    static float THRESHOLD = 10.;
    for (auto v : mVelocities) {
        if (glm::length(v) > THRESHOLD) {
            return true;
        }
    }
    return false;
}

void SoftBody::PhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders) {
    if (!mHasSimulationStarted || mIsSimulationPaused) { return; }
    for (int i = 0; i < mSimulationParams->subPhysicStep; ++i) {
        SolveODE(colliders, true);
        MoveMasses();
    }
    UpdateBoundingBox();
    BuildMesh();
    mSimulationParams->physicsParams.t += mSimulationParams->physicsParams.dt;
    if (mSimulationParams->stopSimIfExploding && IsSpeedExploding()) {
        mHasSimulationStarted = false; // TODO: simulation pause
    }
    ComputeSystemEnergy();
}

void SoftBody::CudaRkPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders) {
    if  (!mHasSimulationStarted || mIsSimulationPaused) { return; }
    for (int i = 0; i < mSimulationParams->subPhysicStep; ++i) {
        CudaComputeRKCoefs(colliders);
    }
    //UpdateBoundingBox();
    CudaBuildMesh();
    mSimulationParams->physicsParams.t += mSimulationParams->physicsParams.dt;
    CudaComputeSystemEnergy();
}

void SoftBody::RkPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders) {
    if  (!mHasSimulationStarted || mIsSimulationPaused) { return; }
    for (int i = 0; i < mSimulationParams->subPhysicStep; ++i) {
        ComputeRKCoefs(colliders);
    }
    UpdateBoundingBox();
    BuildMesh();
    mSimulationParams->physicsParams.t += mSimulationParams->physicsParams.dt;
    if (mSimulationParams->stopSimIfExploding && IsSpeedExploding()) {
        mHasSimulationStarted = false; // TODO: simulation pause
    }
    ComputeSystemEnergy();
}

void SoftBody::SolveODE(const std::vector<std::shared_ptr<Collider>> &colliders, bool solveCollisions) {

    ResetForces();
    ComputeForces();
    ApplyConstraints();
    if (solveCollisions) {
        SolveCollisions(colliders);
    }
}

void SoftBody::CudaSolveODE(CudaODESystem &odeSystem, const std::vector<std::shared_ptr<Collider>> &colliders, bool solveCollisions) {
    CudaResetForces(odeSystem);
    CudaComputeForces(odeSystem);
    CudaApplyConstraints(odeSystem);
    if (solveCollisions) {
        CudaSolveCollisions (odeSystem, colliders);
    }
}


void SoftBody::CudaPhysicsStep(const std::vector<std::shared_ptr<Collider>> &colliders) {
    if  (!mHasSimulationStarted || mIsSimulationPaused) { return; }
    for (int i = 0; i < mSimulationParams->subPhysicStep; ++i) {
        CudaSolveODE(mCuda_Resources.mainOdeSystem, colliders, true);
        CudaMoveMasses(mCuda_Resources.mainOdeSystem);
    }
    CudaBuildMesh();
    mSimulationParams->physicsParams.t += mSimulationParams->physicsParams.dt;
    CudaComputeSystemEnergy();
}

void SoftBody::ResetForces() {
    for (auto& force: mForces) {
        force = glm::vec3(0., mSimulationParams->physicsParams.g*mSimulationParams->vertexMass, 0.);
    }
}

void SoftBody::CudaResetForces(CudaODESystem &odeSystem) {
    Parallel_CudaSetForces(mCuda_Resources, odeSystem, glm::vec3(0., mSimulationParams->physicsParams.g*mSimulationParams->vertexMass, 0.));
}

void SoftBody::ComputeForces() {
    for (auto &springGroup: mSpringGroups) {
        for (auto &spring: springGroup.GetSprings()) {
            auto pi = mPositions[spring.i];
            auto pj = mPositions[spring.j];
            auto n = glm::normalize(pi-pj);
            float relSpeed = glm::dot(n, mVelocities[spring.i]-mVelocities[spring.j]);
            float f = 0.;
            if (spring.enabled){
                f = DampedSpring::ComputeForce(glm::length(pi-pj), relSpeed, springGroup.params);
                if (springGroup.params.enableBreak && f > springGroup.params.maxT) {
                    spring.enabled = false;
                    f = 0.;
                }
            }
            mForces[spring.i] += f*n;
            mForces[spring.j] += -f*n;
            mStructureMeshVertices[spring.n].color.y        = 1. - std::abs(f/springGroup.params.maxT);
            mStructureMeshVertices[spring.n + 1].color.y    = 1. - std::abs(f/springGroup.params.maxT);
            mStructureMeshVertices[spring.n].color.z        = 1. - std::abs(f/springGroup.params.maxT);
            mStructureMeshVertices[spring.n + 1].color.z    = 1. - std::abs(f/springGroup.params.maxT);
        }
    }
}

void SoftBody::CudaComputeForces(CudaODESystem &odeSystem) {
    Parallel_CudaComputeSpringMassForces(mCuda_Resources, odeSystem);
}

float smstep(float x, float a, float b) {
    float t = (x-a)/(b-a);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t*t*(3-2*t);
}

void SoftBody::MoveMasses() {

    // Updating velocities and positions and AABB bounds
    for (uint32_t i = 0; i < mPositions.size(); ++i) {
        mVelocities[i] += mForces[i] * mSimulationParams->physicsParams.dt / mSimulationParams->vertexMass;
        mPositions[i] += mVelocities[i] * mSimulationParams->physicsParams.dt;
        //mVertices[i].color = glm::vec3(0., smstep(glm::length(mVelocities[i]), 0., .4), 0.1)*0.f;
        mColors[i] = glm::vec3(0.);
    }

}

void SoftBody::BackupSystem() {
    mPositionsBackup = mPositions;
    mVelocitiesBackup = mVelocities;
}

void SoftBody::ApplyBackup() {
    mPositions = mPositionsBackup;
    mVelocities = mVelocitiesBackup;
}

void SoftBody::ResetRkCoefs() {
    for (int i = 0; i < 4; ++i) {
        mRK_kForces[i] = std::vector<glm::vec3>(mPositions.size());
        mRK_kVelocities[i] = std::vector<glm::vec3>(mPositions.size());
    }
}

void SoftBody::OffsetODESystem(int i, float offsetCoef) {
    if (i == 0) { return; }
    for (uint32_t j = 0; j < mVelocities.size(); ++j) {
        mVelocities[j] += offsetCoef * mRK_kForces[i-1][j] / mSimulationParams->vertexMass;
    }
    for (uint32_t j = 0; j < mPositions.size(); ++j) {
        mPositions[j] += offsetCoef * mRK_kVelocities[i-1][j];
    }
}

void SoftBody::UpdateBoundingBox() {

    // Reseting AABB bounds
    mAABBLower = mPositions[0];
    mAABBUpper = mPositions[0];

    // Updating velocities and positions and AABB bounds
    for (auto mPosition : mPositions) {
        mAABBUpper = glm::max(mAABBUpper, mPosition);
        mAABBLower = glm::min(mAABBLower, mPosition);
    }

    mAABBLower += mStructureMesh->GetPosition();
    mAABBUpper += mStructureMesh->GetPosition();

    glm::vec3 centerAABB = (mAABBUpper+mAABBLower)*.5f;
    glm::vec3 sidesAABB = mAABBUpper-centerAABB;

    mBoundingBox->UpdateBox(centerAABB, sidesAABB);
}

void SoftBody::ComputeRKCoefs(const std::vector<std::shared_ptr<Collider>> &colliders) {

    ResetRkCoefs();

    std::vector<float> stepCoefs = { 0., mSimulationParams->physicsParams.dt*.5f, mSimulationParams->physicsParams.dt*.5f, mSimulationParams->physicsParams.dt };
    BackupSystem();

    for (int i = 0; i < 4; ++i) {
        OffsetODESystem(i, stepCoefs[i]);

        SolveODE(colliders, true);
        mRK_kForces[i] = mForces;
        mRK_kVelocities[i] = mVelocities;
        ApplyBackup();
    }

    // Updating velocities and positions
    for (uint32_t i = 0; i < mPositions.size(); ++i) {
        mVelocities[i] += mSimulationParams->physicsParams.dt / 6.f * (mRK_kForces[0][i] + 2.f*mRK_kForces[1][i] + 2.f*mRK_kForces[2][i] + mRK_kForces[3][i]) / mSimulationParams->vertexMass;
        mPositions[i] += mSimulationParams->physicsParams.dt / 6.f * (mRK_kVelocities[0][i] + 2.f * mRK_kVelocities[1][i] + 2.f * mRK_kVelocities[2][i] + mRK_kVelocities[3][i]) / mSimulationParams->vertexMass;
        //mPositions[i] += mSimulationParams->physicsParams.dt * mVelocities[i];
    }

}



void SoftBody::CudaComputeRKCoefs(const std::vector<std::shared_ptr<Collider>> &colliders) {
    CudaInitRkIter();
    CudaOffsetODESystems();
    CudaSolveODE(mCuda_Resources.RKOdeSystems[0], colliders, false);
    CudaSolveODE(mCuda_Resources.RKOdeSystems[1], colliders, false);
    CudaSolveODE(mCuda_Resources.RKOdeSystems[2], colliders, false);
    CudaSolveODE(mCuda_Resources.RKOdeSystems[3], colliders, false);
    CudaApplyRkCoefs();
    CudaSolveCollisions(mCuda_Resources.mainOdeSystem, colliders);
}

void SoftBody::CudaApplyRkCoefs() {
    Parallel_CudaApplyRkCoefs(mCuda_Resources, mSimulationParams->physicsParams.dt, mSimulationParams->vertexMass);
}

void SoftBody::CudaInitRkIter() {
    Parallel_CudaInitIter(mCuda_Resources);
}

void SoftBody::CudaOffsetODESystems() {
    Parallel_OffsetODESystem(mCuda_Resources, mCuda_Resources.RKOdeSystems[0], mCuda_Resources.RKOdeSystems[0], 0., false);
    Parallel_OffsetODESystem(mCuda_Resources, mCuda_Resources.RKOdeSystems[1], mCuda_Resources.RKOdeSystems[0], mSimulationParams->physicsParams.dt*.5f, true);
    Parallel_OffsetODESystem(mCuda_Resources, mCuda_Resources.RKOdeSystems[2], mCuda_Resources.RKOdeSystems[1], mSimulationParams->physicsParams.dt*.5f, true);
    Parallel_OffsetODESystem(mCuda_Resources, mCuda_Resources.RKOdeSystems[3], mCuda_Resources.RKOdeSystems[2], mSimulationParams->physicsParams.dt, true);
}


void SoftBody::CudaMoveMasses(CudaODESystem &odeSystem) {

    // TODO Compute AABB

    Parallel_CudaMoveMassesKernel(mCuda_Resources, odeSystem, mSimulationParams->physicsParams.dt, mSimulationParams->vertexMass);
}

void SoftBody::Draw(const PerspectiveCamera &camera, Shader& shader) {
    if (mSimulationParams->displayMode == Hull) {
        mHullMesh->Draw(camera, shader);
    } else if (mSimulationParams->displayMode == Structure) {
        mStructureMesh->Draw(camera, shader);
    }
    mBoundingBox->Draw(camera, shader);
}

void SoftBody::InitMeshData(std::vector<uint32_t> &hullMeshIndices) {

    // -- Building Hull rendering mesh data -- //

    mHullMeshVertices.reserve(mHullIndices.size());
    uint32_t counter = 0;
    for (uint32_t i = 0; i < mHullIndices.size(); ++i) {
        for (uint32_t &mHullIndex: mHullIndices[i]) {
            if (mHullVerticesLUT[i].find(mHullIndex) == mHullVerticesLUT[i].end()) {
                mHullMeshVertices.emplace_back(mPositions[mHullIndex], glm::vec3(1.));
                mHullVerticesLUT[i].insert({mHullIndex, counter});
                hullMeshIndices.push_back(counter++);
            } else {
                hullMeshIndices.push_back(mHullVerticesLUT[i][mHullIndex]);
            }
        }
    }

    // -- Building Structure rendering mesh data -- //

    size_t meshVertexCount = 0;
    for (auto &springGroups: mSpringGroups) {
        meshVertexCount += springGroups.GetSprings().size()*2;
    }
    mStructureMeshVertices.reserve(meshVertexCount);
    mStructureMeshIndices.reserve(meshVertexCount);
    uint32_t count = 0;
    for (auto &springGroups: mSpringGroups) {
        for (auto &spring: springGroups.GetSprings()) {
            mStructureMeshVertices.emplace_back(mPositions[spring.i], glm::vec3(1.));
            mStructureMeshVertices.emplace_back(mPositions[spring.j], glm::vec3(1.));
            mStructureMeshIndices.emplace_back(count++);
            mStructureMeshIndices.emplace_back(count++);
        }
    }
}

void SoftBody::UpdateMeshVertices() {

    if (mSimulationParams->displayMode == Hull) {
        for (auto & face : mHullVerticesLUT) {
            for (const auto &vertexLU: face) {
                mHullMeshVertices[vertexLU.second] = SimpleNormalVertex(mPositions[vertexLU.first], glm::vec3(0.));
            }
        }
    } else if (mSimulationParams->displayMode == Structure) {
        for (auto &springGroup: mSpringGroups) {
            for (auto &spring: springGroup.GetSprings()) {
                mStructureMeshVertices[spring.n].position = mPositions[spring.i];
                mStructureMeshVertices[spring.n + 1].position = mPositions[spring.j];
                //mMeshVertices[i].color.g = mVertices[i].color.g;
            }
        }
        for (auto &springGroup: mSpringGroups) {
            for (auto &spring: springGroup.GetSprings()) {
                if (!spring.enabled) {
                    mStructureMeshVertices[spring.n + 1].position = mStructureMeshVertices[spring.n].position;
                }
            }
        }
    }
}

void SoftBody::CudaUpdateMeshVertices() {
    cudaMemcpy(mPositions.data(), mCuda_Resources.mainOdeSystem.Positions, mPositions.size() * sizeof (glm::vec3), cudaMemcpyDeviceToHost);
    UpdateMeshVertices();
}

#define MEASURE_TIME_START auto start_time = std::chrono::high_resolution_clock::now();
#define MEASURE_TIME_END auto end_time = std::chrono::high_resolution_clock::now(); \
                           auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time); \
                           std::cout << "Execution Time: " << duration.count() << " microseconds" << std::endl;


void SoftBody::SolveCollisions(const std::vector<std::shared_ptr<Collider>> &colliders) {
    for (auto &collider: colliders) {
        if (!collider->Intersect(mAABBLower, mAABBUpper)) {
            continue;
        }

//#define PARALLEL
#ifdef PARALLEL
        //MEASURE_TIME_START
        ParallelSolveCollision(mStructureMesh, collider->GetSphere(), mStructureMesh->GetPosition(), mSimulationParams->physicsParams.dt);
        //MEASURE_TIME_END
#else
        //MEASURE_TIME_START
        for (int i = 0; i < mPositions.size(); ++i) {
            if (collider->Intersect(mPositions[i])) {
                auto path = collider->ShortestSurfacePoint(mPositions[i] + mStructureMesh->GetPosition());
                //mPositions[i] += path;
                //mVelocities[i] = path/mSimulationParams->physicsParams.dt; // TODO : not working for many colliders
                mForces[i] += 20.f*glm::normalize(path); // TODO : not working for many colliders
            }
            //mForces[i] += collider->ComputeCollisionForce(mPositions[i]+mMesh->GetPosition())*.1f;
        }
        //MEASURE_TIME_END
#endif
    }

    auto boundaryCenter = mSimulationParams->boundaryBox->GetCenter();
    auto boundarySides = mSimulationParams->boundaryBox->GetSides();

    for (int i = 0; i < mPositions.size(); ++i) {
        auto relPos = mPositions[i] - boundaryCenter;
        auto diff = glm::min(boundarySides - glm::abs(relPos), glm::vec3(0.));
        // Elastic contact
        //glm::vec3 normalReaction = mSimulationParams->boundarySpringK * diff * glm::sign(relPos);
        glm::vec3 normalReactionDir = diff * glm::sign(relPos);
        if (glm::length(normalReactionDir) < 0.1) {
            continue;
        }
        normalReactionDir = glm::normalize(normalReactionDir);
        glm::vec3 normalReaction = glm::max(0.f, -glm::dot(mForces[i], normalReactionDir)) * normalReactionDir;
        glm::vec3 tangentialReaction;
        glm::vec3 velocityProjection = mVelocities[i]-glm::dot(mVelocities[i], normalReactionDir) * normalReactionDir;
        if (glm::length(velocityProjection) > 0.001) {
            tangentialReaction = -glm::normalize(velocityProjection) * mSimulationParams->boundaryF * glm::length(normalReaction);
        } else {
            tangentialReaction = -(mForces[i] - glm::dot(mForces[i], normalReactionDir)*normalReactionDir);
            if (glm::length(tangentialReaction) > mSimulationParams->boundaryF * glm::length(normalReaction)) {
                tangentialReaction = mSimulationParams->boundaryF * glm::length(normalReaction)*glm::normalize(tangentialReaction);
            }
        }

        mForces[i] += normalReaction + tangentialReaction;
        if (glm::dot(mVelocities[i], normalReactionDir) < 0.) {
            mVelocities[i] -= glm::dot(mVelocities[i], normalReactionDir)*normalReactionDir;
        }
    }
}

void SoftBody::CudaSolveCollisions(CudaODESystem &odeSystem, const std::vector<std::shared_ptr<Collider>> &colliders) {
    for (auto &collider: colliders) {
        ParallelSolveSphereCollision(mCuda_Resources, odeSystem, collider->GetSphere());
    }
    ParallelSolveCollision(mCuda_Resources, odeSystem, mSimulationParams);
}


void SoftBody::DrawWindow() {

    bool startingNow = !mHasSimulationStarted;
    mHasSimulationStarted = ImGui::SliderFloat("dt", &mSimulationParams->physicsParams.dt, 0, .15) || mHasSimulationStarted;

    if  (!mHasSimulationStarted || mIsSimulationPaused) {
        bool shouldUpdate = false;
        shouldUpdate = ImGui::SliderFloat3("center", glm::value_ptr(mSimulationParams->centerPosition), -10., 40.) || shouldUpdate;
        shouldUpdate = ImGui::SliderFloat("scale", &mSimulationParams->scale, .01, 10.) || shouldUpdate;
        if (shouldUpdate) {
            UpdateIC();
        }
    } else if (startingNow) {
        StartSimulation();
    }

    ImGui::Checkbox("Explosion control", &mSimulationParams->stopSimIfExploding);


    ImGui::SliderFloat("g", &mSimulationParams->physicsParams.g, -.1, 0);
    ImGui::SliderFloat("m", &mSimulationParams->vertexMass, 0.001, 3.);
    ImGui::SliderFloat("f", &mSimulationParams->boundaryF, 0.0, 1.5);
    ImGui::SliderInt("Display mode", reinterpret_cast<int *>(&mSimulationParams->displayMode), 0, 1);
    ImGui::SliderInt("Sub Physic Update", reinterpret_cast<int *>(&mSimulationParams->subPhysicStep), 0, 20);
    for (int i = 0; i < mConstraintSets.size(); ++i) {
        char buff[100];
        snprintf(buff, sizeof(buff), "Enable constraints %i", i);
        ImGui::Checkbox(buff, reinterpret_cast<bool *>(&mSimulationParams->enableConstraintSet[i]));
    }

    for (int i = 0; i < mSpringGroups.size(); ++i) {
        char buff[100];
        snprintf(buff, sizeof(buff), "Spring %i (%zu)", i, mSpringGroups[i].GetSprings().size());
        if (ImGui::CollapsingHeader(buff)) {
            bool shouldUpdateSpringGroup = false;
            shouldUpdateSpringGroup = ImGui::SliderFloat("k", &mSpringGroups[i].params.k, 0., 100.) || shouldUpdateSpringGroup;
            shouldUpdateSpringGroup = ImGui::SliderFloat("l0", &mSpringGroups[i].params.l0Mult, 0.1, 2.) || shouldUpdateSpringGroup;
            shouldUpdateSpringGroup = ImGui::SliderFloat("a", &mSpringGroups[i].params.a, -1., 1.) || shouldUpdateSpringGroup;
            shouldUpdateSpringGroup = ImGui::SliderFloat("Relaxation", &mSpringGroups[i].params.rDist, -1., 1.) || shouldUpdateSpringGroup;
            shouldUpdateSpringGroup = ImGui::SliderFloat("Max Tension", &mSpringGroups[i].params.maxT, 0.000001, 100.) || shouldUpdateSpringGroup;
            shouldUpdateSpringGroup = ImGui::Checkbox("Enable Break", &mSpringGroups[i].params.enableBreak) || shouldUpdateSpringGroup;
            if (shouldUpdateSpringGroup && mHasSimulationStarted) {
#ifdef ENABLE_CUDA
                mCuda_Resources.SpringGroups[i].params = mSpringGroups[i].params;
#endif
            }
        }
    }
    if (ImPlot::BeginPlot("Energy")) {
        ImPlot::SetupAxes("Time [s]","Energy");

        int span = 2000;
        float* timeStart = mPltTime.data() + mPltTime.size() - span;
        int count = span;
        int offset = mPltTime.size()-span;
        float maxTime = mSimulationParams->physicsParams.t;
        if (timeStart < mPltTime.data()) {
            timeStart = mPltTime.data();
            count = mPltTime.size();
            offset = 0;
            maxTime = mSimulationParams->physicsParams.dt*span;
        }

        ImPlot::SetupAxesLimits(0,100,0,300);
        ImPlot::SetupLegend(ImPlotLocation_NorthEast);
        if (mHasSimulationStarted) {

            ImPlot::PlotLine("KineticEnergy(t)", timeStart, mPltKineticEnergy.data() + offset, count);
            ImPlot::PlotLine("GravityPotentialEnergy(t)", timeStart, mPltGravityPotentialEnergy.data() + offset, count);
            for (int i = 0; i < mPltSpringsPotentialEnergy.size(); ++i) {
                char buff[100];
                snprintf(buff, sizeof(buff), "Springs[%i]PotentialEnergy(t) ", i);
                ImPlot::PlotLine(buff, timeStart, mPltSpringsPotentialEnergy[i].data() + offset, count);
            }
            ImPlot::PlotLine("MechanicalEnergy(t)", timeStart, mPltMechanicalEnergy.data() + offset, count);
        }
        ImPlot::EndPlot();
    }

}


std::shared_ptr<SimulationParams>   SoftBody::GetParams() {
    mSimulationParams->springParams.clear();
    for (auto & springGroup : mSpringGroups) {
        mSimulationParams->springParams.push_back(springGroup.params);
    }
    return mSimulationParams;
}


static Gnuplot mSystemEnergyPlot;

///////////////////////////////////////////
// -- Simulation Analysis Helper Func -- //
///////////////////////////////////////////

void SoftBody::InitEnergyPlot() {
    mPltTime.reserve(50000);
    mPltKineticEnergy.reserve(50000);
    mPltGravityPotentialEnergy.reserve(50000);
    mPltMechanicalEnergy.reserve(50000);


    for (int i = 0; i < mSpringGroups.size(); ++i) {
        mPltSpringsPotentialEnergy.emplace_back();
        mPltSpringsPotentialEnergy.back().reserve(50000);
    }
    // mSystemEnergyPlot << "set title 'Evolution'\n";
    // mSystemEnergyPlot << "set xrange [0:20]\n";
    // mSystemEnergyPlot << "set yrange [0:300]\n";
    // mSystemEnergyPlot << "set term wxt background 'white'\n";  // Set the background color to white
    // mSystemEnergyPlot << "set xlabel 'Time'\n";
    // mSystemEnergyPlot << "set ylabel 'Energy'\n";
}

void SoftBody::PlotEnergy() {
    // Set up the Gnuplot plot
    // mSystemEnergyPlot << "set terminal gif animate\n";
    // mSystemEnergyPlot << "set output 'runtime_plot.gif'\n";
    // mSystemEnergyPlot << "set title 'Kinetic Energy'\n";
    // mSystemEnergyPlot << "set xlabel 'T'\n";
    // mSystemEnergyPlot << "set ylabel 'E'\n";

    // Set up the plot
    // mSystemEnergyPlot << "set title 'Evolution'\n";
    // mSystemEnergyPlot << "set xrange [0:20]\n";
    // mSystemEnergyPlot << "set yrange [0:300]\n";
    // mSystemEnergyPlot << "set xlabel 'Time'\n";
    // mSystemEnergyPlot << "set ylabel 'Energy'\n";
    // mSystemEnergyPlot << "plot '-' with linespoints title 'Evolution'\n";
    // mSystemEnergyPlot << "set grid\n";

    std::vector<std::pair<double, double>> data; data.reserve(15000);
    for (auto i = 0; i < mPltTime.size(); ++i) {
        data.emplace_back(mPltTime[i], mPltKineticEnergy[i]);
    }

    // Send the data to Gnuplot
    //mSystemEnergyPlot.send1d(data);


}

void SoftBody::ComputeSystemEnergy() {
    mPltTime.push_back(mSimulationParams->physicsParams.t*mSimulationParams->subPhysicStep);
    float kineticEnergy = 0.;
    for (auto &v: mVelocities) {
        kineticEnergy += glm::dot(v, v);
    }
    mPltKineticEnergy.push_back(kineticEnergy*.5f); // TODO :  mass is 1. for now

    float potentialEnergy = 0.;
    for (auto &p: mPositions) {
        potentialEnergy += p.y;
    }
    mPltGravityPotentialEnergy.push_back(-potentialEnergy * mSimulationParams->physicsParams.g); // TODO :  mass is 1. for now

    for (int i = 0; i < mSpringGroups.size(); ++i) {
        float springGroupEnergy = 0.;
        for (auto &spring : mSpringGroups[i].GetSprings()) {
            auto pi = mPositions[spring.i];
            auto pj = mPositions[spring.j];
            if (spring.enabled){
                springGroupEnergy += DampedSpring::ComputePotentialEnergy(glm::length(pi-pj), mSpringGroups[i].params);
            }
        }
        mPltSpringsPotentialEnergy[i].push_back(.5*mSpringGroups[i].params.k*springGroupEnergy);
    }
    mPltGravityPotentialEnergy.push_back(-potentialEnergy * mSimulationParams->physicsParams.g); // TODO :  mass is 1. for now


    mPltMechanicalEnergy.push_back(mPltGravityPotentialEnergy[mPltGravityPotentialEnergy.size() - 1] + mPltKineticEnergy[mPltKineticEnergy.size() - 1]);

    for(auto & i : mPltSpringsPotentialEnergy) {
        mPltMechanicalEnergy.back() += i.back();
    }

    // Plot Kinetic Energy relative to Time
    //mSystemEnergyPlot << "plot '-' with lines title 'Kinetic Energy'\n";
    //mSystemEnergyPlot.send1d(std::vector<std::tuple<float, float>>{{mPltTime.back(), mPltKineticEnergy.back()}});
    //mSystemEnergyPlot << "\n";

}


void SoftBody::CudaComputeSystemEnergy() {
    mPltTime.push_back(mSimulationParams->physicsParams.t*mSimulationParams->subPhysicStep);

    auto ke = Parallel_ComputeKineticEnergy(mCuda_Resources, mSimulationParams->vertexMass);
    auto pe = Parallel_ComputePotentialEnergy(mCuda_Resources, mSimulationParams->vertexMass, mSimulationParams->physicsParams.g);
    mPltKineticEnergy.push_back(ke);
    mPltGravityPotentialEnergy.push_back(pe);
    mPltMechanicalEnergy.push_back(ke+pe);
    for (int i = 0; i < mSpringGroups.size(); ++i) {
        auto spe = Parallel_ComputeSpringsEnergy(mCuda_Resources, i, mSimulationParams->vertexMass);
        mPltSpringsPotentialEnergy[i].push_back(spe);
        mPltMechanicalEnergy.back() += spe;
    }
}
