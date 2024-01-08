#ifndef VISUAL_SOFTBODY_CU
#define VISUAL_SOFTBODY_CU

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include "softbody2D.cuh"
#include "Mesh.h"

#define CUDA_CHECK_ERROR(err) \
    {                         \
        bool fail = false; \
        do { \
            cudaError_t cudaErr = err; \
            if (cudaErr != cudaSuccess) { \
                fail = true;      \
                fprintf(stderr, "CUDA error in %s at line %d: %s (%d)\n", \
                        __FILE__, __LINE__, cudaGetErrorString(cudaErr), cudaErr); \
            } \
        } while (0);               \
        if (fail) { \
            exit(EXIT_FAILURE);   \
        }                         \
    }\

__global__ void Ker_PointSphereCollision(uint32_t N, glm::vec3 *force, glm::vec3 *pos, glm::vec4 sphere)
{
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }

    auto a = pos[i]-glm::vec3(sphere);
    float sideDistSq = glm::dot(a, a)-sphere.a*sphere.a;
    if (sideDistSq > 0.) {
        return;
    }
    //glm::vec3 path = glm::normalize(a)*sqrt(-sideDistSq);
    force[i] += 100.f*glm::normalize(a); // TODO : not working for many colliders

    //vel[i] = path/dt; // TODO : not working for many colliders
}

#define MEASURE_TIME_START auto start_time = std::chrono::high_resolution_clock::now();
#define MEASURE_TIME_END auto end_time = std::chrono::high_resolution_clock::now(); \
                           auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time); \
                           std::cout << "Execution Time: " << duration.count() << " microseconds" << std::endl;


__global__ void Ker_SolveGroundCollisions(int N, glm::vec3* positions, glm::vec3* forces, glm::vec3 boundaryCenter, glm::vec3 boundarySides, float boundaryK) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }

    auto relPos = positions[i] - boundaryCenter;
    auto diff = glm::min(boundarySides - glm::abs(relPos), glm::vec3(0.));
    forces[i] += boundaryK * diff * glm::sign(relPos);
}

void ParallelSolveSphereCollision(const CudaResources& resources, CudaODESystem &odeSystem, glm::vec4 sphere) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 128;
    Ker_PointSphereCollision<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, odeSystem.TotalForces, odeSystem.Positions, sphere);
}

void ParallelSolveCollision(const CudaResources& resources, CudaODESystem &odeSystem, const std::shared_ptr<SimulationParams> params) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 128;
    Ker_SolveGroundCollisions<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, odeSystem.Positions, odeSystem.TotalForces, params->boundaryBox->GetCenter(), params->boundaryBox->GetSides(), params->boundarySpringK);
}



__global__ void Ker_CudaSetForces(uint32_t N, glm::vec3* buffer, glm::vec3 v) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    buffer[i] = v;
}

void Parallel_CudaSetForces(const CudaResources& resources, CudaODESystem &odeSystem, glm::vec3 f) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 128;
    Ker_CudaSetForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, odeSystem.TotalForces, f);
    N = resources.ForcesPerMass*resources.MassCount;
    Ker_CudaSetForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, odeSystem.Forces, glm::vec3(0));
}

__device__ float Hel_ComputeForce(float l, float rel_speed, const DampedSpringParams& p) {
    float spring = -p.k*max(0.f, std::abs(l-p.l0*p.l0Mult)-p.rDist); // TODO ? Could be optimized if relaxation is not used
    if (l-p.l0*p.l0Mult < 0) { spring *= -1; }
    float damping = p.a * rel_speed;
    return spring - damping;
}

__global__ void Ker_CudaComputeSpringMassForces (uint32_t N, DampedSpringParams params, DampedSpring* springs, glm::vec3* positions, glm::vec3* velocities, glm::vec3* forces, uint32_t fPerMass) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    auto pi = positions[springs[i].i];
    auto pj = positions[springs[i].j];
    auto n = glm::normalize(pi-pj);
    float relSpeed = glm::dot(n, velocities[springs[i].i]-velocities[springs[i].j]);
    float f = 0.;
    if (springs[i].enabled){
        //printf("Length of %i - %i is %f / %f)\n", springs[i].i, springs[i].j, glm::length(pi - pj), params.l0*params.l0Mult);

        f = Hel_ComputeForce(glm::length(pi - pj), relSpeed, params);
        if (f > params.maxT) {
            springs[i].enabled = false;
            f = 0.;
        }
    }
    //printf("Force applied on %i per %i is (%f, %f, %f)\n", springs[i].i, springs[i].j, (f*n).x, (f*n).y, (f*n).z);

    forces[springs[i].i*fPerMass+springs[i].i_sp] = f*n;
    forces[springs[i].j*fPerMass+springs[i].j_sp] = -f*n;
    // TODO COLORS
    //mStructureMeshVertices[spring.n].color.y = 1. - std::abs(f) * 5.;
    //mStructureMeshVertices[spring.n + 1].color.y = 1. - std::abs(f) * 5.;
    //mStructureMeshVertices[spring.n].color.z = 1. - std::abs(f) * 5.;
    //mStructureMeshVertices[spring.n + 1].color.z = 1. - std::abs(f) * 5.;
}

__global__ void Ker_ReduceForces (uint32_t N, glm::vec3* totalForces, glm::vec3* forces, uint8_t fPerMass) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    for (uint8_t j = 0; j < fPerMass; ++j) {
        totalForces[i] += forces[i*fPerMass+j];
    }
    //printf("Force exerced on %i is (%f, %f, %f)\n", i, totalForces[i].x, totalForces[i].y, totalForces[i].z);
}

void Parallel_CudaComputeSpringMassForces(const CudaResources& resources, CudaODESystem &odeSystem) {
    for (auto &springGroup: resources.SpringGroups) {
        uint32_t N = springGroup.springCount;
        uint32_t blockSize = 128;
        Ker_CudaComputeSpringMassForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, springGroup.params,
                                                                                        springGroup.springs,
                                                                                        odeSystem.Positions,
                                                                                        odeSystem.Velocities,
                                                                                        odeSystem.Forces,
                                                                                        resources.ForcesPerMass);
        N = resources.MassCount;
        Ker_ReduceForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, odeSystem.TotalForces, odeSystem.Forces, resources.ForcesPerMass);
    }
}

__global__ void Ker_CudaMoveMasses(uint32_t N, glm::vec3* Positions, glm::vec3* Velocities, glm::vec3* TotalForces, float dt, float mass) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    // Updating velocities and positions and AABB bounds
    Velocities[i] += TotalForces[i] * dt / mass;
    Positions[i] += Velocities[i]*dt;
    //printf("New position of %i is (%f, %f, %f)\n", i, Positions[i].x, Positions[i].y, Positions[i].z);

    //Vertices[i].color = glm::vec3(0., smstep(glm::length(Velocities[i]), 0., .4), 0.1)*0.f;
}


void Parallel_CudaMoveMassesKernel(const CudaResources& resources, CudaODESystem &odeSystem, float dt, float vertexMass) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 128;
    Ker_CudaMoveMasses<<<(N + blockSize - 1) / blockSize, blockSize>>>(
            N,
            odeSystem.Positions,
            odeSystem.Velocities,
            odeSystem.TotalForces,
            dt,
            vertexMass
        );

}

__global__ void Ker_ApplyConstraints(uint32_t N, CudaConstraintSet cs, glm::vec3* Velocities, glm::vec3* TotalForces, float dt, float freq) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    Velocities[cs.vertices[i]] = glm::vec3(glm::sin(dt*freq)*1.f*freq, 0., 0.);
    TotalForces[cs.vertices[i]] = glm::vec3(0.f);
}


void Parallel_CudaApplyConstraints (const CudaResources& resources, CudaODESystem &odeSystem, float dt) {
    uint32_t blockSize = 128;
    for (auto constaintSet: resources.ConstraintSets) {
        uint32_t N = constaintSet.size;
        Ker_ApplyConstraints<<<(N + blockSize - 1) / blockSize, blockSize>>>(
                N,
                constaintSet,
                odeSystem.Velocities,
                odeSystem.TotalForces,
                dt,
                constaintSet.freq
        );
    }
}

void Parallel_CudaInitIter (CudaResources& resources) {
    for (auto &ode: resources.RKOdeSystems) {
        // ode.Positions is erased by the offset
        cudaMemcpy(ode.Velocities, resources.mainOdeSystem.Velocities, resources.MassCount*sizeof(glm::vec3), cudaMemcpyDeviceToDevice);
        cudaMemcpy(ode.TotalForces, resources.mainOdeSystem.TotalForces, resources.MassCount*sizeof(glm::vec3), cudaMemcpyDeviceToDevice);
        cudaMemcpy(ode.Forces, resources.mainOdeSystem.Forces, resources.ForcesPerMass*resources.MassCount*sizeof(glm::vec3), cudaMemcpyDeviceToDevice);
    }
}



__global__ void Ker_OffsetODESystem (uint32_t N, glm::vec3 *dstBuffer, glm::vec3 *srcBuffer, glm::vec3 *offBuffer, float stepSize) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    dstBuffer[i] = srcBuffer[i] + offBuffer[i] * stepSize;
}


void Parallel_OffsetODESystem (const CudaResources& resources, CudaODESystem &odeSystem, const CudaODESystem &offBuffer, float stepSize, bool offset) {
    if (!offset) {
        cudaMemcpy(odeSystem.Positions, resources.mainOdeSystem.Positions, resources.MassCount*sizeof(glm::vec3), cudaMemcpyDeviceToDevice);
        return;
    }
    uint32_t blockSize = 128;
    uint32_t N = resources.MassCount;
    Ker_OffsetODESystem<<<(N + blockSize - 1) / blockSize, blockSize>>>(
            resources.MassCount,
            odeSystem.Positions,
            resources.mainOdeSystem.Positions,
            offBuffer.Positions,
            stepSize);

}



__global__ void Ker_CudaApplyRkCoefs(uint32_t N,
                                     glm::vec3* Positions,
                                     glm::vec3* Velocities,
                                     glm::vec3* kForces_1,
                                     glm::vec3* kForces_2,
                                     glm::vec3* kForces_3,
                                     glm::vec3* kForces_4,
                                     glm::vec3* kVelocities_1,
                                     glm::vec3* kVelocities_2,
                                     glm::vec3* kVelocities_3,
                                     glm::vec3* kVelocities_4,
                                     float dt,
                                     float mass) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    // Updating velocities and positions
    Velocities[i] += dt / 6.f * (kForces_1[i] + 2.f*kForces_2[i] + 2.f*kForces_3[i] + kForces_4[i]) * mass;
    Positions[i] += dt / 6.f * (kVelocities_1[i] + 2.f * kVelocities_2[i] + 2.f * kVelocities_3[i] + kVelocities_4[i]);
}


void Parallel_CudaApplyRkCoefs (CudaResources& resources, float dt, float mass) {
    uint32_t blockSize = 128;
    uint32_t N = resources.MassCount;
    Ker_CudaApplyRkCoefs<<<(N + blockSize - 1) / blockSize, blockSize>>>(
            N,
            resources.mainOdeSystem.Positions,
            resources.mainOdeSystem.Velocities,
            resources.RKOdeSystems[0].TotalForces,
            resources.RKOdeSystems[1].TotalForces,
            resources.RKOdeSystems[2].TotalForces,
            resources.RKOdeSystems[3].TotalForces,
            resources.RKOdeSystems[0].Velocities,
            resources.RKOdeSystems[1].Velocities,
            resources.RKOdeSystems[2].Velocities,
            resources.RKOdeSystems[3].Velocities,
            dt, mass
    );
}

__global__ void Ker_Sum(float *array) {
    __shared__ float sdata[1024];
    // load shared mem
    unsigned int tid = threadIdx.x;
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    sdata[tid] = array[i];

    __syncthreads();
    // do reduction in shared mem
    for (unsigned int s = blockDim.x/2; s > 0; s >>= 1) {
        __syncthreads();
        if (tid < s) {
            sdata[tid] += sdata[tid + s];
        }
    }
    __syncthreads();
    // write result for this block to global mem
    if (tid == 0) array[blockIdx.x] = sdata[0];
}

__global__ void Ker_RetreiveKineticEnergy(int N, glm::vec3* velocities, float* speeds) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) {
        speeds[i] = 0.;
        return;
    }

    speeds[i] = glm::dot(velocities[i], velocities[i]);
}

__global__ void Ker_RetreivePotentialEnergy(int N, glm::vec3* positions, float* height) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) {
        height[i] = 0.;
        return;
    }
    height[i] = positions[i].y;
}



__device__ float Hel_ComputeSpringPotentialEnergy(float l, const DampedSpringParams& p) {
    float v = max(0.f, std::abs(l-p.l0*p.l0Mult)-p.rDist); // TODO ? Could be optimized if relaxation is not used
    return v*v;
}

__global__ void Ker_CudaComputeSpringMassEnergy (uint32_t N, DampedSpringParams params, DampedSpring* springs, glm::vec3 *positions, float *energy) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) {
        energy[i] = 0;
        return;
    }
    auto pi = positions[springs[i].i];
    auto pj = positions[springs[i].j];
    if (springs[i].enabled){
        energy[i] = Hel_ComputeSpringPotentialEnergy(glm::length(pi - pj), params);
    } else {
        energy[i] = 0;
    }
}


float Parallel_ComputeKineticEnergy(CudaResources &resources, float mass) {
    uint32_t blockSize = 1024;
    uint32_t N = resources.EnergyResources.kineticEnergySize;
    Ker_RetreiveKineticEnergy<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.MassCount, resources.mainOdeSystem.Velocities, resources.EnergyResources.kineticEnergy);
    uint32_t sumEltSize = resources.EnergyResources.kineticEnergySize;
    if (resources.EnergyResources.kineticEnergySize > 1024) {
        Ker_Sum<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.EnergyResources.kineticEnergy);
        sumEltSize = resources.EnergyResources.kineticEnergySize / blockSize;
    }
    auto ke = std::vector<float>(); ke.reserve(sumEltSize); ke.resize(sumEltSize);
    cudaMemcpy(ke.data(), resources.EnergyResources.kineticEnergy, sumEltSize * sizeof(float), cudaMemcpyDeviceToHost);
    float s = 0.;
    for (int i = 0; i < sumEltSize; ++i) {
        s += ke[i];
    }
    return s*0.5f*mass;
}

float Parallel_ComputePotentialEnergy(CudaResources &resources, float mass, float gravityConstant) {
    uint32_t blockSize = 1024;
    uint32_t N = resources.EnergyResources.potentialEnergySize;
    Ker_RetreivePotentialEnergy<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.MassCount, resources.mainOdeSystem.Positions, resources.EnergyResources.potentialEnergy);
    uint32_t sumEltSize = resources.EnergyResources.potentialEnergySize;
    if (resources.EnergyResources.potentialEnergySize > 1024) {
        Ker_Sum<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.EnergyResources.potentialEnergy);
        sumEltSize = resources.EnergyResources.potentialEnergySize / blockSize;
    }
    auto ke = std::vector<float>(); ke.reserve(sumEltSize); ke.resize(sumEltSize);
    cudaMemcpy(ke.data(), resources.EnergyResources.potentialEnergy, sumEltSize * sizeof(float), cudaMemcpyDeviceToHost);
    float s = 0.;
    for (int i = 0; i < sumEltSize; ++i) {
        s += ke[i];
    }
    return s*mass*std::abs(gravityConstant);
}

float Parallel_ComputeSpringsEnergy    (CudaResources& resources, int i, float mass) {
    uint32_t blockSize = 1024;
    uint32_t N = resources.EnergyResources.springGroupsEnergySize[i];
    Ker_CudaComputeSpringMassEnergy<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.SpringGroups[i].springCount, resources.SpringGroups[i].params, resources.SpringGroups[i].springs, resources.mainOdeSystem.Positions, resources.EnergyResources.springGroupsEnergy[i]);

    uint32_t sumEltSize = resources.EnergyResources.springGroupsEnergySize[i];
    if (resources.EnergyResources.springGroupsEnergySize[i] > 1024) {
        Ker_Sum<<<(N + blockSize - 1) / blockSize, blockSize>>>(resources.EnergyResources.springGroupsEnergy[i]);
        sumEltSize = resources.EnergyResources.springGroupsEnergySize[i] / blockSize;
    }
    auto ke = std::vector<float>(); ke.reserve(sumEltSize); ke.resize(sumEltSize);
    cudaMemcpy(ke.data(), resources.EnergyResources.springGroupsEnergy[i], sumEltSize * sizeof(float), cudaMemcpyDeviceToHost);
    float s = 0.;
    for (int j = 0; j < sumEltSize; ++j) {
        s += ke[j];
    }
    return s*mass*resources.SpringGroups[i].params.k;
}

#endif
