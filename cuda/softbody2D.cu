#ifndef VISUAL_SOFTBODY_CU
#define VISUAL_SOFTBODY_CU

#include <iostream>
#include <vector>
#include <chrono>
#include <glm/gtx/string_cast.hpp>
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

__global__ void PointSphereCollision(uint32_t N, SimpleColorVertex *pos, glm::vec4 sphere, glm::vec3 offset, float dt)
{
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }

    auto a = pos[i].position+offset-glm::vec3(sphere);
    float sideDistSq = glm::dot(a, a)-sphere.a*sphere.a;
    if (sideDistSq > 0.) {
        return;
    }
    glm::vec3 path = glm::normalize(a)*std::sqrt(-sideDistSq);

    pos[i].position += path;
    //vel[i] = path/dt; // TODO : not working for many colliders
}

__global__ void Init(uint32_t N, float*v) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    v[i] = 42.;
}


#define MEASURE_TIME_START auto start_time = std::chrono::high_resolution_clock::now();
#define MEASURE_TIME_END auto end_time = std::chrono::high_resolution_clock::now(); \
                           auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time); \
                           std::cout << "Execution Time: " << duration.count() << " microseconds" << std::endl;


void ParallelSolveCollision(Mesh<SimpleColorVertex>* mesh, glm::vec4 sphere, glm::vec3 offset, float dt)
{

    //// Map the CUDA resource to get a device pointer
    //SimpleColorVertex* cudaPtr;
    //size_t size;
    //CUDA_CHECK_ERROR(cudaGraphicsMapResources(1, &mesh->mCuda_vertexBuffer, 0));
    //CUDA_CHECK_ERROR(cudaGraphicsResourceGetMappedPointer((void**)&cudaPtr, &size, mesh->mCuda_vertexBuffer));
//
    //uint32_t N = size / sizeof(SimpleColorVertex);
    //uint32_t blockSize = 1024;
//
    //MEASURE_TIME_START
    //PointSphereCollision<<<(N+blockSize-1)/blockSize, blockSize>>>(N, cudaPtr, sphere, offset, dt);
    //MEASURE_TIME_END

}

__global__ void Ker_CudaSetForces(uint32_t N, glm::vec3* buffer, glm::vec3 v) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }
    buffer[i] = v;
}

void Parallel_CudaSetForces(const CudaResources& resources, glm::vec3 f) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 1024;
    Ker_CudaSetForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, resources.TotalForces, f);
    N = resources.ForcesPerMass*resources.MassCount;
    Ker_CudaSetForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, resources.Forces, glm::vec3(0));
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
    //printf("Force exerced on %i per %i is (%f, %f, %f)\n", springs[i].i, springs[i].j, (f*n).x, (f*n).y, (f*n).z);

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

void Parallel_CudaComputeSpringMassForces(const CudaResources& resources) {
    for (auto &springGroup: resources.SpringGroups) {
        uint32_t N = springGroup.springCount;
        uint32_t blockSize = 1024;
        Ker_CudaComputeSpringMassForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, springGroup.params,
                                                                                        springGroup.springs,
                                                                                        resources.Positions,
                                                                                        resources.Velocities,
                                                                                        resources.Forces,
                                                                                        resources.ForcesPerMass);
        N = resources.MassCount;
        Ker_ReduceForces<<<(N + blockSize - 1) / blockSize, blockSize>>>(N, resources.TotalForces, resources.Forces, resources.ForcesPerMass);
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
    if (Positions[i].y < -1.f) {
        Velocities[i].y = (-1.f-Positions[i].y)/dt;
    }
}


void Parallel_CudaMoveMassesKernel(const CudaResources& resources, float dt, float vertexMass) {
    uint32_t N = resources.MassCount;
    uint32_t blockSize = 1024;
    Ker_CudaMoveMasses<<<(N + blockSize - 1) / blockSize, blockSize>>>(
            N,
            resources.Positions,
            resources.Velocities,
            resources.TotalForces,
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


void Parallel_CudaApplyConstraints (const CudaResources& resources, float dt) {
    uint32_t blockSize = 1024;
    for (auto constaintSet: resources.ConstraintSets) {
        uint32_t N = constaintSet.size;
        Ker_ApplyConstraints<<<(N + blockSize - 1) / blockSize, blockSize>>>(
                N,
                constaintSet,
                resources.Velocities,
                resources.TotalForces,
                dt,
                constaintSet.freq
        );
    }
}


#endif
