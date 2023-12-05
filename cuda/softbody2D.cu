#ifndef VISUAL_SOFTBODY_CU
#define VISUAL_SOFTBODY_CU

#include <iostream>
#include <vector>
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>
#include "main.cuh"

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

__global__ void PointSphereCollision(uint32_t N, glm::vec3 *pos, glm::vec3 *vel, glm::vec4 sphere, glm::vec3 offset, float dt)
{
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if (i >= N) { return; }

    auto a = pos[i]+offset-glm::vec3(sphere);
    float sideDistSq = glm::dot(a, a)-sphere.a*sphere.a;
    if (sideDistSq > 0.) {
        return;
    }
    glm::vec3 path = glm::normalize(a)*std::sqrt(-sideDistSq);

    pos[i] += path;
    vel[i] = path/dt; // TODO : not working for many colliders
}

__global__ void Init(uint32_t N, float*v) {
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    v[i] = 42.;
}

void ParallelSolveCollision(std::vector<glm::vec3> &positions, std::vector<glm::vec3> &velocities, glm::vec4 sphere, glm::vec3 offset, float dt)
{
    glm::vec3 *d_pos, *d_vel;
    uint32_t N = positions.size();

    cudaMalloc(&d_pos, positions.size() * sizeof(glm::vec3));
    cudaMalloc(&d_vel, velocities.size() * sizeof(glm::vec3));

    cudaMemcpy(d_pos, positions.data(), positions.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vel, velocities.data(), velocities.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);


    CUDA_CHECK_ERROR(cudaMemcpy(positions.data(), d_pos, positions.size() * sizeof(glm::vec3), cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(velocities.data(), d_vel, velocities.size() * sizeof(glm::vec3), cudaMemcpyDeviceToHost));

    uint32_t blockSize = 1024;
    PointSphereCollision<<<(N+blockSize-1)/blockSize, blockSize>>>(N, d_pos, d_vel, sphere, offset, dt);

    CUDA_CHECK_ERROR(cudaMemcpy(positions.data(), d_pos, positions.size() * sizeof(glm::vec3), cudaMemcpyDeviceToHost));
    CUDA_CHECK_ERROR(cudaMemcpy(velocities.data(), d_vel, velocities.size() * sizeof(glm::vec3), cudaMemcpyDeviceToHost));

    cudaFree(d_pos);
    cudaFree(d_vel);
}


#endif
