#ifndef VISUAL_SOFTBODY_CUH
#define VISUAL_SOFTBODY_CUH

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include "Mesh.h"
#include "SoftBody2D.h"

void ParallelSolveCollision(Mesh<SimpleColorVertex>* mesh, glm::vec4 sphere, glm::vec3 offset, float dt);

void Parallel_CudaSetForces(const CudaResources& resources, glm::vec3 f);
void Parallel_CudaComputeSpringMassForces(const CudaResources& resources);
void Parallel_CudaMoveMassesKernel(const CudaResources& resources, float dt, float vertexMass);
void Parallel_CudaApplyConstraints (const CudaResources& resources, float dt);

#endif