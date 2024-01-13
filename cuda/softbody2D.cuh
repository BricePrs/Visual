#ifndef VISUAL_SOFTBODY_CUH
#define VISUAL_SOFTBODY_CUH

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include "Mesh.h"
#include "SoftBody.h"

void ParallelSolveCollision                 (Mesh<SimpleColorVertex>* mesh, glm::vec4 sphere, glm::vec3 offset, float dt);

void Parallel_CudaSetForces                 (const CudaResources& resources, CudaODESystem &odeSystem, glm::vec3 f);
void Parallel_CudaComputeSpringMassForces   (const CudaResources& resources, CudaODESystem &odeSystem);
void Parallel_CudaMoveMassesKernel          (const CudaResources& resources, CudaODESystem &odeSystem, float dt, float vertexMass);
void Parallel_CudaApplyConstraints          (const CudaResources& resources, CudaODESystem &odeSystem, float dt);
void ParallelSolveCollision                 (const CudaResources& resources, CudaODESystem &odeSystem, const std::shared_ptr<SimulationParams> params);
void ParallelSolveSphereCollision           (const CudaResources& resources, CudaODESystem &odeSystem, glm::vec4 sphere);


// -- RK4 Func -- //

void Parallel_CudaInitIter                  (CudaResources& resources);
void Parallel_OffsetODESystem               (const CudaResources& resources, CudaODESystem &odeSystem, const CudaODESystem &offBuffer, float stepSize, bool offset);
void Parallel_CudaApplyRkCoefs              (CudaResources& resources, float dt, float mass);


float Parallel_ComputeKineticEnergy         (CudaResources &resources, float mass);
float Parallel_ComputePotentialEnergy       (CudaResources &resources, float mass, float gravityConstant);
float Parallel_ComputeSpringsEnergy         (CudaResources& resources, int i, float mass);


#endif