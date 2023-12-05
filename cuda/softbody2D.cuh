#ifndef VISUAL_SOFTBODY_CUH
#define VISUAL_SOFTBODY_CUH

#include <iostream>
#include <vector>
#include <glm/glm.hpp>

void ParallelSolveCollision(std::vector<glm::vec3> &positions, std::vector<glm::vec3> &velocities, glm::vec4 sphere, glm::vec3 offset, float dt);

#endif