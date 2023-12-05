//
// Created by brice on 11/29/23.
//

#ifndef VISUAL_COLLIDER_H
#define VISUAL_COLLIDER_H

#include <glm/glm.hpp>

class Collider {
public:
    [[nodiscard]] virtual bool Intersect(glm::vec3 pt) const = 0;
    [[nodiscard]] virtual bool Intersect(glm::vec3 LowerAABB, glm::vec3 UpperAABB) const = 0;
    [[nodiscard]] virtual glm::vec3 ShortestSurfacePoint(glm::vec3 pt) const = 0;
    [[nodiscard]] virtual glm::vec3 ComputeCollisionForce(glm::vec3 pt) const = 0;
    [[nodiscard]] virtual glm::vec4 GetSphere() const = 0;
};


#endif //VISUAL_COLLIDER_H