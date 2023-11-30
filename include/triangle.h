#ifndef BVH_TRIANGLE_HPP
#define BVH_TRIANGLE_HPP

#include <vector>
#include "bbox.h"

class Triangle {
public:
    int indices[3];

    Triangle(int a, int b, int c)
        : indices{ a, b, c }
    {
    }

    BBox bbox(const std::vector<glm::vec4>& vertices) const
    {
        BBox bbox{ vertices[indices[0]] };
        bbox.expand(vertices[indices[1]]);
        bbox.expand(vertices[indices[2]]);
        return bbox;
    }
};

#endif
