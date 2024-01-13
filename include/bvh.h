#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <vector>
#include "bbox.h"
#include "triangle.h"

class BVH {
public:
    struct Node {
        BBox bbox;
        /* nbTriangles == 0 if interior node */
        int nbTriangles;
        /* index == indexFirstTriangle if leaf node else indexRightNode */
        int index;
        int padding[2] = { 0, 0 };
    };

    BVH(const std::vector<glm::vec4>& vertices, const std::vector<uint32_t>& indices)
        : vertices{ vertices }
    {
        triangles.reserve(indices.size()/3);

        for (auto index = indices.begin(); index < indices.end(); index+=3) {
            triangles.emplace_back(*index, *(index+1), *(index+2));
        }

        build();
    }

    const std::vector<Node>& getNodes() const {
        return nodes;
    }

    const std::vector<Triangle>& getTriangles() const {
        return triangles;
    }

private:
    const std::vector<glm::vec4>& vertices;
    std::vector<Node> nodes;
    std::vector<Triangle> triangles;

    void build();
    int build(int start, int end);
};

#endif
