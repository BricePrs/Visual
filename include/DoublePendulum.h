//
// Created by brice on 11/10/23.
//

#ifndef VISUAL_DOUBLEPENDULUM_H
#define VISUAL_DOUBLEPENDULUM_H

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Shader.h"
#include "PerspectiveCamera.h"
#include "Mesh.h"

class DoublePendulum : public Drawable {
public:

    DoublePendulum(glm::dvec2 ogn, double th1, double th2, double thp1, double thp2, double l1, double l2, double m1, double m2, double dt)
        : ogn(ogn), th1(th1), th2(th2), thp1(thp1), thp2(thp2), l1(l1), l2(l2), m1(m1), m2(m2), dt(dt) {
        this->InitOpenGl();
    };

    DoublePendulum(glm::dvec2 ogn, double th1, double th2, double l, double m, double dt)
        : DoublePendulum(ogn, th1, th2, 0., 0., l, l, m, m, dt) {};

    void Step();
    void Step2();
    void Draw(const PerspectiveCamera &camera);

private:

    void InitOpenGl();
    void UpdateMesh();

    // Physic sim parameters
    glm::dvec2 ogn;
    double th1;
    double th2;
    double thp1;
    double thp2;
    double l1;
    double l2;
    double m1;
    double m2;
    double dt;

    // OpenGl Param
    std::vector<Sphere> spheres;
    std::vector<Tube> tubes;

};


#endif //VISUAL_DOUBLEPENDULUM_H
