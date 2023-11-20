//
// Created by brice on 11/10/23.
//

#include "DoublePendulum.h"
#include <glad/glad.h>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <glm/gtc/matrix_transform.hpp>
#include <Mesh.h>

void DoublePendulum::InitOpenGl() {

    for (int i = 0; i < 3; ++i) {
        spheres.emplace_back(0.2, 10);
    }

    spheres[0].SetPosition({ogn, 0.});

    tubes.emplace_back(.02, l1, 10);
    tubes.emplace_back(.02, l2, 10);

}

void DoublePendulum::Draw(const PerspectiveCamera &camera) {
    UpdateMesh();
    for (auto & sphere : spheres) {
        sphere.Draw(camera);
    }
    for (auto & tube : tubes) {
        tube.Draw(camera);
    }
}

void DoublePendulum::Step() {
    auto C = thp1*thp1*l1/l2*glm::sin(th1-th2)-9.81/l2*glm::sin(th2);
    auto D = l1/l2*glm::cos(th1-th2);
    auto A = -glm::sin(th1)*9.81/l1-l2/l1*m2/(m1+m2)*glm::sin(th1-th2)*thp2*thp2;
    auto B = l2/l1*m2/(m1+m2)*glm::cos(th1-th2);

    auto pthp1 = thp1;
    auto pthp2 = thp2;

    thp1 += dt * (A-B*C)/(1.-B*D);
    thp2 += dt * (C - D * thp1);

    th1 += dt * thp1;
    th2 += dt * thp2;
}

void DoublePendulum::Step2() {

    auto A = (m1+m2)*l1*l1;
    auto B = m2*l1*l2*glm::cos(th2);
    auto C = -9.81*(m1*l1*glm::sin(th1)+m2*l2*glm::sin(th1+th2));

    auto D = B;
    auto E = m2*l2*l2;
    auto F = -thp1*thp2*l1*l2*glm::sin(th2)-m2*9.81*l2*glm::cos(th1+th2);

    auto pthp1 = thp1;
    auto pthp2 = thp2;

    thp1 += dt * (C-B/E*F)/(A-B*D/E);
    thp2 += dt * (F-D*thp1)/E;

    th1 += dt * thp1;
    th2 += dt * thp2;
}

void DoublePendulum::UpdateMesh() {
    auto p0 = glm::dvec3(ogn, 0.);
    auto p1 = p0 + this->l1 * glm::dvec3(glm::sin(th1), -glm::cos(th1), 0.);
    auto p2 = p1 + this->l2 * glm::dvec3(glm::sin(th1+th2), -glm::cos(th1+th2), 0.);

    spheres[1].SetPosition(p1);
    spheres[2].SetPosition(p2);
    tubes[0].SetRotation({0., 0., 3.141593+th1});
    tubes[1].SetRotation({0., 0., 3.141593+th1+th2});
    tubes[0].SetPosition(p0);
    tubes[1].SetPosition(p1);
}