#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <thread>
#include <chrono>
#include "glm/vec2.hpp"
#include "DoublePendulum.h"
#include "Mesh.h"
#include "Scene.h"
#include "InputManager.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>


int main() {

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 16);

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Visual", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create window !");
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        glfwTerminate();
        throw std::runtime_error("Failed to get proc address !");
    }

    glfwSwapInterval(0);


    std::vector<SimpleColorVertex> vertices= {
            {{0., 0., 2.}, {1., 0.3, .4}},
            {{1., 0., 2.}, {1., 0.3, .4}},
            {{1., 1., 2.}, {1., 0.3, .4}},
            {{0., 1., 2.}, {1., 0.3, .4}}};

    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};
    Mesh mesh = {vertices, indices, true};
    Tube tube = {0.3, 3., 10};
    Sphere sp = {0.9, 10};
    sp.Translate(glm::vec3(-1.));

    double dt = 0.0005;
    //auto Pendulum = DoublePendulum({0., 0.0}, 3.141592*0.9, -3.141592*0.1, 2., 1., dt);
    auto Pendulum2 = DoublePendulum({0., 0.0}, 3.141592*0.9, -3.141592*0.1, 2., 1., dt);
    auto grid = Grid(100, 1);

    auto ps = ParticleSystem(glm::vec3(10., 0., 0.), glm::vec3(3., 2., 4.), 0.4, 100);
    auto arrowx = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(1., 0., 0.)*0.6f, glm::vec3(1., 0., 0.));
    auto arrowy = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.));
    auto arrowz = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 0., 1.)*0.6f, glm::vec3(0., 0., 1.));
    Scene world;
    world.AddObject(&grid);
    world.AddObject(&Pendulum2);
    world.AddObject(&mesh);
    world.AddObject(&ps);
    world.AddObject(&arrowx);
    world.AddObject(&arrowy);
    world.AddObject(&arrowz);

    InputManager inputManager(window, world);

    while (!glfwWindowShouldClose(window)) {
        auto startFrameTime = std::chrono::high_resolution_clock::now();
        glClearColor(.08, .05, 0.05, 1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x00);

        int32_t stepCount = static_cast<int32_t>(1./60./dt*.01);
        for (int i = 0; i < stepCount; ++i) {
            Pendulum2.Step2();
        }

        world.Draw(inputManager.GetCamera());

        glfwPollEvents();
        inputManager.ProcessInputs();
        glfwSwapBuffers(window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1));
    }

    glfwTerminate();

    return EXIT_SUCCESS;
}
