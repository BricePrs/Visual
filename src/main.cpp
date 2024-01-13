#include <iostream>
#include <ProjectIncludes.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include "DoublePendulum.h"
#include "Mesh.h"
#include "Scene.h"
#include "InputManager.h"
#include "AnimatedMesh.h"
#include "AnimatedJoint.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>
#include <joint.h>


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
            {{0., 0., 0.}, {1., 0.3, .4}},
            {{1., 0., 0.}, {1., 0.3, .4}},
            {{1., 1., 0.}, {1., 0.3, .4}},
            {{0., 1., 0.}, {1., 0.3, .4}}};

    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};
    Mesh mesh = {vertices, indices, true};
    Mesh mesh2 = {vertices, indices, true};
    mesh.SetScale(glm::vec3(1., 0.5, 1.));
    mesh.SetPosition(glm::vec3(-8.3, 0., 9.8));
    mesh2.SetPosition(glm::vec3(-8.3, 0.5, 9.8));
    mesh2.SetScale(glm::vec3(1., .5, 1.));
    mesh2.SetRotation(glm::vec3(3.1415/2., 0., 0.));


    Tube tube = {0.3, 3., 10};
    Sphere sp = {0.9, 10};
    sp.Translate(glm::vec3(-1.));

    auto grid = GraphGrid(100, 1);

    AnimatedJoint::ARROW_SIZE = 0.15f;

    AnimatedMesh animatedMesh = { "bvh/walkSit.bvh", "bvh/skin.off", "bvh/weights.txt" };

    AnimatedJoint animatedJointRoot = AnimatedJoint                                 ("AnimatedData/PELV.txt", glm::vec3(-4., 4., 1.), "Pelv"); // X = Back Z = Up, Y = Right
    std::shared_ptr<AnimatedJoint> animatedJoint1 = animatedJointRoot.AddChildren   ("AnimatedData/UARML.txt", glm::vec3(0., -0.2, 0.6), "UArmL");
    std::shared_ptr<AnimatedJoint> animatedJoint2 = animatedJointRoot.AddChildren   ("AnimatedData/UARMR.txt", glm::vec3(0., 0.2, 0.6), "UArmR");
    std::shared_ptr<AnimatedJoint> animatedJoint3 = animatedJoint1->AddChildren     ("AnimatedData/FARML.txt", glm::vec3(0., 0., .3), "FArmL");
    std::shared_ptr<AnimatedJoint> animatedJoint4 = animatedJoint2->AddChildren     ("AnimatedData/FARMR.txt", glm::vec3(0., 0., .3), "FArmR");
    animatedJoint3->SetEnd(glm::vec3(0., 0., .3));
    animatedJoint4->SetEnd(glm::vec3(0., 0., .3));

    auto a = Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.));
    Scene world;
    world.AddObject(&grid);
    world.AddObject(&a);
    world.AddObject(&mesh);
    world.AddObject(&mesh2);
    world.AddObject(&animatedMesh);

    world.AddObject(&animatedJointRoot);

    InputManager inputManager(window, world);
    while (!glfwWindowShouldClose(window)) {
        auto startFrameTime = std::chrono::high_resolution_clock::now();
        glClearColor(.08, .05, 0.05, 1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x00);

        static auto StartTime = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(time-StartTime).count();

        animatedMesh.Update(elapsed);

        animatedJointRoot.BuildMesh();
        animatedJointRoot.Update(elapsed);

        world.Draw(inputManager.GetCamera());

        glfwPollEvents();
        inputManager.ProcessInputs();
        glfwSwapBuffers(window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1));
    }

    animatedJointRoot.ExportBVH("testExportLN.bvh");

    glfwTerminate();

    return EXIT_SUCCESS;
}
