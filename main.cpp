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
#include "Grid.h"
#include "RayTracingCamera.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>
#include <joint.h>

#include "Application.h"


int main() {


    Application application = {};


    std::vector<SimpleColorVertex> vertices= {
            {{0., 0., 0.}, {1., 0.3, .4}},
            {{1., 0., 0.}, {1., 0.3, .4}},
            {{1., 1., 0.}, {1., 0.3, .4}},
            {{0., 1., 0.}, {1., 0.3, .4}}};

    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};
    
    auto mesh = std::make_shared<Mesh<SimpleColorVertex>>(vertices, indices, true);
    auto mesh2 = std::make_shared<Mesh<SimpleColorVertex>>(vertices, indices, true);

    mesh->SetScale(glm::vec3(1., 0.5, 1.));
    mesh->SetPosition(glm::vec3(-8.3, 0., 9.8));
    mesh2->SetPosition(glm::vec3(-8.3, 0.5, 9.8));
    mesh2->SetScale(glm::vec3(1., .5, 1.));
    mesh2->SetRotation(glm::vec3(3.1415/2., 0., 0.));


    Tube tube = {0.3, 3., 10};
    Sphere sp = {0.9, 10};
    sp.Translate(glm::vec3(-1.));

    auto grid = GraphGrid(100, 1);


    AnimatedJoint::ARROW_SIZE = 0.15f;

    AnimatedMesh animatedMesh = { "bvh/walkSit.bvh", "bvh/skin.off", "bvh/weights.txt" };
    auto skinMesh = std::make_shared<Mesh<SimpleColorVertex>>(ParseOFF("bvh/skin.off"));
    skinMesh->SetScale(glm::vec3(0.01f));
    skinMesh->SetDrawMode(GL_LINE);

    AnimatedJoint animatedJointRoot = AnimatedJoint("AnimatedData/PELV.txt", glm::vec3(-4., 4., 1.), "Pelv"); // X = Back Z = Up, Y = Right
    std::shared_ptr<AnimatedJoint> animatedJoint1 = animatedJointRoot.AddChildren   ("AnimatedData/UARML.txt", glm::vec3(0., -0.2, 0.6), "UArmL");
    std::shared_ptr<AnimatedJoint> animatedJoint2 = animatedJointRoot.AddChildren   ("AnimatedData/UARMR.txt", glm::vec3(0., 0.2, 0.6), "UArmR");
    std::shared_ptr<AnimatedJoint> animatedJoint3 = animatedJoint1->AddChildren     ("AnimatedData/FARML.txt", glm::vec3(0., 0., .3), "FArmL");
    std::shared_ptr<AnimatedJoint> animatedJoint4 = animatedJoint2->AddChildren     ("AnimatedData/FARMR.txt", glm::vec3(0., 0., .3), "FArmR");
    animatedJoint3->SetEnd(glm::vec3(0., 0., .3));
    animatedJoint4->SetEnd(glm::vec3(0., 0., .3));


    Scene world;

    auto defaultShader          = world.AddShader("default.vsh", "default.fsh");
    auto defaultColorShader     = world.AddShader("defaultVertexColor.vsh", "defaultVertexColor.fsh");
    auto defaultTextureShader   = world.AddShader("default_texture.vsh", "default_texture.fsh");
    auto defaultNormalShader    = world.AddShader("defaultVertexNormal.vsh", "defaultVertexNormal.fsh");

    Mesh<SimpleVertex>::MESH_SHADER         = std::make_optional(world.GetShader(defaultShader));
    Mesh<SimpleColorVertex>::MESH_SHADER    = std::make_optional(world.GetShader(defaultColorShader));
    Mesh<SimpleNormalVertex>::MESH_SHADER   = std::make_optional(world.GetShader(defaultNormalShader));
    Mesh<SimpleUvVertex>::MESH_SHADER       = std::make_optional(world.GetShader(defaultTextureShader));


    world.AddObject(std::make_shared<GraphGrid>(grid), defaultColorShader);
    world.AddObject(mesh, defaultShader);
    world.AddObject(mesh2, defaultShader);
    world.AddObject(std::make_shared<AnimatedMesh>(animatedMesh), defaultShader);
    // world.AddObject(&skinMesh);

    world.AddObject(std::make_shared<AnimatedJoint>(animatedJointRoot), defaultNormalShader);

    Camera camera = Camera(ASPECT);

    InputManager inputManager(window, world, &camera);
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

    application.Run();



    return EXIT_SUCCESS;
}
