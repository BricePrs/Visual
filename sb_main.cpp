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
#include "Grid.h"
#include "RayTracingCamera.h"
#include "SoftBody2D.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>
#include <joint.h>
#include <main.cuh>

static bool USE_CUDA = false;
static bool USE_RK4 = true;

void DrawSBWindow(SoftBody2D& sb, uint32_t id, Scene &world) {

    static int sbDim[3] = {10, 10, 10};
    bool shouldReset = false;

    ImGui::Begin("Menu");

    ImGui::Checkbox("Use CUDA", &USE_CUDA);
    ImGui::Checkbox("Use RK4", &USE_RK4);

    sb.DrawWindow();

    ImGui::SliderInt3("Rectangle Dimensions", sbDim, 1, 50);
    shouldReset = ImGui::Button("Restart sim", ImVec2(150, 30));

    if (shouldReset) {
        PhysicsParam params {-0.0, 0.00};
        sb = SoftBody2D::Cube(sbDim[0], sbDim[1], sbDim[2], params, SoftBody2D::Structure);
        world.GetObjects()[id] = &sb;
    }

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}


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


    auto arrowx = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(1., 0., 0.)*0.6f, glm::vec3(1., 0., 0.));
    auto arrowy = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.));
    auto arrowz = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 0., 1.)*0.6f, glm::vec3(0., 0., 1.));

    Quad quad = Quad();

    quad.Translate({0., 10., 15.});
    quad.SetScale(glm::vec3(20.f));
    PhysicsParam params {-0.0, 0.00};
    auto sb = SoftBody2D::Cube(10, 25, 10, params, SoftBody2D::Structure);
    //auto sb = SoftBody2D::Rectangle(10, 20, params);

    auto grid = GraphGrid(100, 1);

    Scene world;
    //world.AddObject(&arrowx);
    //world.AddObject(&arrowy);
    //world.AddObject(&arrowz);
    world.AddObject(&grid);

    world.AddObject(&quad);
    auto sbId = world.AddObject(&sb);

    auto camera = new PerspectiveCamera(ASPECT);
    camera->translate({5., 6.+5., -18.});

    // --- ImGui Set Up --- //

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_CheckMark, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(.5, 0.2, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(.9, 0.7, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(.2, 0.1, 0.05, 1.));

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(.5, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(.3, 0.1, 0.05, 1.));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(.5, 0.2, 0.05, 1.));

    ImGui::GetStyle().FrameRounding = 4;
    ImGui::GetStyle().ChildRounding = 4;
    ImGui::GetStyle().GrabRounding = 4;
    ImGui::GetStyle().WindowRounding = 4;

    const char* glsl_version = "#version 430 core";
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);



    InputManager inputManager(window, world, camera);
    while (!glfwWindowShouldClose(window)) {

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        auto startFrameTime = std::chrono::high_resolution_clock::now();
        glClearColor(.08, .05, 0.05, 1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x00);

        static auto StartTime = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::high_resolution_clock::now();

        if (USE_CUDA) {
            sb.CudaPhysicsStep(world.GetColliderObjects());
        } else if (USE_RK4) {
            sb.RkPhysicsStep(world.GetColliderObjects());
        } else {
            sb.PhysicsStep(world.GetColliderObjects());
        }

        DrawSBWindow(sb, sbId, world);

        world.Draw(inputManager.GetCamera());


        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwPollEvents();
        inputManager.ProcessInputs();
        glfwSwapBuffers(window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1));
    }


    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();



    glfwTerminate();

    return EXIT_SUCCESS;
}
