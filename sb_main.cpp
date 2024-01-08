#include <iostream>
#include <ProjectIncludes.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <implot/implot.h>
#include "DoublePendulum.h"
#include "Mesh.h"
#include "Scene.h"
#include "InputManager.h"
#include "Grid.h"
#include "RayTracingCamera.h"
#include "SoftBody.h"
#include "Octree.h"
#include "SignedDistanceField.h"
#include <GlobalVar.h>
#include <ParticleSystem.h>
#include <joint.h>
#include <main.cuh>
#include <glm/gtc/type_ptr.hpp>

static bool USE_CUDA = false;
static bool USE_RK4 = true;

void DrawSBWindow(std::shared_ptr<SoftBody> &sb, ObjectId id, Scene &world) {

    static int sbDim[3] = {10, 10, 10};

    ImGui::Begin("Menu");

    ImGui::Checkbox("Use CUDA", &USE_CUDA);
    ImGui::Checkbox("Use RK4", &USE_RK4);

    sb->DrawWindow();

    ImGui::SliderFloat3("Boundary sides", glm::value_ptr(sb->GetParams()->boundaryBox->GetSides()), 0., 30.);
    ImGui::SliderFloat3("Boundary Center", glm::value_ptr(sb->GetParams()->boundaryBox->GetCenter()), 0., 30.);
    sb->GetParams()->boundaryBox->UpdateBox(sb->GetParams()->boundaryBox->GetCenter(), sb->GetParams()->boundaryBox->GetSides());

    ImGui::SliderInt3("Rectangle Dimensions", sbDim, 1, 256);
    if (ImGui::Button("Restart sim", ImVec2(150, 30))) {
        std::shared_ptr<SimulationParams> params = sb->GetParams();
        params->physicsParams.dt = 0;
        params->physicsParams.t = 0;
        sb = SoftBody::Cube(sbDim[0], sbDim[1], sbDim[2], params->physicsParams, SoftBody::Hull, params);
        world.SetObject(id, sb);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset sim", ImVec2(150, 30))) {
        PhysicsParam params {-0.0, 0.00};
        auto boundary = sb->GetParams()->boundaryBox;
        boundary->UpdateBox(glm::vec3(0., 10., 0.), glm::vec3(10.));
        sb = SoftBody::Cube(sbDim[0], sbDim[1], sbDim[2], params, SoftBody::Hull);
        sb->GetParams()->boundaryBox = boundary;
        world.SetObject(id, sb);
    }
    ImGui::SameLine();
    if (ImGui::Button("Pause sim", ImVec2(150, 30))) {
        sb->TogglePause();
    }
    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::End();
}


void SetUpImguiStyle() {// Setup Dear ImGui style
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
}

void SetUpImgui(GLFWwindow *window) {// Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Keyboard Controls

    SetUpImguiStyle();

    const char* glsl_version = "#version 430 core";
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

GLFWwindow *SetUpGLFW() {
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
    return window;
}

int main() {

    GLFWwindow *window = SetUpGLFW();


    auto arrowx = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(1., 0., 0.)*0.6f, glm::vec3(1., 0., 0.));
    auto arrowy = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.));
    auto arrowz = IArrow3D(glm::vec3(0., 2., 4.), glm::vec3(0., 0., 1.)*0.6f, glm::vec3(0., 0., 1.));



    //std::shared_ptr<Octree> octree = std::make_shared<Octree>(4, glm::vec3(-10.), glm::vec3(10.), true);

    //auto armadillo = std::make_shared<Mesh<SimpleVertex>>(Mesh<SimpleVertex>::LoadFromPLY("models/Armadillo_LR.ply", 0.1f));
    //armadillo->SetScale(glm::vec3(0.1f, 0.1f, 0.1f));
    //armadillo->SetDrawMode(GL_LINE);


    Scene world;

    auto defaultShader          = world.AddShader("default.vsh", "default.fsh");
    auto defaultColorShader     = world.AddShader("defaultVertexColor.vsh", "defaultVertexColor.fsh");
    auto defaultTextureShader   = world.AddShader("default_texture.vsh", "default_texture.fsh");
    auto defaultNormalShader    = world.AddShader("defaultVertexNormal.vsh", "defaultVertexNormal.fsh");

    Mesh<SimpleVertex>::MESH_SHADER         = std::make_optional(world.GetShader(defaultShader));
    Mesh<SimpleColorVertex>::MESH_SHADER    = std::make_optional(world.GetShader(defaultColorShader));
    Mesh<SimpleNormalVertex>::MESH_SHADER   = std::make_optional(world.GetShader(defaultNormalShader));
    Mesh<SimpleUvVertex>::MESH_SHADER       = std::make_optional(world.GetShader(defaultTextureShader));

    std::shared_ptr<WireframeBox> boundaryBox = std::make_shared<WireframeBox>(
                    glm::vec3(0., 10., 0.),
                    glm::vec3(10.),
                    glm::vec3(1., .5, .3)
                );

    PhysicsParam params {-0.0, 0.00};
    std::shared_ptr<SignedDistanceField> sdf = std::make_shared<SignedDistanceField>(20, 8.);

    //sdf->AddSphere(glm::vec3(0.), 2.);
    //sdf->AddSphere(glm::vec3(1.5), 1.);

    sdf->AddCapsule(glm::vec3(0., 0., .0), glm::vec3(1., 0., 0.2), 3., 1.);
    sdf->AddCapsule(glm::vec3(-2.6, 0., .0), glm::vec3(0., 1., 0.), 4., 1.3);
    sdf->AddCapsule(glm::vec3(2.6, 0., .0), glm::vec3(0., 1., 0.), 4., 1.3);

    //sdf->BuildMesh();
    //auto sbp = SoftBody::Cube(10, 10, 10, params, SoftBody::Hull);
    auto sbp = sdf->BuildSoftbody(params, SoftBody::Structure);
    //auto sbp = SoftBody::Torus(10, 10, 10, params, SoftBody::Hull);
    //auto sb = SoftBody2D::Rectangle(10, 20, params);

    auto grid = std::make_shared<GraphGrid>(GraphGrid(100, 1));

    sbp->GetParams()->boundaryBox = boundaryBox;


    //world.AddObject(&arrowx);
    //world.AddObject(&arrowy);
    //world.AddObject(&arrowz);
    world.AddObject(grid, defaultColorShader);
    //world.AddObject(sdf, defaultColorShader);
    world.AddObject(boundaryBox, defaultShader);
    //world.AddObject(sdf, defaultColorShader);
    //world.AddObject(armadillo, defaultShader);


    auto sbId = world.AddObject(sbp, defaultNormalShader);


    auto camera = new PerspectiveCamera(ASPECT);
    camera->translate({5., 6.+5., -18.});

    // --- ImGui Set Up --- //

    SetUpImgui(window);

    InputManager inputManager(window, world, camera);
    while (!glfwWindowShouldClose(window)) {

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        auto startFrameTime = std::chrono::high_resolution_clock::now();
        glClearColor(.08f, .05f, 0.05f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x00);

        static auto StartTime = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::high_resolution_clock::now();

        if (USE_CUDA && USE_RK4) {
            sbp->CudaRkPhysicsStep(world.GetColliderObjects());
        } else if (USE_CUDA && !USE_RK4) {
            sbp->CudaPhysicsStep(world.GetColliderObjects());
        } else if (USE_RK4) {
            sbp->RkPhysicsStep(world.GetColliderObjects());
        } else {
            sbp->PhysicsStep(world.GetColliderObjects());
        }

        DrawSBWindow(sbp, sbId, world);

        world.Draw(inputManager.GetCamera());


        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwPollEvents();
        inputManager.ProcessInputs();
        glfwSwapBuffers(window);
        std::this_thread::sleep_until(startFrameTime+std::chrono::duration<double, std::ratio<1, 300>>(1));
    }

    sbp->PlotEnergy();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwTerminate();

    return EXIT_SUCCESS;
}
