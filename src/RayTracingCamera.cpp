//
// Created by brice on 11/27/23.
//

#include <GlobalVar.h>
#include <iostream>
#include "RayTracingCamera.h"
#include "bvh.h"
#include "happly/happly.h"
#include <glm/gtc/type_ptr.hpp>
#include "imgui/imgui.h"


RayTracingCamera::RayTracingCamera(double aspect)
    : PerspectiveCamera(aspect),  mFramebufferTex(CAMERA_RES, CAMERA_RES, 4), mFramebufferId(0)
{
    glGenFramebuffers(1, &mFramebufferId);
    glBindFramebuffer(GL_FRAMEBUFFER, mFramebufferId);
    mFramebufferTex.Bind();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mFramebufferTex.GetOpenglId(), 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Could not create framebuffer");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    mShader = {"rtsia.comp"};

}

void RayTracingCamera::DrawScene(const PerspectiveCamera& camera) {

    mShader.use();
    if (mRtMesh) {
        mRtMesh->BindSSBO();
    }

    glBindImageTexture(2, mFramebufferTex.GetOpenglId(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    mFramebufferTex.Bind();

    // Binding Uniforms

    mShader.setFloat("groundDistance", mSPGroundDistance);
    mShader.setFloat("radius", mSPRadius);
    mShader.setMat4("mat_inverse", glm::inverse(camera.getViewMatrix()));
    mShader.setMat4("persp_inverse", glm::inverse(camera.getProjMatrix()));
    mShader.setVec3("lightPosition", mSPLightPosition);
    mShader.setFloat("blinnPhong", 0.);
    mShader.setBool("fastGammaCorrection", true);
    mShader.setFloat("lightIntensity", mSPLightIntensity);
    mShader.setFloat("shininess", mSPShininess);
    mShader.setBool("enableEnvMap", mSPEnvMap);
    mShader.setBool("resetAccumulation", mResetAccumulation);
    mShader.setInt("accumulationCount", static_cast<int32_t>(mAccumulationCount));
    mShader.setVec3("eta3dReal", mSPEta3dReal);
    mShader.setVec3("eta3dImag", mSPEta3dImag);
    mShader.setInt("colorTexture", 0);
    mShader.setFloat("tileScale", 1.);

    mShader.setInt("envMap", 1);
    glActiveTexture(GL_TEXTURE1);
    mEnvMap.Bind();

    uint32_t kernelSize = 8;
    uint32_t dispatchGroupX = (CAMERA_RES + kernelSize - 1) / kernelSize;
    uint32_t dispatchGroupY = (CAMERA_RES + kernelSize - 1) / kernelSize;
    uint32_t dispatchGroupZ = 1;
    glDispatchCompute(dispatchGroupX, dispatchGroupY, dispatchGroupZ);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    ++mAccumulationCount;
    mResetAccumulation = false;

}

void RayTracingCamera::SetMesh(RayTracingMesh *mesh) {
    int resolution = 40;
    float radius = 1.;

    std::vector<glm::vec4> vertices;
    for (int i = 0; i < resolution; i++) {
        double pitch = glm::radians(180.*static_cast<double>(i)/(resolution-1));
        for (int j = 0.; j < resolution; j++) {
            double yaw = glm::radians(360.*static_cast<double>(j)/resolution);
            vertices.emplace_back(
                    radius * glm::sin(yaw) * glm::sin(pitch),
                    radius * glm::cos(pitch),
                    radius * glm::cos(yaw) * glm::sin(pitch),
                    0.
            );
        }
    }

    std::vector<glm::vec4> normals;
    for (int i = 0; i < resolution; i++) {
        double pitch = glm::radians(180.*static_cast<double>(i)/(resolution-1));
        for (int j = 0.; j < resolution; j++) {
            double yaw = glm::radians(360.*static_cast<double>(j)/resolution);
            normals.emplace_back(
                    glm::sin(yaw) * glm::sin(pitch),
                    glm::cos(pitch),
                    glm::cos(yaw) * glm::sin(pitch),
                    0.
            );
        }
    }

    std::vector<uint32_t> indices;
    for (int i = 1.; i < resolution; i++) {
        for (int j = 0.; j < resolution; j++) {
            indices.emplace_back(i*resolution+j);
            indices.emplace_back((i-1)*resolution+(j+1)%resolution);
            indices.emplace_back((i-1)*resolution+j);

            indices.emplace_back((i-1)*resolution+(j+1)%resolution);
            indices.emplace_back(i*resolution+j);
            indices.emplace_back(i*resolution+(j+1)%resolution);
        }
    }

    std::vector<glm::vec4> colors;
    for (int i = 1.; i < resolution; i++) {
        for (int j = 0.; j < resolution; j++) {
            colors.emplace_back(0., 0., 1., 0.);
        }
    }

    BVH bvh{ vertices, indices };
    const std::vector<BVH::Node>& nodes = bvh.getNodes();
    const std::vector<Triangle>& triangles = bvh.getTriangles();

    mRtMesh = new RayTracingMesh(vertices, normals, colors, triangles, {}, nodes);

}

void RayTracingCamera::SetMesh(const std::string &fileName) {
    mRtMesh = RayTracingMesh::LoadFromPLY(fileName);
}

void RayTracingCamera::ResetAccumulation() {
    mAccumulationCount = 0;
    mResetAccumulation = true;
}

void RayTracingCamera::translate(glm::vec3 t) {
    PerspectiveCamera::translate(t*glm::vec3(-1, 1, 1));
    ResetAccumulation();
}

void RayTracingCamera::rotate(double dPitch, double dYaw) {
    PerspectiveCamera::rotate(dPitch, -dYaw);
    ResetAccumulation();
}

void RayTracingCamera::SetEnvMap(const std::string &fileName) {
    mEnvMap = Texture::LoadFromFile(fileName);
}

void RayTracingCamera::DrawWindow() {

    bool shouldReset = false;

    ImGui::Begin("Menu");

    shouldReset = shouldReset || ImGui::Checkbox("Blinn-Phong", &mSPBlinnPhong);
    shouldReset = shouldReset || ImGui::Checkbox("EnvMap", &mSPEnvMap);

    shouldReset = shouldReset || ImGui::ColorEdit3("Real eta", glm::value_ptr(mSPEta3dReal));
    shouldReset = shouldReset || ImGui::ColorEdit3("Imag eta", glm::value_ptr(mSPEta3dImag));

    shouldReset = shouldReset || ImGui::ColorEdit3("Light pos", glm::value_ptr(mSPLightPosition));

    shouldReset = shouldReset || ImGui::SliderFloat("Ground dist", &mSPGroundDistance, -10.0f, 10.0f);
    shouldReset = shouldReset || ImGui::SliderFloat("Radius", &mSPRadius, 0.0f, 10.0f);
    shouldReset = shouldReset || ImGui::SliderFloat("Light Intensity", &mSPLightIntensity, 0.0f, 5.0f);
    shouldReset = shouldReset || ImGui::SliderFloat("Shininess", &mSPShininess, 0.0f, 200.0f);


    ImGui::Text("Accumulated %i frames", mAccumulationCount);

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

    if (shouldReset) { ResetAccumulation(); }

}

RayTracingMesh::RayTracingMesh(const std::vector<glm::vec4> &vertices, const std::vector<glm::vec4> &normals,
                               const std::vector<glm::vec4> &colors, const std::vector<Triangle> &indices,
                               const std::vector<uint32_t> &texcoords, const std::vector<BVH::Node> &nodes)
{
    glGenBuffers(1, &mVertexBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mVertexBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, vertices.size()*sizeof(glm::vec4), vertices.data(), GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, mVertexBuffer);

    glGenBuffers(1, &mNormalBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mNormalBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, normals.size()*sizeof(glm::vec4), normals.data(), GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, mNormalBuffer);

    glGenBuffers(1, &mColorBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mColorBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, colors.size()*sizeof(glm::vec4), colors.data(), GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mColorBuffer);

/*
    glGenBuffers(1, &mTexcoordBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mTexcoordBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, texcoords.size()*sizeof(glm::vec2), texcoords.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(5);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void *)0);
*/


    glGenBuffers(1, &mIndexBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mIndexBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, indices.size()*sizeof(Triangle), indices.data(), GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, mIndexBuffer);

    glGenBuffers(1, &mNodesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mNodesBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, nodes.size()*sizeof(BVH::Node), nodes.data(), GL_STATIC_READ);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, mNodesBuffer);
}

void RayTracingMesh::BindSSBO() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mVertexBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, mVertexBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mNormalBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, mNormalBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mColorBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, mColorBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mIndexBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, mIndexBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mNodesBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, mNodesBuffer);
}

std::vector<glm::vec4> RayTracingMesh::ComputeNormals(const std::vector<glm::vec4> &vertices,
                                                      const std::vector<Triangle> &indices)
{
    std::vector<glm::vec4> normals(vertices.size());
    for (auto &tri: indices) {
        auto n = glm::vec4(glm::normalize(glm::cross(glm::vec3(vertices[tri.indices[1]]-vertices[tri.indices[0]]), glm::vec3(vertices[tri.indices[2]]-vertices[tri.indices[0]]))), 0.);
        normals[tri.indices[0]] += n;
        normals[tri.indices[1]] += n;
        normals[tri.indices[2]] += n;
    }
    for (auto &normal: normals) {
        normal = glm::normalize(normal);
    }
    return normals;
}


RayTracingMesh *RayTracingMesh::LoadFromPLY(const std::string &fileName) {
    auto parsedData = happly::PLYData(fileName);
    std::vector<std::array<double, 3>> parsedVertices = parsedData.getVertexPositions();
    std::vector<std::vector<size_t>> faceIndices = parsedData.getFaceIndices<size_t>();
    //std::vector<std::array<unsigned char, 3>> parsedColors = parsedData.getVertexColors();

    std::vector<glm::vec4> vertices; vertices.reserve(parsedVertices.size());
    for (auto & v : parsedVertices) {
        vertices.emplace_back(v[0], v[1], v[2], 0.);
    }

    std::vector<glm::vec4> colors; colors.reserve(parsedVertices.size());
    for (auto & v : parsedVertices) {
        colors.emplace_back(1., 1., 1., 0.);
    }

    std::vector<uint32_t> indices; indices.reserve(faceIndices.size()*3);
    for (auto &face: faceIndices) {
        assert(face.size() == 3);
        indices.emplace_back(face[0]);
        indices.emplace_back(face[1]);
        indices.emplace_back(face[2]);
    }


    BVH bvh{ vertices, indices };
    const std::vector<BVH::Node>& nodes = bvh.getNodes();
    const std::vector<Triangle>& triangles = bvh.getTriangles();

    auto normals = ComputeNormals(vertices, triangles);

    return new RayTracingMesh(vertices, normals, colors, triangles, {}, nodes);
}
