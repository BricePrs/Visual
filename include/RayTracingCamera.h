//
// Created by brice on 11/27/23.
//

#ifndef VISUAL_RAYTRACINGCAMERA_H
#define VISUAL_RAYTRACINGCAMERA_H


#include <array>
#include "glad/glad.h"
#include "Texture.h"
#include "Scene.h"
#include "Shader.h"
#include "bvh.h"

class RayTracingMesh {
public:

    RayTracingMesh(const std::vector<glm::vec4> &vertices,
                   const std::vector<glm::vec4> &normals,
                   const std::vector<glm::vec4> &colors,
                   const std::vector<Triangle> &indices,
                   const std::vector<uint32_t> &texcoords,
                   const std::vector<BVH::Node> &nodes);

    void BindSSBO();

    static RayTracingMesh* LoadFromPLY(const std::string &fileName);
    static std::vector<glm::vec4> ComputeNormals(const std::vector<glm::vec4> &vertices,
                                                 const std::vector<Triangle> &indices);
private:

    GLuint mVertexBuffer;
    GLuint mNormalBuffer;
    GLuint mIndexBuffer;
    GLuint mColorBuffer;
    //GLuint mTexcoordBuffer;
    GLuint mNodesBuffer;
};

class RayTracingCamera : public PerspectiveCamera {
public:

    explicit RayTracingCamera(double aspect);

    void DrawScene(const PerspectiveCamera& camera);

    [[nodiscard]] Texture GetTexture() const { return mFramebufferTex; };
    void SetMesh(RayTracingMesh *mesh);
    void SetMesh(const std::string &fileName);
    void ResetAccumulation();

    void translate(glm::vec3 t) override;
    void rotate(double dPitch, double dYaw) override;

    void SetEnvMap(const std::string& fileName);
    void SetGroundTex(const std::string& fileName);
    void SetSphereTex(const std::string& fileName);
    void SetSphereNormalMap(const std::string& fileName);

    void DrawWindow();

private:

    GLuint mFramebufferId;
    Texture mFramebufferTex;
    Texture mEnvMap;
    Texture mGroundTex;

    ComputeShader mShader;
    RayTracingMesh *mRtMesh;

    uint32_t mAccumulationCount = 0;
    bool mResetAccumulation = true;

    static inline uint32_t CAMERA_RES = 600;


    // --- Scene params --- //
    float mSPShininess = 200.;
    float mSPGroundDistance = 1.;
    float mSPRadius = 10.;
    bool mSPBlinnPhong = false;
    bool mSPEnvMap = true;
    bool mSPCheckboard = true;
    bool mSPTransparent = true;
    bool mSPSphereModel = true;
    float mSPLightIntensity = 1.;
    glm::vec3 mSPLightPosition = glm::vec3(10.f);
    glm::vec3 mSPEta3dReal = glm::vec3(0.f, 0., 1.);
    glm::vec3 mSPEta3dImag = glm::vec3(0.f, 0., 0.);
    glm::vec4 mSPObjectColor = glm::vec4(0.7);
    float mGlassIndex = 1.5;

};


#endif //VISUAL_RAYTRACINGCAMERA_H
