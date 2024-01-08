//
// Created by brice on 11/12/23.
//

#ifndef VISUAL_SHADER_H
#define VISUAL_SHADER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <string>
#include <glm/glm.hpp>
#include <iostream>

class Shader {
public:

    Shader() : mProgramID(0) {}

    Shader(const std::string &vertexName, const std::string &fragmentName);
    ~Shader() {
        if (mRefCounter != nullptr && mRefCounter.unique()) {
            glDeleteProgram(mProgramID);
            std::cout << "Deleting program" << std::endl;
        }
    };

    void use() const;

    bool setInt(const std::string &name, int32_t value);
    bool setFloat(const std::string &name, float value);
    bool setVec3(const std::string &name, glm::vec3 value);
    bool setVec4(const std::string &name, glm::vec4 value);
    bool setMat4(const std::string &name, glm::mat4 value);
    bool setBool(const std::string &name, bool value);

private:

    static std::string readShaderSource(const std::string &shaderPath);
    static GLuint compileProgram(const std::string &vertexName, const std::string &fragmentName);
    static GLuint compileShader(GLuint type, const char *source);   // TODO return optional

protected:
    GLuint mProgramID;
    std::shared_ptr<int> mRefCounter;

};


class ComputeShader : public Shader {
public:

    ComputeShader() = default;
    ComputeShader(const std::string &computeName);

private:

    static std::string readComputeSource(const std::string &shaderPath);
    static GLuint compileComputeProgram(const std::string &computeName);
    static GLuint compileComputeShader(GLuint type, const char *source);

};

#endif //VISUAL_SHADER_H
