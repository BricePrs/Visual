//
// Created by brice on 11/12/23.
//

#include <sstream>
#include <fstream>
#include <iostream>
#include "Shader.h"
#include <glm/gtc/type_ptr.hpp>


GLuint Shader::compileProgram(const std::string &vertexName, const std::string &fragmentName) {
    
    std::string vertPath = std::string("shaders/") + vertexName;
    std::string fragPath = std::string("shaders/") + fragmentName;

    std::string vertSource = readShaderSource(vertPath);
    std::string fragSource = readShaderSource(fragPath);

    GLuint vertShader = compileShader(GL_VERTEX_SHADER, vertSource.c_str());
    GLuint fragShader = compileShader(GL_FRAGMENT_SHADER, fragSource.c_str());
    
    GLuint Program = glCreateProgram();
    glAttachShader(Program, fragShader);
    glAttachShader(Program, vertShader);

    int32_t result;
    glLinkProgram(Program);
    glGetProgramiv(Program, GL_LINK_STATUS, &result);
    if (!result) {
        char error[512];
        glGetProgramInfoLog(Program, 512, nullptr, error);
        std::cout << "ERROR::PROGRAM::COULD_NOT_LINK::" << error << std::endl;
    }

    glDeleteShader(fragShader);
    glDeleteShader(vertShader);

    return Program;
}

std::string Shader::readShaderSource(const std::string &shaderPath) {
    std::string shaderSource;
    std::stringstream shaderStream;
    std::ifstream shaderSourceFile;
    shaderSourceFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    try {
        shaderSourceFile.open(shaderPath);
        shaderStream << shaderSourceFile.rdbuf();
        shaderSourceFile.close();
        shaderSource = shaderStream.str();
    }
    catch (std::ifstream::failure e)
    {
        std::cout << "ERROR::SHADER::COULD_NOT_READ_FILE:" << "shaders/default.fsh" << std::endl;
    }
    return shaderSource;
}

Shader::Shader(const std::string &vertexName, const std::string &fragmentName) : mProgramID(compileProgram(vertexName, fragmentName)) {}

GLuint Shader::compileShader(GLuint type, const char *source) {
    
    GLuint shader = glCreateShader(type);
    
    std::string strType = "UNKNOWN";
    if (type == GL_FRAGMENT_SHADER) {
        strType = "FRAGMENT";
    } else if (type == GL_VERTEX_SHADER) {
        strType = "VERTEX";
    }

    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    int32_t result;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
    if (!result) {
        char error[512];
        glGetShaderInfoLog(shader, 512, nullptr, error);
        std::cout << "ERROR::" << strType << "::COULD_NOT_COMPILE::" << error << std::endl;
    }
    
    return shader;
}

void Shader::use() const {
    glUseProgram(this->mProgramID);
}



bool Shader::setFloat(const std::string &name, float value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniform1f(location, value);
    return true;
}

bool Shader::setInt(const std::string &name, int32_t value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniform1i(location, value);
    return true;
}

bool Shader::setMat4(const std::string &name, glm::mat4 value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
    return true;
}

bool Shader::setVec3(const std::string &name, glm::vec3 value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniform3f(location, value.x, value.y, value.z);
    return true;
}

bool Shader::setVec4(const std::string &name, glm::vec4 value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniform4f(location, value.x, value.y, value.z, value.w);
    return true;
}

bool Shader::setBool(const std::string &name, bool value) {
    GLint location = glGetUniformLocation(this->mProgramID, name.c_str());
    if (location == -1) {
        return false;
    }
    glUniform1i(location, value);
    return true;}
