//
// Created by brice on 11/25/23.
//

#include <iostream>
#include <string>
#include <stb_image/stb_image.h>
#include "Texture.h"

// Function to convert OpenGL error code to string
std::string glErrorToString(GLenum error) {
    switch (error) {
        case GL_NO_ERROR:
            return "GL_NO_ERROR";
        case GL_INVALID_ENUM:
            return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE:
            return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION:
            return "GL_INVALID_OPERATION";
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            return "GL_INVALID_FRAMEBUFFER_OPERATION";
        case GL_OUT_OF_MEMORY:
            return "GL_OUT_OF_MEMORY";
        case GL_STACK_UNDERFLOW:
            return "GL_STACK_UNDERFLOW";
        case GL_STACK_OVERFLOW:
            return "GL_STACK_OVERFLOW";
        default:
            return "Unknown OpenGL Error";
    }
}

// Macro to check and print OpenGL errors
#define CHECK_GL_ERROR() \
    do { \
        GLenum error = glGetError(); \
        if (error != GL_NO_ERROR) { \
            std::cerr << "OpenGL Error (" << __FILE__ << ":" << __LINE__ << "): " << glErrorToString(error) << " (" << error << ")" << std::endl; \
        } \
    } while (false)

GLenum Texture::ChannelToFormat(uint8_t channels) {
    if (channels == 1) {
        return GL_RED;
    } else if (channels == 4) {
        return GL_RGBA;
    } else {
        throw std::runtime_error("Cannot have this number of channels");
    }
}

GLenum Texture::ChannelToInternalFormat(uint8_t channels) {
    if (channels == 1) {
        return GL_R8;
    } else if (channels == 4) {
        return GL_RGBA32F;
    } else {
        throw std::runtime_error("Cannot have this number of channels");
    }
}

Texture::Texture(const char *data, uint32_t width, uint32_t height, uint8_t channels)
    : Texture(data, width, height, ChannelToFormat(channels), ChannelToInternalFormat(channels))
{}

Texture::Texture(const char *data, uint32_t width, uint32_t height, GLenum format, GLenum internalFormat) {
    CHECK_GL_ERROR();
    glGenTextures(1, &mId);
    CHECK_GL_ERROR();
    glBindTexture(GL_TEXTURE_2D, mId);
    CHECK_GL_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    CHECK_GL_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    CHECK_GL_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    CHECK_GL_ERROR();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    CHECK_GL_ERROR();
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, static_cast<GLint>(width), static_cast<GLint>(height), 0, format, GL_UNSIGNED_BYTE, data);
    CHECK_GL_ERROR();
    glGenerateMipmap(GL_TEXTURE_2D);
    CHECK_GL_ERROR();

}

void Texture::AssignToLocation(uint8_t n) const {
    glActiveTexture(GL_TEXTURE0 + n);
    glBindTexture(GL_TEXTURE_2D, mId);
}

void Texture::SetData(const char *data) {
    auto format = GL_RED;
    glBindTexture(GL_TEXTURE_2D, mId);
    glTexImage2D(GL_TEXTURE_2D, 0, format, static_cast<GLint>(mWidth), static_cast<GLint>(mHeight), 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}

void Texture::Bind() {
    glBindTexture(GL_TEXTURE_2D, mId);
}

Texture Texture::LoadFromFile(const std::string &fileName) {
    stbi_set_flip_vertically_on_load(true);
    int width, height, channels;
    auto data = stbi_load(fileName.c_str(), &width, &height, &channels, 0);
    return {reinterpret_cast<const char *>(data), static_cast<uint32_t>(width), static_cast<uint32_t>(height), GL_RGB, GL_RGBA8 };
}
