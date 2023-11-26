//
// Created by brice on 11/25/23.
//

#include <iostream>
#include "Texture.h"

Texture::Texture(const char *data, uint32_t width, uint32_t height, uint8_t channelCount) {
    glGenTextures(1, &Id);
    glBindTexture(GL_TEXTURE_2D, Id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    if (data)
    {
        auto format = GL_RED;
        glTexImage2D(GL_TEXTURE_2D, 0, format, static_cast<GLint>(width), static_cast<GLint>(height), 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
}

void Texture::AssignToLocation(uint8_t n) const {
    glActiveTexture(GL_TEXTURE0 + n);
    glBindTexture(GL_TEXTURE_2D, Id);
}
