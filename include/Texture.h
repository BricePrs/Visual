//
// Created by brice on 11/25/23.
//

#ifndef VISUAL_TEXTURE_H
#define VISUAL_TEXTURE_H

#include <GlobalVar.h>
#include "glad/glad.h"

class Texture {
public:

    Texture(const char *data, uint32_t width, uint32_t height, uint8_t channelCount);

    void AssignToLocation(uint8_t n) const;

private:
    GLuint Id;
};


#endif //VISUAL_TEXTURE_H
