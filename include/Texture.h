//
// Created by brice on 11/25/23.
//

#ifndef VISUAL_TEXTURE_H
#define VISUAL_TEXTURE_H

#include "glad/glad.h"
#include <GlobalVar.h>

class Texture {
public:

    Texture() : mId(0), mWidth(0), mHeight(0) {}
    Texture(const char *data, uint32_t width, uint32_t height, uint8_t channels);
    Texture(const char *data, uint32_t width, uint32_t height, GLenum format, GLenum internalFormat);
    Texture(uint32_t width, uint32_t height, uint8_t channels)
            : Texture(nullptr, width, height, channels) {}

    void AssignToLocation(uint8_t n) const;
    void CudaRegister();
    void* CudaMap();
    void CudaUnMap();

    void SetData(const char *data);
    void Bind();

    GLuint GetOpenglId() const { return mId; }

    static Texture LoadFromFile(const std::string& fileName);
    static GLenum ChannelToFormat(uint8_t channels);
    static GLenum ChannelToInternalFormat(uint8_t channels);

private:

    GLuint mId;
    uint32_t mWidth;
    uint32_t mHeight;

};


#endif //VISUAL_TEXTURE_H