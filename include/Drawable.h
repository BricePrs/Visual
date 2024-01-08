//
// Created by brice on 11/22/23.
//

#ifndef VISUAL_DRAWABLE_H
#define VISUAL_DRAWABLE_H


#include "PerspectiveCamera.h"
#include "SceneId.h"
#include <Shader.h>
#include <optional>
#include <vector>

class Drawable {
public:
    virtual void Draw(const PerspectiveCamera &camera, Shader &shader) = 0;
    virtual std::optional<std::vector<Drawable>> GetDrawableElts() { return {}; }
};


#endif //VISUAL_DRAWABLE_H
