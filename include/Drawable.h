//
// Created by brice on 11/22/23.
//

#ifndef VISUAL_DRAWABLE_H
#define VISUAL_DRAWABLE_H


#include "Camera.h"
#include "SceneId.h"
#include <Shader.h>
#include <optional>
#include <vector>

class Drawable {
public:
    virtual void Draw(const Camera &camera, Shader &shader) = 0;
    virtual void DrawWindow() {} // TODO better ?
    virtual void Update(double deltaTime) {}
    virtual std::optional<std::vector<Drawable>> GetDrawableElts() { return {}; };
};


#endif //VISUAL_DRAWABLE_H
