//
// Created by brice on 11/22/23.
//

#ifndef VISUAL_DRAWABLE_H
#define VISUAL_DRAWABLE_H


#include "PerspectiveCamera.h"

class Drawable {
public:
    virtual void Draw(const PerspectiveCamera &camera) = 0;
};


#endif //VISUAL_DRAWABLE_H
