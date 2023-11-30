//
// Created by brice on 11/17/23.
//

#ifndef VISUAL_SCENE_H
#define VISUAL_SCENE_H
#include <vector>
#include <Drawable.h>
#include <optional>
#include "Collider.h"


class Scene {
public:

    Scene() = default;

    void Draw(const PerspectiveCamera &camera);
    void AddObject(Drawable* object);
    void AddCollider(Collider* object);
    std::vector<Drawable*>& GetObjects();
    std::vector<Collider*>& GetColliderObjects();

private:
    std::vector<Drawable*> mSceneObjects;
    std::vector<Collider*> mSceneColliderObjects;
    std::optional<uint32_t> mSelectedObject;
};


#endif //VISUAL_SCENE_H
