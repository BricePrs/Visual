//
// Created by brice on 11/17/23.
//

#include "Scene.h"

void Scene::Draw(const PerspectiveCamera &camera) {
    for (auto &object :mSceneObjects) {
        object->Draw(camera);
    }
}

uint32_t Scene::AddObject(Drawable *object) {
    mSceneObjects.emplace_back(object);
    return mSceneObjects.size()-1;
}

void Scene::AddCollider(Collider *object) {
    mSceneColliderObjects.emplace_back(object);
}

std::vector<Drawable *> &Scene::GetObjects() {
    return mSceneObjects;
}

std::vector<Collider *> &Scene::GetColliderObjects() {
    return mSceneColliderObjects;
}

