//
// Created by brice on 11/17/23.
//

#include "Scene.h"

void Scene::Draw(const PerspectiveCamera &camera) {
    for (auto &object :mSceneObjects) {
        object->Draw(camera);
    }
}

void Scene::AddObject(Drawable *object) {
    mSceneObjects.emplace_back(object);
}

void Scene::AddCollider(Collider *object) {
    mSceneColliderObjects.emplace_back(object);
}

std::vector<Collider *> &Scene::GetColliderObjects() {
    return mSceneColliderObjects;
}

