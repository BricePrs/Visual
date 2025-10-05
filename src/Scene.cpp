//
// Created by brice on 11/17/23.
//

#include <memory>
#include "Scene.h"

void Scene::Draw() const {
    for (size_t i = 0; i < mSceneShaders.size(); i++) {
        for (auto &object :mSceneObjects[i]) {
            object->Draw(_camera, *mSceneShaders[i]);
        }
    }
}


ShaderId Scene::AddShader(const std::string &vertexShaderName, const std::string &fragmentShaderName) {
    mSceneShaders.emplace_back(std::make_unique<Shader>(vertexShaderName, fragmentShaderName));
    mSceneObjects.emplace_back();
    return {mSceneShaders.size() - 1, vertexShaderName, fragmentShaderName};
}


void Scene::DrawWindows() const {
    for (auto &objects : mSceneObjects) {
        for (auto &object : objects) {
            object->DrawWindow();
        }
    }
}

std::vector<std::vector<std::unique_ptr<Drawable>>> &Scene::GetObjects() {
    return mSceneObjects;
}




