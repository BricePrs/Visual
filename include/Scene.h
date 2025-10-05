//
// Created by brice on 11/17/23.
//

#ifndef VISUAL_SCENE_H
#define VISUAL_SCENE_H
#include <utility>
#include <vector>
#include <Drawable.h>
#include <optional>
#include <memory>
#include "Collider.h"
#include "Shader.h"
#include <SceneId.h>

template <typename T>
concept DrawableType = std::is_base_of_v<Drawable, T>;

class Scene {
public:
    explicit Scene(Camera camera) : _camera(std::move(camera)) {}

    void Draw() const;

    ShaderId AddShader(const std::string &vertexShaderName, const std::string &fragmentShaderName);
    Shader  GetShader(ShaderId& shaderId) { return *mSceneShaders[shaderId.GetLocation()]; };

    template <DrawableType T, typename... Args>
    T* AddObject(ShaderId &shaderId, Args&&... args);

    void DrawWindows() const;

    std::vector<std::vector<std::unique_ptr<Drawable>>>& GetObjects();

    Camera          _camera;

private:

    std::vector<std::vector<std::unique_ptr<Drawable>>> mSceneObjects;
    std::optional<uint32_t> mSelectedObject;
    std::vector<std::unique_ptr<Shader>> mSceneShaders;

};


template <DrawableType T, typename... Args>
T* Scene::AddObject(ShaderId &shaderId, Args&&... args) {
    mSceneObjects[shaderId.GetLocation()].emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
    T *ptr = static_cast<T*>(mSceneObjects[shaderId.GetLocation()].back().get());
    return ptr;
}


#endif //VISUAL_SCENE_H
