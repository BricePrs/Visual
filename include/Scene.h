//
// Created by brice on 11/17/23.
//

#ifndef VISUAL_SCENE_H
#define VISUAL_SCENE_H
#include <vector>
#include <Drawable.h>
#include <optional>
#include <memory>
#include <unordered_map>
#include "Collider.h"
#include "Shader.h"
#include <SceneId.h>


class Scene {
public:


    Scene() = default;

    void Draw(const PerspectiveCamera &camera);

    ShaderId AddShader(const std::string &vertexShaderName, const std::string &fragmentShaderName);
    Shader  GetShader(ShaderId& shaderId) { return *mSceneShaders[shaderId.GetLocation()]; };

    template <class T>
    ObjectId AddObject(std::shared_ptr<T> object, ShaderId &shaderId);

    template <class T>
    uint32_t AddCollider(std::shared_ptr<T> object);

    template <class T>
    void SetObject(ObjectId& objectId, std::shared_ptr<T> object);

    std::vector<std::vector<std::shared_ptr<Drawable>>>& GetObjects();
    std::vector<std::shared_ptr<Collider>>& GetColliderObjects();

private:

    // Type trait to check if a type is derived from Drawable
    template <typename T>
    struct is_drawable : std::is_base_of<Drawable, T> {};

    // Type trait to check if a type is derived from Drawable
    template <typename T>
    struct is_collider : std::is_base_of<Collider, T> {};


    std::vector<std::vector<std::shared_ptr<Drawable>>> mSceneObjects;
    std::vector<std::shared_ptr<Collider>> mSceneColliderObjects;
    std::optional<uint32_t> mSelectedObject;

    std::vector<std::unique_ptr<Shader>> mSceneShaders;
    std::vector<std::vector<ObjectId>> mDependencyTree;

};

//template <class T>
//ObjectId Scene::AddObject(std::shared_ptr<T> object) {
//    static_assert(is_drawable<T>::value, "T must be derived from Drawable");
//    auto drawableObject = std::static_pointer_cast<Drawable>(object);
//    if (drawableObject->GetDrawableElts().has_value()) {
//        for (auto &drawableElt: drawableObject->GetDrawableElts().value()) {
//            AddObject(std::make_shared<Drawable>(drawableObject));
//        }
//    } else {
//        if (drawableObject->GetShaderId().has_value()) {
//            throw std::runtime_error("Adding object with no shader set");
//        }
//        mSceneObjects[drawableObject->GetShaderId().value()].emplace_back();
//    }
//    return { drawableObject->GetShaderId().value(), static_cast<uint32_t>(mSceneObjects.size()-1) };
//}

template <class T>
ObjectId Scene::AddObject(std::shared_ptr<T> object, ShaderId &shaderId) {
    static_assert(is_drawable<T>::value, "T must be derived from Drawable");
    auto drawableObject = std::static_pointer_cast<Drawable>(object);
    mSceneObjects[shaderId.GetLocation()].emplace_back(drawableObject);
    return {shaderId, static_cast<uint32_t>(mSceneObjects[shaderId.GetLocation()].size()-1) };
}

template <class T>
uint32_t Scene::AddCollider(std::shared_ptr<T> object) {
    static_assert(is_collider<T>::value, "T must be derived from Collider");
    mSceneColliderObjects.emplace_back(std::static_pointer_cast<Collider>(object));
    return static_cast<uint32_t>(mSceneColliderObjects.size()-1);
}

template <class T>
void Scene::SetObject(ObjectId& objectId, std::shared_ptr<T> object) {
    static_assert(is_drawable<T>::value, "T must be derived from Drawable");
    mSceneObjects[objectId.getShaderId().GetLocation()][objectId.getObjectId()] = std::static_pointer_cast<Drawable>(object);
}


#endif //VISUAL_SCENE_H
