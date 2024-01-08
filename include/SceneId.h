//
// Created by brice on 12/19/23.
//

#ifndef VISUAL_SCENEID_H
#define VISUAL_SCENEID_H


#include <cstddef>
#include <string>
#include <memory>

struct ShaderId {
private:
    ShaderId(size_t location, const std::string &vertexShaderName, const std::string &fragmentShaderName) : mLocation(location) {
        mVertexShaderName = std::make_shared<std::string>(vertexShaderName);
        mFragmentShaderName = std::make_shared<std::string>(fragmentShaderName);
    }


    [[nodiscard]] size_t GetLocation() const {
        return mLocation;
    }

    [[nodiscard]] const std::shared_ptr<std::string> &getVertexShaderName() const {
        return mVertexShaderName;
    }

    [[nodiscard]] const std::shared_ptr<std::string> &getFragmentShaderName() const {
        return mFragmentShaderName;
    }


    friend class Scene;

    size_t mLocation;
    std::shared_ptr<std::string> mVertexShaderName;
    std::shared_ptr<std::string> mFragmentShaderName;
};

struct ObjectId {
private:
    ObjectId(ShaderId &shaderId, size_t objectId) : mShaderId(shaderId), mObjectId(objectId) {}

    const ShaderId &getShaderId() const {
        return mShaderId;
    }

    size_t getObjectId() const {
        return mObjectId;
    }


    friend class Scene;

    struct ShaderId mShaderId;
    size_t mObjectId;
};


#endif //VISUAL_SCENEID_H
