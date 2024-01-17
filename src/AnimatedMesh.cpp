//
// Created by brice on 12/5/23.
//

#include "AnimatedMesh.h"
#include <algorithm>
#include <sstream>

AnimatedMesh::AnimatedMesh(const std::string &skeletonFileName, const std::string &skinFileName, const std::string &weightsFileName)
{

    // -- Skeleton Set-up -- //

    mRootJoint = Joint::createFromFile(skeletonFileName, mFrameCount, mFrameTime);
    
    std::vector<SimpleVertex> skeletonVertices;
    std::vector<uint32_t> skeletonIndices;
    mRootJoint->buildSkeletonMatrices(skeletonVertices, skeletonIndices, glm::mat4(1.));

    mSkeletonMesh = Mesh<SimpleVertex>(skeletonVertices, skeletonIndices);
    mSkeletonMesh.SetPrimitiveMode(GL_LINES);
    mSkeletonMesh.SetScale(glm::vec3(0.01));
    // mSkeletonMesh.SetPosition(glm::vec3(-4., 4., 1.));
    // mSkeletonMesh.SetColor(glm::vec3(1., 0.3, 0.2));

    // -- Skin Set-up -- //
    mSkinMesh = ParseOFF(skinFileName);
    mSkinMeshTransformed = ParseOFF(skinFileName);
    mSkinMesh.SetScale(glm::vec3(0.01));
    // mSkinMesh.SetPosition({-3, 1., 0.});
    // mSkinMesh.SetDrawMode(GL_LINE);
    mSkinMeshTransformed.SetScale(glm::vec3(0.01));
    // mSkinMeshTransformed.SetPosition({3, 1., 3});
    mSkinMeshTransformed.SetDrawMode(GL_LINE);

    // -- Weights Set-up -- //
    std::unordered_map<std::string, Joint *> jointMap; // Temporary map of joints
    mRootJoint->populateJointMap(jointMap);
    ParseWeights(weightsFileName, jointMap);
    // assert(jointMap.size() == mSkeletonMesh.GetNbVertices());
    mRootJoint->transformMatricesBinding(B_MJ, glm::mat4(1.));

    auto &vertices = mSkinMesh.GetVertices();
    for(int i = 0; i < vertices.size(); ++i) {
        std::vector<std::pair<double, Joint *>> weight_array = weight[i];
        vertices[i].color = glm::vec3(0.);
        for (const auto& weight_pair : weight_array) {
            vertices[i].color += static_cast<float>(weight_pair.first) * weight_pair.second->_color;
        }
    }
    mSkinMesh.UpdateVerticesData();
}

void AnimatedMesh::ParseWeights(const std::string &weightsFileName, std::unordered_map<std::string, Joint *> &jointMap){
    // Order of joint in weight is assumed to be the same as in bvh
    std::cout << "Parsing weights" << std::endl;
    std::ifstream inputfile(weightsFileName.c_str());
    if(inputfile.good()) {
        std::string token;
        inputfile >> token;

        // Parse first line
        std::string line;
        std::getline(inputfile, line);
        std::istringstream iss(line); // Tokenize the line
        while (iss >> token) {
            // Process each token as needed
            // std::cout << "Token: " << token << std::endl;
            auto it = jointMap.find(token);
            if (it != jointMap.end()) {
                Joint* foundJoint = it->second;
                // std::cout << "TOKEN FOUND : " << token << std::endl;
                jointArray.push_back(foundJoint);
            }
             else{
                std::cout << "TOKEN NOT FOUND : " << token << std::endl;
                jointArray.push_back(nullptr);
            }
        }

        // Parse the other lines
        for (int i = 0; i < mSkinMesh.GetNbVertices(); ++i){
            inputfile >> token;
            // std::cout << "PARSED TOKEN : " << token << std::endl;
            unsigned int index = std::stoi(token);
            std::vector<std::pair<double, Joint *>> weight_array;
            for (int j = 0; j < jointArray.size(); ++j){
                inputfile >> token;
                if (jointArray[j] != nullptr){
                    double value = std::stod(token);
                    if (value != 0){
                        auto pair = std::pair<double, Joint *>(value, jointArray[j]);
                        // std::cout << "Forming pair " << value << std::endl;
                        weight_array.push_back(pair);
                    }
                }
            }
            weight.push_back(weight_array);
        }

        inputfile.close();
    } else {
        std::cerr << "Failed to load the weight file " << weightsFileName.c_str() << std::endl;
        fflush(stdout);
    }
}

void AnimatedMesh::Update(double dt) {

    // -- Skeleton Update -- //

    double animationTime = fmod(dt, mFrameTime*mFrameCount) / mFrameTime;
    uint32_t frameNumber = static_cast<uint32_t>(trunc(animationTime));
    double framePercent = animationTime-frameNumber;

    mRootJoint->animateLerp(frameNumber, framePercent);
    // mRootJoint->animate(frameNumber);
    std::vector<SimpleVertex> skeletonVertices;
    std::vector<uint32_t> skeletonIndices; // Not used

    mRootJoint->buildSkeletonMatrices(skeletonVertices, skeletonIndices, glm::mat4(1.));
    mSkeletonMesh.ChangeVertices(skeletonVertices);


    // -- Skin Update -- //
    mRootJoint->transformMatrices(C_JM, glm::mat4(1.));

    std::vector<SimpleColorVertex> const &old_vertices = mSkinMesh.GetVertices();
    std::vector<SimpleColorVertex> new_vertices;
    new_vertices.reserve(old_vertices.size());
    for(int i = 0; i < old_vertices.size(); ++i){
        glm::vec4 old_position = glm::vec4(old_vertices[i].position, 1.f);
        glm::vec4 new_position = glm::vec4(0.0, 0.0, 0.0, 0.f);
        std::vector<std::pair<double, Joint *>> weight_array = weight[i];
        // assert(weight_array.size() != 0);
        int counter = 0;
        double sum_weight = 0;
        glm::mat4 accumulateTransform = glm::mat4(0);
        for (const auto& weight_pair : weight_array) {
            double weight = weight_pair.first;
            counter++;
            sum_weight += weight;
        }
        for (const auto& weight_pair : weight_array) {
            double weight = weight_pair.first;
            Joint* jointPtr = weight_pair.second;
            glm::mat4 K = C_JM[jointPtr]*B_MJ[jointPtr];
            // glm::mat4 K = B_MJ[jointPtr];
            // K = glm::identity<glm::mat4>();
            // K = glm::angleAxis(glm::radians(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

            accumulateTransform = accumulateTransform + (static_cast<float>(weight/sum_weight))*K;
            // new_position = new_position + (static_cast<float>(weight/sum_weight))*K*old_position;
            // std::cout << "weight " << weight << std::endl;
        }
        new_position = accumulateTransform * old_position;
        // std::cout << "sum_weight " << sum_weight << std::endl;
        // assert(std::abs(1 - sum_weight) <= 0.9);
        // std::cout << "NEW_POSITION " << new_position.x << " " << new_position.y << " " << new_position.z << std::endl;
        // std::cout << "counter : " << counter << std::endl;
        new_vertices.emplace_back(glm::vec3(new_position)/new_position.w, old_vertices[i].color);
    }
    mSkinMeshTransformed.ChangeVertices(new_vertices);
}

void AnimatedMesh::Draw(const PerspectiveCamera &camera) {
    mRootJoint->Draw(camera);
    mSkeletonMesh.Draw(camera);
    mSkinMeshTransformed.Draw(camera);
}
