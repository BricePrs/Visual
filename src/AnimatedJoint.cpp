//
// Created by brice on 1/8/24.
//

#include <fstream>
#include "AnimatedJoint.h"

void AnimatedJoint::Draw(const PerspectiveCamera &camera) {
    mArrowX.Draw(camera);
    mArrowY.Draw(camera);
    mArrowZ.Draw(camera);
}

void AnimatedJoint::Update(double dt) {

    double animationTime = fmod(dt, mFrameTime*mFrameCount) / mFrameTime;
    uint32_t frameNumber = static_cast<uint32_t>(trunc(animationTime));
    double framePercent = animationTime-frameNumber;

    auto eulerAngles = glm::eulerAngles(mOrientations[frameNumber]);

    mArrowX.SetRotation(eulerAngles);
    mArrowY.SetRotation(eulerAngles);
    mArrowZ.SetRotation(eulerAngles);
}

AnimatedJoint::AnimatedJoint(const std::string &fileName, glm::vec3 position) :
        mArrowX(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(1., 0., 0.)*0.6f, glm::vec3(1., 0., 0.))),
        mArrowY(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 1., 0.)*0.6f, glm::vec3(0., 1., 0.))),
        mArrowZ(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 0., 1.)*0.6f, glm::vec3(0., 0., 1.)))
{
    mArrowX.SetPosition(position);
    mArrowY.SetPosition(position);
    mArrowZ.SetPosition(position);


    std::ifstream inputfile(fileName.data());
    if(inputfile.good()) {
        std::string buf;
        inputfile >> buf;

        while (!buf.compare("Quat_q3")) {
            inputfile >> buf;
        }
        while (!inputfile.eof()) {
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;
            inputfile >> buf;

            glm::quat q;
            inputfile >> buf;
            q.x = strtof(buf.c_str(), nullptr);
            inputfile >> buf;
            q.y = strtof(buf.c_str(), nullptr);
            inputfile >> buf;
            q.z = strtof(buf.c_str(), nullptr);
            inputfile >> buf;
            q.w = strtof(buf.c_str(), nullptr);

            mOrientations.push_back(q);
            mFrameCount++;
        }


        inputfile.close();
    } else {
        std::cerr << "Failed to load the file " << fileName.data() << std::endl;
        fflush(stdout);
    }

    mFrameTime = 10./60.;
    std::cout << "file loaded" << std::endl;
}
