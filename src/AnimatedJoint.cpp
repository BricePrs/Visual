//
// Created by brice on 1/8/24.
//

#include <fstream>
#include "AnimatedJoint.h"

void AnimatedJoint::Draw(const PerspectiveCamera &camera) {
    mArrowX.Draw(camera);
    mArrowY.Draw(camera);
    mArrowZ.Draw(camera);
    if (mMesh.has_value()) {
        mMesh->Draw(camera);
    }
    for (auto &child: mChildren) {
        child->Draw(camera);
    }
}

void AnimatedJoint::Update(double dt) {

    double animationTime = fmod(dt, mFrameTime*mFrameCount) / mFrameTime;
    auto frameNumber = static_cast<uint32_t>(trunc(animationTime));
    double framePercent = animationTime-frameNumber;
    mCurrentOrientation = glm::slerp(mOrientations[frameNumber], mOrientations[(frameNumber+1)%mFrameCount], (float)framePercent);
    mArrowX.SetRotation(mCurrentOrientation);
    mArrowY.SetRotation(mCurrentOrientation);
    mArrowZ.SetRotation(mCurrentOrientation);

    for (auto &child: mChildren) {
        child->SetPosition(mPosition + glm::inverse(mCurrentOrientation) * child->mRelativePosition);
        child->Update(dt);
    }
}

std::shared_ptr<AnimatedJoint> AnimatedJoint::AddChildren(const std::string &fileName, glm::vec3 relativePosition, const std::string &name) {
    mChildren.emplace_back(std::make_shared<AnimatedJoint>(fileName, mPosition+ glm::inverse(mOrientations[0]) * relativePosition, name));
    mChildren.back()->mRelativePosition = relativePosition;
    return mChildren.back();
}

AnimatedJoint::AnimatedJoint(const std::string &fileName, glm::vec3 position, const std::string &name) :
        mName(name),
        mArrowX(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(1., 0., 0.)*ARROW_SIZE, glm::vec3(1., 0., 0.))),
        mArrowY(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 1., 0.)*ARROW_SIZE, glm::vec3(0., 1., 0.))),
        mArrowZ(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 0., 1.)*ARROW_SIZE, glm::vec3(0., 0., 1.))),
        mFrameCount(0),
        mPosition(position),
        mRelativePosition(0.f)
{

    if (name.empty()) { mName = fileName; }

    mArrowX.SetPosition(position);
    mArrowY.SetPosition(position);
    mArrowZ.SetPosition(position);

    std::ifstream inputfile(fileName.data());
    if(inputfile.good()) {
        std::string buf;
        inputfile >> buf;
        while (buf.compare("Quat_q3")) {
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

            mOrientations.push_back(q*glm::quat(glm::vec3(0., 0., glm::radians(90.))));
            mFrameCount++;
        }
        inputfile.close();
    } else {
        throw std::runtime_error("Failed to load the file");
    }

    mFrameTime = 1./60.;
    std::cout << "file loaded" << std::endl;
}

void AnimatedJoint::SetPosition(const glm::vec3 &position) {
    mPosition = position;
    mArrowX.SetPosition(mPosition);
    mArrowY.SetPosition(mPosition);
    mArrowZ.SetPosition(mPosition);
}

void AnimatedJoint::SetEnd(glm::vec3 v) {
    mEndPos = std::make_optional(v);
}

void AnimatedJoint::BuildMesh() {
    std::vector<SimpleVertex> vertices;
    std::vector<uint32_t> indices;
    BuildMeshRec(vertices, indices, mPosition);
    mMesh = Mesh<SimpleVertex>(vertices, indices);
    mMesh->SetPrimitiveMode(GL_LINES);
    mMesh->SetDrawMode(GL_LINE);
}

void AnimatedJoint::BuildMeshRec(std::vector<SimpleVertex> &vertices, std::vector<uint32_t> &indices, glm::vec3 position) {
    uint32_t currentIndex = vertices.size();
    vertices.emplace_back(position);
    for (auto &child: mChildren) {
        indices.emplace_back(currentIndex);
        indices.emplace_back(vertices.size());
        child->BuildMeshRec(vertices, indices, position+glm::inverse(mCurrentOrientation)*child->mRelativePosition);
    }
    if (mEndPos.has_value()) {
        indices.emplace_back(currentIndex);
        indices.emplace_back(vertices.size());
        vertices.emplace_back(position+glm::inverse(mCurrentOrientation)*mEndPos.value());
    }

}

void AnimatedJoint::ExportBVH(const std::string &filename) {
    std::ofstream outStream(filename, std::ios::trunc);
    // Check if the file is opened successfully
    if (!outStream.is_open()) {
        throw std::runtime_error("Error opening the file!");
    }

    outStream << "HIERARCHY\n";
    WriteJoint(outStream, "ROOT");

    outStream << "MOTION\n";
    outStream << "Frames: " << mFrameCount << "\n";
    outStream << "Frame Time: " << mFrameTime << "\n";

    for (uint32_t frame = 0; frame < mFrameCount; ++frame) {
        WriteKeyFrame(outStream, frame);
        outStream << "\n";
    }

    outStream.close();

}

void AnimatedJoint::WriteJoint(std::ofstream &outStream, const std::string &type, const std::string &prefix) {
    float SCALE_FACTOR = 100.f;
    std::string newPrefix = prefix + '\t';
    outStream << prefix << type << " " << mName << "\n";
    outStream << prefix << "{\n";
    outStream << newPrefix << "OFFSET " << mRelativePosition.x*SCALE_FACTOR << " " << mRelativePosition.y*SCALE_FACTOR << " " << mRelativePosition.z*SCALE_FACTOR << "\n";

    auto o = glm::eulerAngles(mOrientations[0]);
    outStream << newPrefix << "CHANNELS 3 Xrotation Yrotation Zrotation\n";

    for (auto &child: mChildren) {
        child->WriteJoint(outStream, "JOINT", newPrefix);
    }
    if (mEndPos.has_value()) {
        outStream << newPrefix << "End " << mName << "_End\n";
        outStream << newPrefix << "{\n";
        outStream << newPrefix << "\t" << "OFFSET " << mEndPos.value().x*SCALE_FACTOR << " " << mEndPos.value().y*SCALE_FACTOR << " " << mEndPos.value().z*SCALE_FACTOR << "\n"; // SCALING VALUES
        outStream << newPrefix << "}\n";
    }
    outStream << prefix << "}\n";

}


void AnimatedJoint::WriteKeyFrame(std::ofstream &outStream, uint32_t frame, glm::quat parentOrientation) {
    glm::vec3 o = -glm::eulerAngles(mOrientations[frame]*glm::inverse(parentOrientation));
    outStream << glm::degrees(o.x) << " " << glm::degrees(o.y) << " " << glm::degrees(o.z) << " ";
    for (auto &child: mChildren) {
        child->WriteKeyFrame(outStream, frame, mOrientations[frame]);
    }
}
