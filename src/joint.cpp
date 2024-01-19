#include "joint.h"
#include "Mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstring>
#include <glm/gtc/quaternion.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/color_space.hpp>

using namespace std;

Joint* Joint::parseHierarchy(std::ifstream &inputfile) {
    auto* root = new Joint();
    string buf;
    inputfile >> buf;
    if (buf == "ROOT") {
        root->parseJoint(inputfile);
    }
    return root;
}

void Joint::parseMotion(std::ifstream &inputfile, Joint *root, uint32_t &frameCount, double &frameTime) {
    string buf;
    inputfile >> buf; // parse Frames:
    if (buf != "Frames:") {
        std::cout << "ERROR : found " << buf << " instead of Frames:";
    }
    inputfile >> buf;
    frameCount = stoi(buf);
    inputfile >> buf; // parse Frame Time:
    if (buf != "Frame") {
        std::cout << "ERROR : found " << buf << " instead of Frames";
    }
    inputfile >> buf; // parse Frame Time:
    if (buf != "Time:") {
        std::cout << "ERROR : found " << buf << " instead of Time:";
    }
    inputfile >> buf; // parse Frame Time:
    frameTime = strtod(buf.c_str(), nullptr);

    for (int i = 0; i < frameCount; ++i) {
        root->parseCurveKeyframes(inputfile);
    }
    while (!inputfile.eof()) {
        std::cout << "left ";
        inputfile >> buf;
        std::cout << buf << "\n";
    }
}

void Joint::parseCurveKeyframes(std::ifstream &inputfile) {
    string buf;
    for (auto &curve : _dofs) {
        inputfile >> buf;
        char* result;
        double value = strtod(buf.c_str(), &result);
        if (std::strcmp(result, "") != 0) {
            std::cout << "result :" << result;
        }
        curve._values.push_back(value);
        if (_name == "r_knee_dup") {
            std::cout << curve.name << " " << value << std::endl;
            std::cout << "buf = " << buf << std::endl;
        }
    }
    for (auto &child: _children) {
        child->parseCurveKeyframes(inputfile);
    }

}

void Joint::parseJoint(std::ifstream &inputfile) {
    string buf;
    inputfile >> buf;
    _name = buf;

    inputfile >> buf; //parse {
    if (buf != "{") {
        std::cout << "ERROR : found " << buf << " instead of {";
    }

    inputfile >> buf; //parse OFFSET
    if (buf != "OFFSET") {
        std::cout << "ERROR : found " << buf << " instead of OFFSET";
    }

    inputfile >> buf;
    _offX = strtod(buf.c_str(), nullptr);
    inputfile >> buf;
    _offY = strtod(buf.c_str(), nullptr);
    inputfile >> buf;
    _offZ = strtod(buf.c_str(), nullptr);

    _curRx = 0.;
    _curRy = 0.;
    _curRz = 0.;

    inputfile >> buf; // parse CHANNELS
    if (buf == "CHANNELS") {
        inputfile >> buf;
        int channelsCount = stoi(buf, nullptr);
        for (int i = 0; i < channelsCount; ++i) {
            inputfile >> buf;
            AnimCurve curve;
            curve.name = buf;
            _dofs.emplace_back(curve);
        }
        inputfile >> buf;
    }

    while (buf == "JOINT" || buf == "End") {
        Joint* child = new Joint();
        child->parseJoint(inputfile);
        _children.emplace_back(child);
        inputfile >> buf;
    }

    if (buf != "}") {
        std::cout << "ERROR : found " << buf << " instead of }";
    }

}

Joint * Joint::createFromFile(std::string fileName, uint32_t &frameCount, double &frameTime) {
	Joint* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	
			inputfile >> buf;
            if (buf == "HIERARCHY") {
                root = parseHierarchy(inputfile);
            }
            if (buf == "MOTION") {
                parseMotion(inputfile, root, frameCount, frameTime);
            }

        }
		inputfile.close();
	} else {
		std::cerr << "Failed to load the file " << fileName.data() << std::endl;
		fflush(stdout);
	}

	cout << "file loaded" << endl;

	return root;
}

void Joint::animate(int iframe)
{
    //std::cout << "Computing frame " << iframe << std::endl;
    // Update dofs :
    _curTx = 0; _curTy = 0; _curTz = 0;
    _curRx = 0; _curRy = 0; _curRz = 0;
    for (unsigned int idof = 0 ; idof < _dofs.size() ; idof++) {
        if (_children.empty()) continue;
        if (_dofs[idof]._values.size() <= iframe) {
            //std::cout << "Max value is " << _dofs[idof]._values.size() << " but asked for " << iframe << std::endl;
            continue;
        }
        if(!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
        if(!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
        if(!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
        if(!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
        if(!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
        if(!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
    }

    // Animate children :
    for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
        _children[ichild]->animate(iframe);
    }
}

void Joint::animateLerp(int iframe, double framePercent)
{
    assert(0. <= framePercent && framePercent < 1.);

    animate(iframe);
    double p_curTx = _curTx; double p_curTy = _curTy; double p_curTz = _curTz;
    glm::quat pr = glm::quat(glm::vec3(glm::radians(_curRx), glm::radians(_curRy), glm::radians(_curRz)));
    animate(iframe+1);
    glm::quat nr = glm::quat(glm::vec3(glm::radians(_curRx), glm::radians(_curRy), glm::radians(_curRz)));
    _curTx = (1.-framePercent) * p_curTx + framePercent * _curTx ;
    _curTy = (1.-framePercent) * p_curTy + framePercent * _curTy ;
    _curTz = (1.-framePercent) * p_curTz + framePercent * _curTz ;
    glm::vec3 res = glm::eulerAngles(glm::slerp(pr, nr, (float)framePercent));
    _curRx = glm::degrees(res.x);
    _curRy = glm::degrees(res.y);
    _curRz = glm::degrees(res.z);
    // Animate children :
    for (auto &child: _children) {
        child->animateLerp(iframe, framePercent);
    }

}


void Joint::nbDofs() {
	if (_dofs.empty()) return;

	double tol = 1e-4;

	int nbDofsR = -1;

	// TODO :
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}


void Joint::buildSkeleton(std::vector<SimpleVertex> &vertices, std::vector<uint32_t> &indices, glm::vec3 O, glm::vec3 X, glm::vec3 Y, glm::vec3 Z) const {

    glm::mat4 res = glm::mat4(1.);
    glm::mat4 rotX = glm::mat4(1.);
    glm::mat4 rotY = glm::mat4(1.);
    glm::mat4 rotZ = glm::mat4(1.);
    for (auto &curve: _dofs) {
        if(!curve.name.compare("Zrotation")) rotZ = glm::rotate(rotZ, (float)glm::radians(-_curRz), glm::vec3(0., 0., 1.));
        if(!curve.name.compare("Yrotation")) rotY = glm::rotate(rotY, (float)glm::radians(-_curRy), glm::vec3(0., 1., 0.));
        if(!curve.name.compare("Xrotation")) rotX = glm::rotate(rotX, (float)glm::radians(-_curRx), glm::vec3(1., 0., 0.));
    }
    for (auto &curve: _dofs) {
        if(!curve.name.compare("Zrotation")) res = glm::rotate(glm::mat4(1.), (float)glm::radians(-_curRz), glm::vec3(0., 0., 1.))*res;
        if(!curve.name.compare("Yrotation")) res = glm::rotate(glm::mat4(1.), (float)glm::radians(-_curRy), glm::vec3(0., 1., 0.))*res;
        if(!curve.name.compare("Xrotation")) res = glm::rotate(glm::mat4(1.), (float)glm::radians(-_curRx), glm::vec3(1., 0., 0.))*res;
    }

    auto rot = glm::mat3(rotZ*rotY*rotX);
    auto res3 = glm::mat3(res);
    glm::vec3 nX = res3 * X;
    glm::vec3 nY = res3 * Y;
    glm::vec3 nZ = res3 * Z;

    glm::vec3 origin = O + X*(float)(_curTx+_offX) + Y*(float)(_curTy+_offY) + Z*(float)(_curTz+_offZ);

    for (auto &child: _children) {

        vertices.push_back(origin);
        indices.push_back(vertices.size() - 1);
        indices.push_back(vertices.size());
        child->buildSkeleton(vertices, indices, origin, nX, nY, nZ);
    }
    if (_children.empty()) {
        vertices.push_back(origin);
    }
}

Joint::Joint()
    : _curRx(0), _curRy(0), _curRz(0), _curTx(0), _curTy(0), _curTz(0), _offX(0), _offY(0), _offZ(0),
      _ArrowX(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(1., 0., 0.)*0.15f, glm::vec3(1., 0., 0.))),
      _ArrowY(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 1., 0.)*0.15f, glm::vec3(0., 1., 0.))),
      _ArrowZ(Arrow3D(glm::vec3(0., 0., 0.), glm::vec3(0., 0., 1.)*0.15f, glm::vec3(0., 0., 1.))),
      _Box(WireframeBox(glm::vec3(0.), glm::vec3(0.01), glm::vec3(1., 0.4, 0.1), glm::vec3(-1., 0., -1.), glm::vec3(1., 2., 1.)))
{
    // Generate a saturated random color
    _color = glm::normalize(glm::vec3(
                    glm::linearRand(0.0f, 1.0f),
                    glm::linearRand(0.0f, 1.0f),
                    glm::linearRand(0.0f, 1.0f)));

}

void Joint::buildSkeletonMatrices(vector<SimpleVertex> &vertices, vector<uint32_t> &indices, const glm::mat4 &transform){
    glm::mat4 childTransform = glm::mat4(1.);
    for (auto &curve: _dofs) {
        if(!curve.name.compare("Zrotation")) childTransform = childTransform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRz), glm::vec3(0., 0., 1.));
        if(!curve.name.compare("Yrotation")) childTransform = childTransform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRy), glm::vec3(0., 1., 0.));
        if(!curve.name.compare("Xrotation")) childTransform = childTransform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRx), glm::vec3(1., 0., 0.));
    }

    childTransform[3][0] = (float)(_curTx+_offX);
    childTransform[3][1] = (float)(_curTy+_offY);
    childTransform[3][2] = (float)(_curTz+_offZ);

    childTransform = transform * childTransform;

    for (auto &child: _children) {
        vertices.emplace_back(glm::vec3(childTransform*glm::vec4(0., 0., 0., 1.)));
        indices.push_back(vertices.size() - 1);
        indices.push_back(vertices.size());
        child->buildSkeletonMatrices(vertices, indices, childTransform);
    }
    if (_children.empty()) {
        vertices.emplace_back(glm::vec3(childTransform*glm::vec4(0., 0., 0., 1.)));
    }
}

void Joint::populateJointMap(std::unordered_map<std::string, Joint *> &jointMap) {
    jointMap.insert(std::make_pair(this->_name, this));
    for (auto &child: _children) {
        child->populateJointMap(jointMap);
    }
}

void Joint::transformMatrices(std::unordered_map<Joint *, glm::mat4> &matrices, const glm::mat4 &parentTransform){
    glm::mat4 transform = glm::mat4(1.);
    for (auto &curve: _dofs) {
        if(!curve.name.compare("Zrotation")) transform = transform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRz), glm::vec3(0., 0., 1.));
        if(!curve.name.compare("Yrotation")) transform = transform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRy), glm::vec3(0., 1., 0.));
        if(!curve.name.compare("Xrotation")) transform = transform * glm::rotate(glm::mat4(1.), (float)glm::radians(_curRx), glm::vec3(1., 0., 0.));
    }

    transform[3][0] = (float)(_curTx+_offX);
    transform[3][1] = (float)(_curTy+_offY);
    transform[3][2] = (float)(_curTz+_offZ);

    transform = parentTransform * transform;

    auto arrowTransform = transform;
    arrowTransform[3][0]*=0.01f;
    arrowTransform[3][1]*=0.01f;
    arrowTransform[3][2]*=0.01f;
    _ArrowX.setModel(arrowTransform);
    _ArrowY.setModel(arrowTransform);
    _ArrowZ.setModel(arrowTransform);
    if (!_IsRoot) {
        auto boxTransform = parentTransform;
        auto offset = glm::vec3(_offX*0.01, _offY*0.01, _offZ*0.01);
        float l = glm::length(glm::vec3(offset));
        glm::vec3 mainAxis;
        if (offset.x > offset.y && offset.x > offset.z) {
            mainAxis = glm::vec3(1., 0., .0);
        } else if (offset.y > offset.x && offset.y > offset.z) {
            mainAxis = glm::vec3(0., 1., .0);
        } else {
            mainAxis = glm::vec3(0., 0., 1.);
        }

        _Box = WireframeBox(
                glm::vec3(0.),
                glm::vec3(0.01),
                glm::vec3(1., 0.4, 0.1),
                glm::vec3(-0.1f*l)*(glm::vec3(1.)-mainAxis),
                glm::vec3(0.1f*l)*(glm::vec3(1.)-mainAxis)+glm::vec3(_offX*0.01, _offY*0.01, _offZ*0.01)
            );
        boxTransform[3][0]*=0.01f;
        boxTransform[3][1]*=0.01f;
        boxTransform[3][2]*=0.01f;
        _Box.setModel(boxTransform);
    }
    matrices[this] = transform;

    for (auto &child: _children) {
        child->transformMatrices(matrices, transform);
    }
}

void Joint::transformMatricesBinding(std::unordered_map<Joint *, glm::mat4> &matrices, const glm::mat4 &parentTransform, bool IsRoot) {
    glm::mat4 transform = glm::mat4(1.);

    if (!IsRoot) {
        transform[3][0] = (float) (-_offX);
        transform[3][1] = (float) (-_offY);
        transform[3][2] = (float) (-_offZ);
    }
    transform = transform * parentTransform;

    matrices[this] = transform;

    for (auto &child: _children) {
        child->transformMatricesBinding(matrices, transform);
    }
}

void Joint::Draw(const PerspectiveCamera &camera) {
    _ArrowX.Draw(camera);
    _ArrowY.Draw(camera);
    _ArrowZ.Draw(camera);
    if (!_IsRoot) {
        //_Box.Draw(camera);
    }
    for (auto & child: _children) {
        child->Draw(camera);
    }
}
