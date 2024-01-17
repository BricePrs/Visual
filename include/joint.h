#ifndef _JOINT_H_
#define _JOINT_H_

#include <string>
#include <vector>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <glm/glm.hpp>
#include "Mesh.h"

class AnimCurve {
public :
	AnimCurve() = default;
	~AnimCurve() {
		_values.clear();
	}
public :
	std::string name;					// name of dof
	std::vector<double> _values;		// different keyframes = animation curve
};


enum RotateOrder {roXYZ=0, roYZX, roZXY, roXZY, roYXZ, roZYX};

class Joint {
public :
	std::string _name;					// name of joint
	double _offX;						// initial offset in X
	double _offY;						// initial offset in Y
	double _offZ;						// initial offset in Z
	std::vector<AnimCurve> _dofs;		// keyframes : _animCurves[i][f] = i-th dof at frame f;
	double _curTx;						// current value of translation on X
	double _curTy;						// current value of translation on Y
	double _curTz;						// current value of translation on Z
	double _curRx;						// current value of rotation about X (deg)
	double _curRy;						// current value of rotation about Y (deg)
	double _curRz;						// current value of rotation about Z (deg)
	int _rorder;						// order of euler angles to reconstruct rotation
	std::vector<Joint*> _children;	// children of the current joint
    glm::vec3 _color;

public :
	// Constructor :
	Joint();
	// Destructor :
	~Joint() {
		_dofs.clear();
		_children.clear();
	}

    static Joint *parseHierarchy(std::ifstream &inputfile);
    static void parseMotion(std::ifstream &inputfile, Joint *root, uint32_t &frameCount, double &frameTime);
    void parseCurveKeyframes(std::ifstream &inputfile);
    void parseJoint(std::ifstream &inputfile);

    void buildSkeleton(std::vector<SimpleVertex> &vertices, std::vector<uint32_t> &indices, glm::vec3 O, glm::vec3 X, glm::vec3 Y, glm::vec3 Z) const;
    void buildSkeletonMatrices(std::vector<SimpleVertex> &vertices, std::vector<uint32_t> &indices, const glm::mat4 &transform);

	// Create from data :
	static Joint* create(std::string name, double offX, double offY, double offZ, Joint* parent) {
		Joint* child = new Joint();
		child->_name = name;
		child->_offX = offX;
		child->_offY = offY;
		child->_offZ = offZ;
		child->_curTx = 0;
		child->_curTy = 0;
		child->_curTz = 0;
		child->_curRx = 0;
		child->_curRy = 0;
		child->_curRz = 0;
		if(parent != NULL) {
			parent->_children.push_back(child);
		}
		return child;
	}

	// Load from file (.bvh) :	
	static Joint *createFromFile(std::string fileName, uint32_t &frameCount, double &frameTime);

    void animate(int iframe=0);
    void animateLerp(int iframe=0, double framePercent=0.);

	// Analysis of degrees of freedom :
	void nbDofs();

	// For Skinning
    void populateJointMap(std::unordered_map<std::string, Joint *> &jointMap);
	void transformMatrices(std::unordered_map<Joint *, glm::mat4> &matrices, const glm::mat4 &parentTransform);
	void transformMatricesBinding(std::unordered_map<Joint *, glm::mat4> &matrices, const glm::mat4 &parentTransform);
};


#endif
