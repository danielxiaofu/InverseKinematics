#pragma once
#include "BaseObject.h"
#include <vector>

#define DEG2RAD PI/180.0

class Joint :
	public BaseObject
{
public:
	Joint(const std::string& name);

	void reset(double time);

	void initialize(double length_, double rx, double ry, double rz, double angle);

	// set passed joint as the parent of this joint, also add to parent's children list
	void parentTo(Joint* parent_);

	void startDraw();

	double getLength() { return length; }

	double getRotation(Vector outRotAxis) { VecCopy(outRotAxis, rotAxis); return rotAngle; }

	void isolate();

	Joint* getParent() { return parent; }

protected:
	
	Vector rotAxis; // rotation axis of the joint relative to parent

	double rotAngle; // in degrees
	double length; // distance between this joint and its parent (or length of bone)

	Joint* parent;
	std::vector<Joint*> children;

	void draw();

	void addChild(Joint* child);
};

