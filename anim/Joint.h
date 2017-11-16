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

	// set initial position of the joint relative to its parent
	void initialize(double x, double y, double z);

	// set passed joint as the parent of this joint, also add to parent's children list
	void parentTo(Joint* parent_);

	void startDraw();

	void getPosition(Vector outPos) { VecCopy(outPos, pos); }

	void getRotation(Vector outRot) { VecCopy(outRot, rot); }

	void isolate();

	Joint* getParent() { return parent; }

	void setRotateX(double x) { rot[0] = x; }

	void setRotateY(double y) { rot[1] = y; }

	void setRotateZ(double z) { rot[2] = z; }

	// rotate around x axis by x degrees at next draw call
	void rotateX(double x) { dx = x; }

	// rotate around y axis by y degrees at next draw call
	void rotateY(double y) { dy = y; }

	// rotate around z axis by z degrees at next draw call
	void rotateZ(double z) { dz = z; }

protected:
	
	Vector rot; // angle of rotation (x, y, z) of the joint relative to its parent , in degrees
	Vector pos; // position of joint relative to parent
	Vector xAxis = {1.0, 0.0, 0.0};

	double dx, dy, dz; // delta angle of rotation each frame (will be cleared after drawing)

	Joint* parent;
	std::vector<Joint*> children;

	void draw();

	void addChild(Joint* child);
};

