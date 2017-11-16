#pragma once
#include "BaseSystem.h"
#include <vector>

class Joint;

class SkeletonSystem :
	public BaseSystem
{
public:
	SkeletonSystem(const std::string& name);
	virtual void getState(double *p);
	virtual void setState(double  *p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	/* add a joint under currentJoint, if there is no root joint, assign the joint to root. 
	 * Then assign the new joint to currentJoint
	 */
	void addJoint(Joint* joint);

	/* add an end effector under currentJoint if the root joint exsists.
	* Then assign the end effector to currentJoint
	*/
	void addEndEffector(Joint* endEff);

	Joint* getCurrentJoint() { return currentJoint; }

	// set currentJoint to its parent and return it
	Joint* traverseUp();

	void setLeftShoulder(Joint* leftShoulder_) { leftShoulder = leftShoulder_; }

	void setLeftElbow(Joint* leftElbow_) { leftElbow = leftElbow_; }

	// get initial position of endEff
	void getInitialPos(Vector outPos);

protected:
	Joint* root;
	Joint* currentJoint;

	Joint* leftShoulder;
	Joint* leftElbow;
	Joint* endEffector;

	// initial position of endEff
	Vector startPos;

	/* seven rotation angles that define the position of end effector
	 * shoulder: rx(t1), ry(t2), rz(t3)
	 * elbow: rx(t4), ry(t5)
	 * wrist: ry(t6), rz(t7)
	 */
	double t1, t2, t3, t4, t5, t6, t7;

};

