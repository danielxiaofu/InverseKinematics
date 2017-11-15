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

protected:
	Joint* root;
	Joint* currentJoint;
	Joint* endEffector;


};

