#include "SkeletonSystem.h"
#include "Joint.h"

SkeletonSystem::SkeletonSystem(const std::string & name)
	: BaseSystem(name)
{
	root = NULL;
	endEffector = NULL;
}

void SkeletonSystem::getState(double * p)
{
}

void SkeletonSystem::setState(double * p)
{
}

void SkeletonSystem::reset(double time)
{
}

void SkeletonSystem::display(GLenum mode)
{
	Vector endEffPos;
	getInitialPos(endEffPos);
	animTcl::OutputMessage("pos = %f, %f, %f", endEffPos[0], endEffPos[1], endEffPos[2]);
		
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	if (root)
		root->startDraw();
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex3d(endEffPos[0], endEffPos[1], endEffPos[2]);
	glEnd();
	glPopMatrix();

}

void SkeletonSystem::addJoint(Joint * joint)
{
	joint->isolate();
	if (!root)
	{
		root = joint;
		currentJoint = root;
	}
	else
	{
		joint->parentTo(currentJoint);
		currentJoint = joint;
	}
}

void SkeletonSystem::addEndEffector(Joint * endEff)
{
	if (root)
	{
		addJoint(endEff);
		endEffector = endEff;
	}
}

Joint * SkeletonSystem::traverseUp()
{
	if (currentJoint == root)
		return currentJoint;
	else
	{
		currentJoint = currentJoint->getParent();
		return currentJoint;
	}
}

void SkeletonSystem::getInitialPos(Vector outPos)
{
	if (!endEffector)
	{
		zeroVector(outPos);
		return;
	}		
	Vector result;
	zeroVector(result);
	Joint* parent = endEffector;
	while (parent != root)
	{
		Vector parentPos;
		parent->getPosition(parentPos);
		VecAdd(result, result, parentPos);
		parent = parent->getParent();
	}
	VecCopy(outPos, result);
}

