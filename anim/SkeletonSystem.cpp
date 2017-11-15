#include "SkeletonSystem.h"
#include "Joint.h"

SkeletonSystem::SkeletonSystem(const std::string & name)
	: BaseSystem(name)
{
	root = NULL;
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
	glMatrixMode(GL_MODELVIEW);
	if (root)
		root->startDraw();
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

