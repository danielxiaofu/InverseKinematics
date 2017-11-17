#include "SkeletonSystem.h"
#include "Joint.h"

SkeletonSystem::SkeletonSystem(const std::string & name)
	: BaseSystem(name)
{
	root = NULL;
	endEffector = NULL;
	t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0.0;
	zeroVector(endEffPos);
	firstDisplay = true;
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
	glPushMatrix();
	if (root)
		root->startDraw();
	endEffector->getWorldPosition(endEffPos);
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex3d(endEffPos[0], endEffPos[1], endEffPos[2]);
	glEnd();
	glPopMatrix();

	// when the skeleton is drawn first time (before camera is rotated), store the endEff position to startPos
	if (firstDisplay)
	{
		VecCopy(startPos, endEffPos);
		animTcl::OutputMessage("start position = %f, %f, %f", startPos[0], startPos[1], startPos[2]);
		firstDisplay = false;
	}
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

void SkeletonSystem::setInitialAngles(double shoulderX, double shoulderY, double shoulderZ, double elbowX, double elbowY, double wristY, double wristZ)
{
	t1 = shoulderX;
	t2 = shoulderY;
	t3 = shoulderZ;
	t4 = elbowX;
	t5 = elbowY;
	t6 = wristY;
	t7 = wristZ;
	updateJointAngle();
}

void SkeletonSystem::updateJacobian()
{

}

void SkeletonSystem::updateJointAngle()
{
	leftShoulder->setRotationX(t1);
	leftShoulder->setRotationY(t2);
	leftShoulder->setRotationZ(t3);
	leftElbow->setRotationX(t4);
	leftElbow->setRotationY(t5);
	endEffector->setRotationY(t6);
	endEffector->setRotationZ(t7);
}

void SkeletonSystem::computeMatrixSequence(double * result)
{
}

