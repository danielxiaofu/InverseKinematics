#include "SkeletonSystem.h"
#include "Joint.h"
#include "LUDecompose.h"

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
	// compute velocity
	Vector origin;
	zeroVector(origin);
	//VecSubtract(velocity, origin, startPos);
	setVector(velocity, 50.0, 0.0, 0.0);
	//VecNormalize(velocity);
	double delta = p[0];
	VecScale(velocity, delta);
	solveIK();
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

void SkeletonSystem::setAngleParameter(double shoulderX, double shoulderY, double shoulderZ, double elbowX, double elbowY, double wristZ, double wristY)
{
	t1 = shoulderX;
	t2 = shoulderY;
	t3 = shoulderZ;
	t4 = elbowX;
	t5 = elbowY;
	t6 = wristZ;
	t7 = wristY;
	updateJointAngle();
}

void SkeletonSystem::updateJacobian()
{
	for (int i = 0; i < 7; i++)
	{
		double column4[4];
		turnOnDerivative(i);
		computeMatrixSequence(column4);
		jacobian[0][i] = column4[0];
		jacobian[1][i] = column4[1];
		jacobian[2][i] = column4[2];
	}

	// update transpose of jacobian
	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 3; j++)
			transposeJ[i][j] = jacobian[j][i];

	// update jJt
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			JJt[i][j] = 0.0;

	for (int j = 0; j < 3; j++)
		for (int k = 0; k < 3; k++)
			for (int i = 0; i < 7; i++)
				JJt[j][k] += jacobian[j][i] * transposeJ[i][k];

}

void SkeletonSystem::updateJointAngle()
{
	leftShoulder->setRotationX(t1);
	leftShoulder->setRotationY(t2);
	leftShoulder->setRotationZ(t3);
	leftElbow->setRotationX(t4);
	leftElbow->setRotationY(t5);
	endEffector->setRotationZ(t6);
	endEffector->setRotationY(t7);
}

void SkeletonSystem::computeMatrixSequence(double * result)
{
	Matrix resultMatrix = root->getTranslateMatrix() *
		spine->getTranslateMatrix() *
		leftCollar->getTranslateMatrix() *
		leftShoulder->getRotZMatrix() *
		leftShoulder->getRotYMatrix() *
		leftShoulder->getRotXMatrix() *
		leftShoulder->getTranslateMatrix() *
		leftElbow->getRotYMatrix() *
		leftElbow->getRotXMatrix() *
		leftElbow->getTranslateMatrix() *
		endEffector->getRotYMatrix() *
		endEffector->getRotZMatrix();

	double endEffLocalPos[4];
	Vector temp;
	endEffector->getPosition(temp);
	endEffLocalPos[0] = temp[0];
	endEffLocalPos[1] = temp[1];
	endEffLocalPos[2] = temp[2];
	endEffLocalPos[3] = 1.0;

	resultMatrix.multiplyByVector(result, endEffLocalPos);
}

void SkeletonSystem::turnOnDerivative(int index)
{
	// first turn off all derivative mode
	leftShoulder->setDerivativeModeZ(false);
	leftShoulder->setDerivativeModeY(false);
	leftShoulder->setDerivativeModeX(false);
	leftElbow->setDerivativeModeY(false);
	leftElbow->setDerivativeModeX(false);
	endEffector->setDerivativeModeY(false);
	endEffector->setDerivativeModeZ(false);

	switch (index)
	{
	case 0:
		leftShoulder->setDerivativeModeX(true);
		break;
	case 1:
		leftShoulder->setDerivativeModeY(true);
		break;
	case 2:
		leftShoulder->setDerivativeModeZ(true);
		break;
	case 3:
		leftElbow->setDerivativeModeX(true);
		break;
	case 4:
		leftElbow->setDerivativeModeY(true);
		break;
	case 5:
		endEffector->setDerivativeModeZ(true);
		break;
	case 6:
		endEffector->setDerivativeModeY(true);
		break;
	default:
		break;
	}
}

bool SkeletonSystem::computeInverseJJt()
{
	double result[3][3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			result[i][j] = JJt[i][j];

	int P[3];

	if (LUPdecompose(result, P) < 0)
	{
		animTcl::OutputMessage("LUPdecompose failed, a singular matrix is supplied");
		return false;
	}
	double B[3][3], X[3], Y[3]; // tempory spaces
	LUPinverse(P, result, B, X, Y);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			inverseJJt[i][j] = result[i][j];
	return true;
}

void SkeletonSystem::solveIK()
{
	updateJacobian();
	if (!computeInverseJJt()) return;

	double beta[3] = {0.0, 0.0, 0.0};

	// define beta = inverseJJt * velocity
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			beta[i] += inverseJJt[i][j] * velocity[j];

	// delta angles
	double dT[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// compute dT = transposeJ * beta
	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 3; j++)
			dT[i] += transposeJ[i][j] * beta[j];

	setAngleParameter(t1 + dT[0], t2 + dT[1], t3 + dT[2], t4 + dT[3], t5 + dT[4], t6 + dT[5], t7 + dT[6]);
}

