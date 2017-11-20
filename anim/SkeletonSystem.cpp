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
	zeroVector(destination);
	transposeMode = false;
	threshold = 0.94;
	rightArmBend = 0.0;

	blackboard.ReadOBJ("data/blackboard.obj");
	floor.ReadOBJ("data/floor.obj");
	glmFacetNormals(&blackboard);
	glmVertexNormals(&blackboard, 90);
	glmFacetNormals(&floor);
	glmVertexNormals(&floor, 90);
}

void SkeletonSystem::getState(double * p)
{
	endEffector->getWorldPosition(endEffPos);
	p[0] = 0.0;
	p[1] = endEffPos[0];
	p[2] = endEffPos[1];
	p[3] = endEffPos[2];
}

void SkeletonSystem::setState(double * p)
{
	// compute velocity
	Vector newTarget;
	setVector(newTarget, p[1], p[2], p[3]);
	setVector(destination, p[4], p[5], p[6]);
	endEffector->getWorldPosition(endEffPos);
	VecSubtract(velocity, newTarget, endEffPos);

	// solve ik
	Vector targetP, error;
	VecAdd(targetP, endEffPos, velocity);
	while (true)
	{
		VecSubtract(error, targetP, endEffPos);
		solveIK(transposeMode);
		computeJointTransform();
		endEffector->getWorldPosition(endEffPos);
		double err = VecLength(error);
		//animTcl::OutputMessage("error = %f", err);
		if (err <= threshold)
		{
			transposeMode = false;
			break;
		}
		// if error is too large, switch to transpose Jacobian method
		else if (err > threshold * 4)
			transposeMode = true;

		VecSubtract(velocity, targetP, endEffPos);
		VecScale(velocity, 0.5);
	}
	
	// bend right arm
	if(rightArmBend > -45)
		rightArmBend -= 1.0;
	
}

void SkeletonSystem::reset(double time)
{
	zeroVector(destination);
	transposeMode = false;
	threshold = 0.94;
	rightArmBend = 0.0;
}

void SkeletonSystem::display(GLenum mode)
{
	computeJointTransform();

	// tell all joints to draw themselves
	root->startDraw();
	
	endEffector->getWorldPosition(endEffPos);
	//animTcl::OutputMessage("endEffPos = %.2f, %.2f, %.2f", endEffPos[0], endEffPos[1], endEffPos[2]);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glPointSize(9.0);
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_POINTS);
	glVertex3f(destination[0], destination[1], destination[2]);
	glVertex3f(endEffPos[0], endEffPos[1], endEffPos[2]);
	glEnd();
	glPopAttrib();
	
	//glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	// draw blackboard and floor
	glPushMatrix();
	glTranslated(0.0, 3.0, 0.0);
	glScaled(1.0, 1.0, 0.5);
	if (blackboard.numvertices > 0)
		glmDraw(&blackboard, GLM_SMOOTH);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0.0, -2.8, 2.0);
	glRotated(90, 1.0, 0.0, 0.0);
	if (floor.numvertices > 0)
		glmDraw(&floor, GLM_SMOOTH);
	glPopMatrix();
}

int SkeletonSystem::command(int argc, myCONST_SPEC char ** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name);
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "position") == 0)
	{
		if (argc == 4)
		{
			double x = atof(argv[1]);
			double y = atof(argv[2]);
			double z = atof(argv[3]);
			root->setPosition(x, y, z);
			computeJointTransform();
			glutPostRedisplay();
			return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: position < x y z>");
			return TCL_ERROR;
		}
	}
	return 0;
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

void SkeletonSystem::initialize(double shoulderX, double shoulderY, double shoulderZ, double elbowX, double elbowY, double wristY, double wristZ)
{
	setAngleParameter(shoulderX, shoulderY, shoulderZ, elbowX, elbowY, wristY, wristZ);
	computeJointTransform();
	endEffector->getWorldPosition(endEffPos);
	VecCopy(startPos, endEffPos);
}

void SkeletonSystem::setAngleParameter(double shoulderX, double shoulderY, double shoulderZ, double elbowX, double elbowY, double wristY, double wristZ)
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

void SkeletonSystem::setDestination(VectorObj & dest)
{
	setVector(destination, dest.x(), dest.y(), dest.z());
}

void SkeletonSystem::computeJointTransform()
{
	if (!root)
		return;

	// compute world transform and position for all joints
	double identity[4][4] =
	{
		{ 1.0, 0.0, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0, 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 }
	};
	Matrix I = Matrix(identity);
	root->computeGlobalTransform(I);
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
	leftCollar->setRotationX(t1);
	leftCollar->setRotationY(t2);
	leftCollar->setRotationZ(t3);
	leftShoulder->setRotationX(t4);
	leftShoulder->setRotationY(t5);
	leftElbow->setRotationY(t6);
	leftElbow->setRotationZ(t7);
	rightCollar->setRotationZ(rightArmBend);
	
}

void SkeletonSystem::computeMatrixSequence(double * result)
{
	Matrix resultMatrix = root->getTranslateMatrix() *
		spine->getTranslateMatrix() *
		leftCollar->getTranslateMatrix() *
		leftCollar->getRotZMatrix() *
		leftCollar->getRotYMatrix() *
		leftCollar->getRotXMatrix() *
		leftShoulder->getTranslateMatrix() *
		leftShoulder->getRotYMatrix() *
		leftShoulder->getRotXMatrix() *
		leftElbow->getTranslateMatrix() *
		leftElbow->getRotZMatrix() *
		leftElbow->getRotYMatrix();

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
	leftCollar->setDerivativeModeZ(false);
	leftCollar->setDerivativeModeY(false);
	leftCollar->setDerivativeModeX(false);
	leftShoulder->setDerivativeModeY(false);
	leftShoulder->setDerivativeModeX(false);
	leftElbow->setDerivativeModeY(false);
	leftElbow->setDerivativeModeZ(false);

	switch (index)
	{
	case 0:
		leftCollar->setDerivativeModeX(true);
		break;
	case 1:
		leftCollar->setDerivativeModeY(true);
		break;
	case 2:
		leftCollar->setDerivativeModeZ(true);
		break;
	case 3:
		leftShoulder->setDerivativeModeX(true);
		break;
	case 4:
		leftShoulder->setDerivativeModeY(true);
		break;
	case 5:
		leftElbow->setDerivativeModeY(true);
		break;
	case 6:
		leftElbow->setDerivativeModeZ(true);
		break;
	default:
		break;
	}
}

bool SkeletonSystem::computeInverseJJt()
{
	double result[4][4];

	for (int i = 1; i <= 3; i++)
		for (int j = 1; j <= 3; j++)
			result[i][j] = JJt[i-1][j-1];

	int P[4];

	if (LUPdecompose(result, P) < 0)
	{
		animTcl::OutputMessage("LUPdecompose failed, a singular matrix is supplied");
		return false;
	}
	double B[4][4], X[4], Y[4]; // tempory spaces
	LUPinverse(P, result, B, X, Y);

	for (int i = 1; i <= 3; i++)
		for (int j = 1; j <= 3; j++)
			inverseJJt[i-1][j-1] = result[i][j];

	return true;
}

void SkeletonSystem::solveIK(bool transposeMode)
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
	if (transposeMode)
	{
		for (int i = 0; i < 7; i++)
			for (int j = 0; j < 3; j++)
				dT[i] += transposeJ[i][j] * velocity[j];
	}
	else
	{
		for (int i = 0; i < 7; i++)
			for (int j = 0; j < 3; j++)
				dT[i] += transposeJ[i][j] * beta[j];
	}

	setAngleParameter(t1 + dT[0], t2 + dT[1], t3 + dT[2], t4 + dT[3], t5 + dT[4], t6 + dT[5], t7 + dT[6]);
}

