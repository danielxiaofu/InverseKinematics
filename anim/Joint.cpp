#include "Joint.h"

Joint::Joint(const std::string & name) :
		BaseObject(name)
{
	setVector(pos, 0.0, 0.0, 0.0);
	setVector(rot, 0.0, 0.0, 0.0);
	setVector(xAxis, 1.0, 0.0, 0.0);
	trackWorldPos = false;
	worldPos[0] = 0.0;
	worldPos[1] = 0.0;
	worldPos[2] = 0.0;
	worldPos[3] = 1.0;
	derivativeModeX = derivativeModeY = derivativeModeZ = false;
	parent = nullptr;
}

void Joint::reset(double time)
{
}

void Joint::initialize(double x, double y, double z)
{
	setVector(pos, x, y, z);
}

void Joint::parentTo(Joint * parent_)
{
	parent = parent_;
	parent->addChild(this);
}

void Joint::addChild(Joint * child)
{
	children.push_back(child);
}

void Joint::computeGlobalTransform(Matrix parentTransform)
{
	derivativeModeX = derivativeModeY = derivativeModeZ = false;

	Matrix allRotation = parentTransform * getRotZMatrix() * getRotYMatrix() * getRotXMatrix();
	double temp[4] = { 0.0, 0.0, 0.0, 1.0 };
	//allRotation.multiplyByVector(worldPos, temp);

	tranformMat = parentTransform * getTranslateMatrix() * getRotZMatrix() * getRotYMatrix() * getRotXMatrix();
	tranformMat.multiplyByVector(worldPos, temp);

	for (Joint* child : children)
		child->computeGlobalTransform(tranformMat);

}

void Joint::startDraw()
{
	globalDraw();

	glPushMatrix();
	glTranslated(pos[0], pos[1], pos[2]);
	glRotated(rot[2], 0.0, 0.0, 1.0);
	glRotated(rot[1], 0.0, 1.0, 0.0);
	glRotated(rot[0], 1.0, 0.0, 0.0);
	localDraw();
	glPopMatrix();
}

void Joint::isolate()
{
	parent = NULL;
	children.clear();
}

void Joint::getWorldPosition(Vector outPos)
{
	setVector(outPos, worldPos[0], worldPos[1], worldPos[2]);
}

Joint * Joint::getChild(int index)
{
	if (index < 0 || index >= childCount())
		return nullptr;

	return children[index];
}

Matrix Joint::getRotXMatrix()
{
	double cosX = cos(rot[0] * DEG2RAD);
	double sinX = sin(rot[0] * DEG2RAD);
	if (!derivativeModeX)
	{
		double matrix[4][4] = 
		{
			{1.0, 0.0, 0.0, 0.0},
			{0.0, cosX, -sinX, 0.0},
			{0.0, sinX, cosX, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};
		return Matrix(matrix);
	}
	else
	{
		double matrixDrv[4][4] =
		{
			{0.0, 0.0, 0.0, 0.0},
			{0.0, -sinX, -cosX, 0.0},
			{0.0, cosX, -sinX, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};
		return Matrix(matrixDrv);
	}
}

Matrix Joint::getRotYMatrix()
{
	double cosY = cos(rot[1] * DEG2RAD);
	double sinY = sin(rot[1] * DEG2RAD);
	if (!derivativeModeY)
	{
		double matrix[4][4] =
		{
			{ cosY, 0.0, sinY, 0.0 },
			{ 0.0, 1.0, 0.0, 0.0 },
			{ -sinY, 0.0, cosY, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 }
		};
		return Matrix(matrix);
	}
	else
	{
		double matrixDrv[4][4] =
		{
			{ -sinY, 0.0, cosY, 0.0 },
			{ 0.0, 0.0, 0.0, 0.0 },
			{ -cosY, 0.0, -sinY, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 }
		};
		return Matrix(matrixDrv);
	}
}

Matrix Joint::getRotZMatrix()
{
	double cosZ = cos(rot[2] * DEG2RAD);
	double sinZ = sin(rot[2] * DEG2RAD);
	if (!derivativeModeY)
	{
		double matrix[4][4] =
		{
			{ cosZ, -sinZ, 0.0, 0.0 },
			{ sinZ, cosZ, 0.0, 0.0 },
			{ 0.0, 0.0, 1.0, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 }
		};
		return Matrix(matrix);
	}
	else
	{
		double matrixDrv[4][4] =
		{
			{ -sinZ, -cosZ, 0.0, 0.0 },
			{ cosZ, -sinZ, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 }
		};
		return Matrix(matrixDrv);
	}
}

Matrix Joint::getTranslateMatrix()
{
	double matrixDrv[4][4] =
	{
		{ 1.0, 0.0, 0.0, pos[0] },
		{ 0.0, 1.0, 0.0, pos[1] },
		{ 0.0, 0.0, 1.0, pos[2] },
		{ 0.0, 0.0, 0.0, 1.0 }
	};
	return Matrix(matrixDrv);
}



void Joint::globalDraw()
{
	// method1: draw child
	/*Vector worldPosition;
	getWorldPosition(worldPosition);

	for (Joint* child : children)
	{
		Vector childPos;
		child->getWorldPosition(childPos);

		glLineWidth(3.0);
		glBegin(GL_LINES);
		glVertex3d(worldPosition[0], worldPosition[1], worldPosition[2]);
		glVertex3d(childPos[0], childPos[1], childPos[2]);
		glEnd();
		child->globalDraw();
	
	}*/
	
	//method2: draw self
	if (parent)
	{
		Vector parentPos, pos;
		parent->getWorldPosition(parentPos);
		getWorldPosition(pos);
		glColor3f(0.0, 1.0, 0.0);
		glLineWidth(3.0);
		glBegin(GL_LINES);
		glVertex3f(parentPos[0], parentPos[1], parentPos[2]);
		glVertex3f(pos[0], pos[1], pos[2]);
		glEnd();
	}
	for (Joint* child : children)
		child->globalDraw();
}

void Joint::localDraw()
{
	for (Joint* child : children)
	{
		// use local position to draw
		Vector childRot, childPos;
		double length, dot;
		child->getPosition(childPos);
		child->getRotation(childRot);

		Vector toChild;
		VecCopy(toChild, childPos);
		length = VecLength(toChild);
		VecNormalize(toChild);

		dot = VecDotProd(toChild, xAxis);
		double drawAngle = acos(dot) / (DEG2RAD);
		if (childPos[1] < 0)
		drawAngle *= -1;

		glPushMatrix();
		
		glPushMatrix();
		glRotated(drawAngle, 0.0, 0.0, 1.0);
		glColor3f(0.0, 1.0, 0.0);
		glLineWidth(3.0);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 360; i++)
		{
		float degInRad = i * DEG2RAD;
		glVertex2f(cos(degInRad) * length / 2 + length / 2, sin(degInRad) * length / 8);
		}
		glEnd();

		glPopMatrix();

		glTranslated(childPos[0], childPos[1], childPos[2]);
		glRotated(childRot[2], 0.0, 0.0, 1.0);
		glRotated(childRot[1], 0.0, 1.0, 0.0);
		glRotated(childRot[0], 1.0, 0.0, 0.0);
		child->localDraw();
		glPopMatrix();
	}


}
