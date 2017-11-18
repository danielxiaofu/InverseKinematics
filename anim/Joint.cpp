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

void Joint::startDraw()
{
	draw();
}

void Joint::isolate()
{
	parent = NULL;
	children.clear();
}

void Joint::getWorldPosition(Vector outPos)
{
	if (!trackWorldPos)
	{
		zeroVector(outPos);
		return;
	}
	setVector(outPos, worldPos[0], worldPos[1], worldPos[2]);
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



void Joint::draw()
{

	for(Joint* child : children)
	{
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
		glRotated(childRot[2], 0.0, 0.0, 1.0);
		glRotated(childRot[1], 0.0, 1.0, 0.0);
		glRotated(childRot[0], 1.0, 0.0, 0.0);

		glPushMatrix();
		glRotated(drawAngle, 0.0, 0.0, 1.0);

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
		child->draw();
		glPopMatrix();
	}

	if (trackWorldPos)
	{
		//glPushMatrix();

		//glRotated(rot[0], 1.0, 0.0, 0.0);
		//glRotated(rot[1], 0.0, 1.0, 0.0);
		//glRotated(rot[2], 0.0, 0.0, 1.0);
		
		GLfloat mvMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, mvMatrix);

		worldPos[0] = mvMatrix[12];
		worldPos[1] = mvMatrix[13];
		worldPos[2] = 0.0;
	}

}
