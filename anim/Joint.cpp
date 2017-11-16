#include "Joint.h"

Joint::Joint(const std::string & name) :
		BaseObject(name)
{
	setVector(pos, 0.0, 0.0, 0.0);
	setVector(rot, 0.0, 0.0, 0.0);
	setVector(xAxis, 1.0, 0.0, 0.0);
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

void Joint::draw()
{
	if (children.size() == 0)
		return;

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
		glRotated(childRot[0], 1.0, 0.0, 0.0);
		glRotated(childRot[1], 0.0, 1.0, 0.0);
		glRotated(childRot[2], 0.0, 0.0, 1.0);

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

}
