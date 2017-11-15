#include "Joint.h"

Joint::Joint(const std::string & name) :
		BaseObject(name)
{
	length = 0.0;
	setVector(rotAxis, 0.0, 0.0, 0.0);
	rotAngle = 0.0;
}

void Joint::reset(double time)
{
}

void Joint::initialize(double length_, double rx, double ry, double rz, double angle)
{
	length = length_;
	setVector(rotAxis, rx, ry, rz);
	VecNormalize(rotAxis);
	rotAngle = angle;
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
		Vector rotAxis;
		double length, angle;
		length = child->getLength();
		angle = child->getRotation(rotAxis);

		glPushMatrix();
		glRotated(angle, rotAxis[0], rotAxis[1], rotAxis[2]);
		glLineWidth(3.0);
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < 360; i++)
		{
			float degInRad = i * DEG2RAD;
			glVertex2f(cos(degInRad) * length / 2 + length / 2, sin(degInRad) * length / 8);
		}
		glEnd();
		glTranslated(length, 0.0, 0.0); // align the child with its x-axis
		child->draw();
		glPopMatrix();
	}

}
