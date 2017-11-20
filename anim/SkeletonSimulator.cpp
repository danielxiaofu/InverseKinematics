#include "SkeletonSimulator.h"
#include "BaseSystem.h"
#include "Hermite.h"
#include "SkeletonSystem.h"

SkeletonSimulator::SkeletonSimulator(const std::string & name, BaseSystem * target) :
	BaseSimulator(name),
	m_object(target)
{
	previous = 0.0;
	hermite = nullptr;
	speed = 2.0;
	hermiteLength = currentT = currentLength = 0.0;
	destination = VectorObj(0.0, 0.0, 0.0);
	zeroVector(velocity);
}

int SkeletonSimulator::step(double time)
{
	double delta = time - previous;
	previous = time;
	
	// make sure the hermite length is not 0
	if (abs(hermiteLength - 0.0) <= DBL_EPSILON)
		return 0;

	double p[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	Vector currentPosition;
	Vector targetPosition;
	setVector(targetPosition, 0.0, 2.0, -2.0);

	// move along curve
	if(currentLength < hermiteLength - 0.05)
		currentLength += speed * delta;
	currentT = hermite->getTFromArcLength(currentLength);
	currentT = currentT >= 1.0 ? 1.0 : currentT;

	hermite->getPoint(destination, currentT);

	m_object->getState(p);
	p[0] = delta;
	setVector(currentPosition, p[1], p[2], p[3]);
	setVector(targetPosition, destination.x(), destination.y(), destination.z());
	VecSubtract(velocity, targetPosition, currentPosition);
	VecNormalize(velocity);
	//VecScale(velocity, delta);
	VecScale(velocity, 1.0);
	VecAdd(currentPosition, currentPosition, velocity);
	p[1] = currentPosition[0];
	p[2] = currentPosition[1];
	p[3] = currentPosition[2];
	p[4] = destination.x();
	p[5] = destination.y();
	p[6] = destination.z();
	m_object->setState(p);

	return 0;
}

int SkeletonSimulator::init(double time)
{
	return 0;
}

int SkeletonSimulator::command(int argc, myCONST_SPEC char ** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name);
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (!hermite)
		{
			animTcl::OutputMessage("Hermite class not found");
			return TCL_ERROR;
		}

		if (argc == 2)
		{
			hermite->loadFromFile2D(argv[1]);
			hermite->setStartingArclength(0.0);
			hermiteLength = hermite->getLength();
			animTcl::OutputMessage("Hermite loaded, length = %f", hermiteLength);
			currentLength = hermite->getArcLengthFromT(0.0);
			currentT = hermite->getTFromArcLength(currentLength);
			hermite->getPoint(destination, currentT);
			dynamic_cast<SkeletonSystem*>(m_object)->setDestination(destination);
			dynamic_cast<SkeletonSystem*>(m_object)->initialize(0, 0, 45, 0, 0, 0, 45);
			glutPostRedisplay();
			return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: read <file_name>");
			return TCL_ERROR;
		}
	}
	return 0;
}

void SkeletonSimulator::reset(double time)
{
	previous = 0.0;
	speed = 2.0;
	hermiteLength = currentT = currentLength = 0.0;
	hermite->setStartingArclength(0.0);
	hermiteLength = hermite->getLength();
	currentLength = hermite->getArcLengthFromT(0.0);
	currentT = hermite->getTFromArcLength(currentLength);
	hermite->getPoint(destination, currentT);
	dynamic_cast<SkeletonSystem*>(m_object)->setDestination(destination);
	dynamic_cast<SkeletonSystem*>(m_object)->initialize(0, 0, 45, 0, 0, 0, 45);
	glutPostRedisplay();
}

void SkeletonSimulator::setHermite(Hermite* hermite_)
{
	hermite = hermite_;
}
