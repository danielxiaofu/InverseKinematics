#include "SkeletonSimulator.h"
#include "BaseSystem.h"

SkeletonSimulator::SkeletonSimulator(const std::string & name, BaseSystem * target) :
	BaseSimulator(name),
	m_object(target)
{
	previous = 0.0;
}

int SkeletonSimulator::step(double time)
{
	double delta = time - previous;
	previous = time;
	double p[4] = {0.0, 0.0, 0.0, 0.0};
	Vector currentPosition;
	Vector targetPosition;
	setVector(targetPosition, 0.0, 2.0, -2.0);
	m_object->getState(p);
	p[0] = delta;
	setVector(currentPosition, p[1], p[2], p[3]);
	VecSubtract(velocity, targetPosition, currentPosition);
	VecNormalize(velocity);
	//VecScale(velocity, delta);
	VecScale(velocity, 0.95);
	VecAdd(currentPosition, currentPosition, velocity);
	p[1] = currentPosition[0];
	p[2] = currentPosition[1];
	p[3] = currentPosition[2];
	m_object->setState(p);

	return 0;
}

int SkeletonSimulator::init(double time)
{
	setVector(velocity, 50.0, 0.0, 0.0);
	previous = 0.0;
	return 0;
}
