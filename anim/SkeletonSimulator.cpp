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
	double p[1] = {0.0};
	m_object->getState(p);
	p[0] = delta;
	m_object->setState(p);

	return 0;
}

int SkeletonSimulator::init(double time)
{
	previous = 0.0;
	return 0;
}
