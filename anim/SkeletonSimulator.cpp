#include "SkeletonSimulator.h"
#include "BaseSystem.h"

SkeletonSimulator::SkeletonSimulator(const std::string & name, BaseSystem * target) :
	BaseSimulator(name),
	m_object(target)
{
}

SkeletonSimulator::~SkeletonSimulator()
{
}
