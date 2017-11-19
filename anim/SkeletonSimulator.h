#pragma once
#include "BaseSimulator.h"

class BaseSystem;

class SkeletonSimulator :
	public BaseSimulator
{
public:
	SkeletonSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);

protected:
	BaseSystem* m_object;
	Vector velocity;

	double previous;
};

