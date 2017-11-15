#pragma once
#include "BaseSimulator.h"

class BaseSystem;

class SkeletonSimulator :
	public BaseSimulator
{
public:
	SkeletonSimulator(const std::string& name, BaseSystem* target);
	~SkeletonSimulator();

protected:
	BaseSystem* m_object;
};

