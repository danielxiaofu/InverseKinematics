#pragma once
#include "BaseSimulator.h"

class BaseSystem;
class Hermite;

class SkeletonSimulator :
	public BaseSimulator
{
public:
	SkeletonSimulator(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);
	int command(int argc, myCONST_SPEC char **argv);

	void setHermite(Hermite* hermite_);

protected:
	BaseSystem* m_object;
	Hermite* hermite;
	Vector velocity;
	VectorObj destination;

	double previous;

	double hermiteLength, currentT, currentLength, speed;

};

