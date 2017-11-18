#pragma once
#include "BaseSystem.h"
#include <vector>

class Joint;

class SkeletonSystem :
	public BaseSystem
{
public:
	SkeletonSystem(const std::string& name);
	virtual void getState(double *p);
	virtual void setState(double  *p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	/* add a joint under currentJoint, if there is no root joint, assign the joint to root. 
	 * Then assign the new joint to currentJoint
	 */
	void addJoint(Joint* joint);

	/* add an end effector under currentJoint if the root joint exsists.
	* Then assign the end effector to currentJoint
	*/
	void addEndEffector(Joint* endEff);

	Joint* getCurrentJoint() { return currentJoint; }

	// set currentJoint to its parent and return it
	Joint* traverseUp();

	void setSpine(Joint* spine_) { spine = spine_; }

	void setLeftCollar(Joint* leftCollar_) { leftCollar = leftCollar_; }

	void setLeftShoulder(Joint* leftShoulder_) { leftShoulder = leftShoulder_; }

	void setLeftElbow(Joint* leftElbow_) { leftElbow = leftElbow_; }

	// get initial position of endEff
	void getInitialPos(Vector outPos);

	void setAngleParameter(double shoulderX, double shoulderY, double shoulderZ, double elbowX, double elbowY, double wristZ, double wristY);

	// recalculate jacobian matrix based on current joint angles
	void updateJacobian();

protected:
	Joint* currentJoint;

	Joint* root;
	Joint* spine;
	Joint* leftCollar;
	Joint* leftShoulder;
	Joint* leftElbow;
	Joint* endEffector;

	// initial position of endEff
	Vector startPos;
	// global position of endEff (extracted from model view matrix)
	Vector endEffPos;
	// destination of endEff next frame
	Vector destination;
	Vector velocity;

	/* seven angle parameters that define the position of end effector
	 * shoulder: rx(t1), ry(t2), rz(t3)
	 * elbow: rx(t4), ry(t5)
	 * wrist: ry(t7), rz(t6)
	 */
	double t1, t2, t3, t4, t5, t6, t7;

	bool firstDisplay;

	double jacobian[3][7];
	double transposeJ[7][3];
	double JJt[3][3];
	double inverseJJt[3][3];

	void updateJointAngle();

	/* compute matrix sequence that form the column of jacobian
	 * Troot * Tspine * Tcollar * RshoulderZ * RshoulderY * RshoulderX * Tshoulder * RelbowY * RelbowX * Telbow * RwristY * RwristZ * Twrist
	 */
	void computeMatrixSequence(double* result);

	/* turn on derivative mode of an angle parameter specified by index
	 * index should be between 0 and 6, otherwise all derivative mode will be turn off
	 */
	void turnOnDerivative(int index);

	bool computeInverseJJt();

	void solveIK();
};

