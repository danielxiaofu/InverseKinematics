#pragma once
#include "BaseObject.h"
#include <vector>

#define DEG2RAD PI/180.0

// a simple struct that stores a 4 * 4 matrix in row-major order
struct Matrix
{
	double matrix[4][4];

	Matrix(double matrix_[4][4])
	{
		setMatrix(matrix_);
	}

	Matrix()
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				matrix[i][j] = 0.0;
	}

	void setMatrix(const double matrix_[4][4])
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				matrix[i][j] = matrix_[i][j];
	}

	void getMatrix(double outMat[4][4]) const
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				outMat[i][j] = matrix[i][j];
	}

	void multiplyByVector(double* result, const double* vector)
	{
		result[0] = 0.0;
		result[1] = 0.0;
		result[2] = 0.0;
		result[3] = 1.0;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result[i] += matrix[i][j] * vector[j];
	}

	Matrix operator * (const Matrix rhs)
	{
		double result[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result[i][j] = 0.0;

		double rhsMat[4][4];
		rhs.getMatrix(rhsMat);

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 4; k++)
					result[i][j] += matrix[i][k] * rhsMat[k][j];
			}
			
		return Matrix(result);
	}

};

class Joint :
	public BaseObject
{
public:
	Joint(const std::string& name);

	void reset(double time);

	// set initial position of the joint relative to its parent
	void initialize(double x, double y, double z);

	// set passed joint as the parent of this joint, also add to parent's children list
	void parentTo(Joint* parent_);

	void startDraw();

	void getPosition(Vector outPos) { VecCopy(outPos, pos); }

	void getRotation(Vector outRot) { VecCopy(outRot, rot); }

	void isolate();

	Joint* getParent() { return parent; }

	void setRotationX(double x) { rot[0] = x; }

	void setRotationY(double y) { rot[1] = y; }

	void setRotationZ(double z) { rot[2] = z; }

	void setPosition(double x, double y, double z) { setVector(pos, x, y, z); }

	// rotate around x axis by x degrees at next draw call
	void rotateX(double x) { dx = x; }

	// rotate around y axis by y degrees at next draw call
	void rotateY(double y) { dy = y; }

	// rotate around z axis by z degrees at next draw call
	void rotateZ(double z) { dz = z; }

	// set if world position will be tracked
	void setTrackWorldPos(bool b) { trackWorldPos = b; }

	void setDerivativeModeX(bool b) { derivativeModeX = b; }
	void setDerivativeModeY(bool b) { derivativeModeY = b; }
	void setDerivativeModeZ(bool b) { derivativeModeZ = b; }

	// get world position, only work if trackWorldPos is true 
	void getWorldPosition(Vector outPos);

	Joint* getChild(int index);

	int childCount() { return children.size(); }

	// return rotation matrix around x-axis, will return derivative of matrix when derivativeModeX is true
	Matrix getRotXMatrix();
	// return rotation matrix around y-axis, will return derivative of matrix when derivativeModeY is true
	Matrix getRotYMatrix();
	// return rotation matrix around z-axis, will return derivative of matrix when derivativeModeZ is true
	Matrix getRotZMatrix();

	Matrix getTranslateMatrix();

	// given transform of its parent, compute the global transform and world position of this joint and its children
	void computeGlobalTransform(Matrix parentTransform);

protected:
	
	Vector rot; // angle of rotation (x, y, z) of the joint relative to its parent , in degrees
	Vector pos; // position of joint relative to parent
	Vector xAxis = {1.0, 0.0, 0.0};

	double dx, dy, dz; // delta angle of rotation each frame (will be cleared after drawing)

	// if true, the world position of this joint will be calculated when calling draw method
	bool trackWorldPos;
	double worldPos[4];

	Joint* parent;
	std::vector<Joint*> children;

	// global transform matrix of this joint
	Matrix tranformMat;

	bool derivativeModeX;
	bool derivativeModeY;
	bool derivativeModeZ;

	void globalDraw();
	void localDraw();

	void addChild(Joint* child);

	
};

