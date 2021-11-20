//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef VECTOR_ROTATOR_DEF
#define VECTOR_ROTATOR_DEF

class VectorRotator {

public:

	VectorRotator();

	virtual ~VectorRotator();

	// Rotated vector
	double* rotatedVector;

	// XYZ rotation for three-dimensional vectors
	// vec	-->	Vector to be rotated
	// x	-->	X axis rotation (in radians)
	// y	-->	Y axis rotation (in radians)
	// z	-->	Z axis rotation (in radians)
	// (NOTE: Order of rotations is actually z, y, x -- NOT x, y, z)
	// (NOTE: 'this' vector should contain 3 elements)
	void rotateVector(double* vec, double x, double y, double z);

private:

	// Number of dimensions
	const int NUMBER_OF_DIMENSIONS = 3;

	// Rotation matrix
	double** rotationMatrix;

};

#endif
