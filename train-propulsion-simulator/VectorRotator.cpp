//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <cmath>
#include "VectorRotator.h"


VectorRotator::VectorRotator() {
	// Allocate space for rotation matrix
	rotationMatrix = new double* [NUMBER_OF_DIMENSIONS];
	for (int i = 0; i < NUMBER_OF_DIMENSIONS; i++) {
		rotationMatrix[i] = new double[NUMBER_OF_DIMENSIONS];
	}
	// Allocate space for rotated vector
	rotatedVector = new double[NUMBER_OF_DIMENSIONS];
}


VectorRotator::~VectorRotator() {
	// Delete rotation matrix 'rotationMatrix'
	for (int i = 0; i < NUMBER_OF_DIMENSIONS; i++) {
		delete[] rotationMatrix[i];
	}
	delete[] rotationMatrix;
	// Delete rotated vector
	delete[] rotatedVector;
}


void VectorRotator::rotateVector(double* vec, double x, double y, double z) {
	// Rotation matrix
	rotationMatrix[0][0] = cos(y) * cos(z);
	rotationMatrix[0][1] = -cos(y) * sin(z);
	rotationMatrix[0][2] = sin(y);
	rotationMatrix[1][0] = (cos(z) * sin(x) * sin(y)) + (cos(x) * sin(z));
	rotationMatrix[1][1] = (cos(x) * cos(z)) - (sin(x) * sin(y) * sin(z));
	rotationMatrix[1][2] = -cos(y) * sin(x);
	rotationMatrix[2][0] = (-cos(x) * cos(z) * sin(y)) + (sin(x) * sin(z));
	rotationMatrix[2][1] = (cos(z) * sin(x)) + (cos(x) * sin(y) * sin(z));
	rotationMatrix[2][2] = cos(x) * cos(y);
	// Calculate rotated vector
	int nrows = 3;
	int ncols = 3;
	for (int i = 0; i < nrows; i++) {
		double sum = 0.0;
		for (int k = 0; k < ncols; k++) {
			sum = sum + (rotationMatrix[i][k] * vec[k]);
		}
		rotatedVector[i] = sum;
	}
}

