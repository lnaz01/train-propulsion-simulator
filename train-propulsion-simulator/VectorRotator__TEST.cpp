//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "UnitConverter.h"
#include "VectorRotator.h"


void rotateVector__TEST(double* vec, int vecSize);


int main() {

	int vecSize = 3;
	double* vec = new double[vecSize];
	//vec[0] = 0.0;
	//vec[1] = 0.0;
	//vec[2] = -100.0;
	vec[0] = 0.0;
	vec[1] = -10.0;
	vec[2] = 0.0;
	rotateVector__TEST(vec, vecSize);
	delete[] vec;
	return 0;

}


void rotateVector__TEST(double* vec, int vecSize) {
	std::cout << "rotateVector__TEST: " << std::endl;
	// Print original vector
	std::cout << "vector: ";
	for (int i = 0; i < vecSize; i++) {
		std::cout << vec[i];
		if (i == (vecSize - 1)) {
			std::cout << std::endl;
		}
		else {
			std::cout << ", ";
		}
	}
	// Vector rotator
	VectorRotator* vectorRotator = new VectorRotator;
	// First rotation
	//vectorRotator->rotateVector(vec, M_PI / 2.0, 0.0, 0.0);
	//vectorRotator->rotateVector(vec, 0.05235986, 0.0, 0.0);
	vectorRotator->rotateVector(vec, 0.0, 0.05235986, 0.0);
	//vectorRotator->rotateVector(vec, 0.0, 0.0, M_PI / 4.0);
	// Print first rotation
	std::cout << "rotation 1: ";
	for (int i = 0; i < vecSize; i++) {
		std::cout << vectorRotator->rotatedVector[i];
		if (i == (vecSize - 1)) {
			std::cout << std::endl;
		}
		else {
			std::cout << ", ";
		}
	}
	// Delete vector rotator
	delete vectorRotator;
}

