//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <cmath>
#include "LinearSystem.h"


LinearSystem::LinearSystem() {
	stateSpaceVectorCalcBool = false;
}


LinearSystem::~LinearSystem() {
	// Delete state space vector, if necessary
	if (stateSpaceVectorCalcBool == true) {
		delete[] stateSpaceVector;
	}
}


void LinearSystem::gaussElimination(double** systemMatrix, double* forcingVector, int dimension) {
	// Make copy of system matrix and forcing vector
	double** systemMatrixCOPY = new double* [dimension];
	for (int i = 0; i < dimension; i++) {
		systemMatrixCOPY[i] = new double[dimension];
	}
	double* forcingVectorCOPY = new double[dimension];
	for (int i = 0; i < dimension; i++) {
		for (int j = 0; j < dimension; j++) {
			systemMatrixCOPY[i][j] = systemMatrix[i][j];
		}
		forcingVectorCOPY[i] = forcingVector[i];
	}
	// Forward elimination
	for (int ic = 0; ic < (dimension - 1); ic++) {
		// Find pivot row ('pivr')
		int pivr = ic;
		for (int i = ic; i < dimension; i++) {
			if ((std::abs(systemMatrixCOPY[i][ic])) > std::abs(systemMatrixCOPY[pivr][ic])) {
				pivr = i;
			}
		}
		// Interchange rows, if necessary
		if (pivr != ic) {
			// Interchange two rows
			double* pivr_row_index_address = systemMatrixCOPY[pivr];
			double* ic_row_index_address = systemMatrixCOPY[ic];
			systemMatrixCOPY[pivr] = ic_row_index_address;
			systemMatrixCOPY[ic] = pivr_row_index_address;
			// Interchange elements of forcing vector
			double temp1 = forcingVectorCOPY[ic];
			forcingVectorCOPY[ic] = forcingVectorCOPY[pivr];
			forcingVectorCOPY[pivr] = temp1;
		}
		// Update elements of upper triangular matrix
		for (int ir = (ic + 1); ir < dimension; ir++) {
			double multnum = systemMatrixCOPY[ir][ic];
			double multden = systemMatrixCOPY[ic][ic];
			if (multden == 0.0) {
				// If this condition is met, an unchecked exception will occur, and the program will crash.
			}
			for (int i = 0; i <= ic; i++) {
				systemMatrixCOPY[ir][i] = 0.0;
			}
			for (int i = (ic + 1); i < dimension; i++) {
				double temp = systemMatrixCOPY[ir][i] + ((-multnum / multden) * systemMatrixCOPY[ic][i]);
				systemMatrixCOPY[ir][i] = temp;
			}
			double temp = forcingVectorCOPY[ir] + ((-multnum / multden) * forcingVectorCOPY[ic]);
			forcingVectorCOPY[ir] = temp;
		}
	}
	// Back substitution
	if (stateSpaceVectorCalcBool == true) {
		delete[] stateSpaceVector;
	}
	stateSpaceVector = new double[dimension];
	for (int ir = (dimension - 1); ir >= 0; ir--) {
		double sum = 0.0;
		if (ir == (dimension - 1)) {
			stateSpaceVector[ir] = forcingVectorCOPY[ir] / systemMatrixCOPY[ir][ir];
		}
		else {
			for (int ic = (ir + 1); ic < dimension; ic++) {
				sum = sum + (systemMatrixCOPY[ir][ic] * stateSpaceVector[ic]);
			}
			stateSpaceVector[ir] = (forcingVectorCOPY[ir] - sum) / systemMatrixCOPY[ir][ir];
		}
	}
	stateSpaceVectorCalcBool = true;
	// Delete copy of system matrix and forcing vector
	for (int i = 0; i < dimension; i++) {
		delete[] systemMatrixCOPY[i];
	}
	delete[] systemMatrixCOPY;
	delete[] forcingVectorCOPY;
}

