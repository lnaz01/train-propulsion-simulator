//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LINEAR_SYSTEM_DEF
#define LINEAR_SYSTEM_DEF

class LinearSystem {

public:

	LinearSystem();

	virtual ~LinearSystem();

	// State space vector
	double* stateSpaceVector;

	// Gaussian elimination with partial pivoting
	// systemMatrix		--> System matrix
	// forcingVector	--> Forcing vector
	// dimension		-->	Number of rows (or columns) in square system matrix
	void gaussElimination(double** systemMatrix, double* forcingVector, int dimension);

private:

	// State space vector calculated boolean
	bool stateSpaceVectorCalcBool;

};

#endif
