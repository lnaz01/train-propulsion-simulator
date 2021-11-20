//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRAKE_PIPE_CUMULATIVE_DEF
#define BRAKE_PIPE_CUMULATIVE_DEF

#include <vector>

class BrakePipe_FiniteElement;
class LinearSystem;
class RailVehicle;
class TrainConsist;

class BrakePipe_Cumulative {

public:

	BrakePipe_Cumulative(TrainConsist* trainConsist);

	virtual ~BrakePipe_Cumulative();

	// Train consist
	TrainConsist* trainConsist;

	// Vector of brake pipe finite elements
	std::vector<BrakePipe_FiniteElement*> brakePipe_FiniteElements;

	// Vector of rail vehicles
	std::vector<RailVehicle*> railVehicles;

	// System matrix dimension
	int systemMatrixDim;

	// System matrix
	double** systemMatrix;

	// Forcing vector
	double* forcingVector;

	// Linear system
	LinearSystem* linearSystem;

	// Initializes system matrix and forcing vector
	void initializeSystemMatrixAndForcingVector();

	// Calculates vector of brake pipe finite elements
	void calc_brakePipe_FiniteElements();

	// Calculates array of rail vehicle types
	void calc_railVehicleTypes();

	// Calculates system matrix
	void calc_systemMatrix();

	// Calculates forcing vector
	void calc_forcingVector();

	// Updates pressure, 'm' value, velocity, and density values at brake pipe finite element nodes
	void updatePressureAndVelocityAtBrakePipeFiniteElementNodes();

private:

	// Array of rail vehicle types defined boolean
	bool railVehicleTypesDefinedBool;

	// Array of rail vehicle types ('0' for car; '1' for locomotive)
	int* railVehicleTypes;

	// System matrix and forcing vector space allocated boolean
	bool systemMatrixAndForcingVectorBoolean;

};

#endif
