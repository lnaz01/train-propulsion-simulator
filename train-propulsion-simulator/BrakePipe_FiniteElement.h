//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRAKE_PIPE_FINITE_ELEMENT
#define BRAKE_PIPE_FINITE_ELEMENT

#include <vector>

class BrakePipe_Cumulative;
class RailVehicle;

class BrakePipe_FiniteElement {

public:

	BrakePipe_FiniteElement(BrakePipe_Cumulative* brakePipe_Cumulative, int brakePipeFEIndex);

	virtual ~BrakePipe_FiniteElement();

	// Maximum number of rail vehicles to a brake pipe finite element
	static const int MAX_RAIL_VEHICLES_PER_BRAKE_PIPE_FE = 3;

	// Number of nodes
	static const int NUMBER_OF_NODES_PER_BRAKE_PIPE_FE = 2;

	// Brake pipe diameter (meters)
	static const double DIAMETER;

	// Vector of rail vehicles
	std::vector<RailVehicle*> railVehicles;

	// Leakage
	double leakage;

	// Array of rail vehicle types ('0' for car; '1' for locomotive)
	int* railVehicleTypes;

	// Brake pipe finite element length
	double brakePipeFELength;

	// Node Diameters
	double* nodeDiameters;

	// Node Areas
	double* nodeAreas;

	// Node pressures
	double* nodePressures;

	// Node velocities
	double* nodeVelocities;

	// Node densities
	double* nodeDensities;

	// Node 'm' values
	double* nodeMValues;

	// Node 'c' values
	double* nodeCValues;

	// Effective 'c' value (which is the arithmetic mean of the 'c' value of the two nodes)
	double effCValue;

	// Effective diameter (which is the harmonic mean of the diameter at the two nodes)
	double effDiameter;

	// Effective area (which is the harmonic mean of the area at the two nodes)
	double effArea;

	// Calculates leakage
	void calc_leakage();

	// Calculates array of rail vehicle types
	void calc_railVehicleTypes();

	// Calculate X location of rail vehicles along brake pipe finite element
	void calc_railVehicleXLocations();

	// Calculates brake pipe finite element length
	void calc_brakePipeFELength();

	// Calculates node velocities
	void calc_nodeVelocities();

	// Calculates node densities
	void calc_nodeDensities();

	// Calculates node areas
	void calc_nodeAreas();

	// Calculates node 'c' values
	void calc_nodeCValues();

	// Calculates effective 'c' value
	void calc_effCValue();

	// Calculates effective diameter
	void calc_effDiameter();

	// Calculates effective area
	void calc_effArea();

	// Calculate interpolated brake pipe pressure values for each rail vehicle
	void calculateBrakePipePressureForRailVehicles();

	// Calculates first node reynolds number
	void calc_nodeReynoldsNumbers();

	// Calculates first node wall friction factor
	void calc_nodeWallFrictionFactors();

private:

	// Cumulative brake pipe
	BrakePipe_Cumulative* brakePipe_Cumulative;

	// Array of rail vehicle types defined boolean
	bool railVehicleTypesDefinedBool;

	// Array of x location of rail vehicles along brake pipe finite element defined boolean
	bool railVehicleXLocationsDefinedBool;

	// X location of rail vehicles along brake pipe finite element
	double* railVehicleXLocations;

	// Brake pipe finite element index
	int brakePipeFEIndex;

	// Node reynolds numbers
	double* nodeReynoldsNumbers;

	// Node wall friction factors
	double* nodeWallFrictionFactors;

	// Calculate interpolated value
	// x				-->	x value
	// pressureBool		-->	'true' for pressure interpolation, and 'false' for m variable interpolation
	double calculateInterpolatedValue(double x, bool pressureBool);

};

#endif
