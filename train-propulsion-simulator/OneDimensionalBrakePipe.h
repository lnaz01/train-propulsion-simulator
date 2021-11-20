//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ONE_DIMENSIONAL_BRAKE_PIPE_DEF
#define ONE_DIMENSIONAL_BRAKE_PIPE_DEF

#include <string>

class Function;
class LinearSystem;
class ResultsWriter;

class OneDimensionalBrakePipe {

public:

	// numFE			-->	Number of finite elements
	// h				-->	Length of finite elements array
	// d1				-->	Diameter at first node of each finite element
	// d2				-->	Diameter at second node of each finite element
	OneDimensionalBrakePipe(int numFE, double* h, double* d1, double* d2);

	// Destructor
	virtual ~OneDimensionalBrakePipe();

	// Simulate
	// resWriter_pressure			--> Results writer for air pressure results
	// resWriter_velocity			-->	Results writer for air velocity results
	// timeSim						-->	Total simulation time
	// timeStep						-->	Time step
	// airTemp						--> Air temperature
	// pressureInit					-->	Initial condition array for pressure
	// velocityInit					-->	Initial condition array for velocity
	// leftRightPressureBoundCond	--> Indicates left or right pressure boundary condition ("left" for left boundary condition; any other string besides "left" will result in a right boundary condition)
	// pUP							-->	Upstream pressure array
	// pUPAct						--> Upstream pressure array activated
	void simulate(ResultsWriter* resWriter_pressure, ResultsWriter* resWriter_velocity, double timeSim, double timeStep, double airTemp, double* pressureInit, double* velocityInit, std::string leftRightPressureBoundCond, double* pUP, bool pUPAct);

protected:

	// Left-end reservoir pressure function
	Function* endReservoirPressure;

	// Pressure array
	double* pressure;

	// Leakage array
	double* leakArr;

	// Number of finite elements
	int numFE;

	// Left-end reservoir pressure function defined boolean
	bool endReservoirPressureDefinedBool;

private:

	// Individual gas constant of air (joules / (kilogram * kelvin))
	const double Rg = 287.0;

	// Linear system
	LinearSystem* linearSystem;

	// Length of finite elements array
	double* h;

	// Diameter at first node of each finite element
	double* d1;

	// Diameter at second node of each finite element
	double* d2;

	// Size of location of nodes array
	int locNodArrSize;

	// Location of nodes array
	double* locNodArr;

	// Reynolds number array
	double* reynoldsNumArr;

	// Wall friction factor array
	double* wallFricFactorArr;

	// Velocity array
	double* velocity;

	// Calculates left boundary condition (a.k.a. reservoir pressure on left end of brake pipe)
	virtual double pressureBoundCondCalc(double t, double tStep) = 0;

	// Calculates leakage
	virtual void leakArrCalc(double airTemp, double* pUP, bool pUPAct) = 0;

	// Calculates wall friction factor
	void wallFricFactorArrCalc(double airTemp, double airDynVisc);

	// Calculates reynolds number
	void reynoldsNumArrCalc(double airTemp, double airDynVisc);

	// Calculates air dynamic viscosity which depends on air temperature
	double airDynViscCalc(double airTemp);

};

#endif
