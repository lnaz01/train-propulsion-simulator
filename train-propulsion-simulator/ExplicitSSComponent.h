//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef EXPLICIT_SS_COMPONENT_DEF
#define EXPLICIT_SS_COMPONENT_DEF

#include "UserDefinedRRComponent.h"

class InputFileReader_Simulation;

class ExplicitSSComponent : public UserDefinedRRComponent {

public:

	// physicalConstantsSize		-->	Number of physical constants
	// physicalVariablesSize		-->	Number of physical variables
	// ssvSize						-->	Number of state space variables
	ExplicitSSComponent(InputFileReader_Simulation* inputFileReader_Simulation, int physicalConstantsSize, int physicalVariablesSize, int ssvSize);

	virtual ~ExplicitSSComponent();

	// State-space variables
	double* ssv;

	// State space variables approximation
	double* ssvApp;

	// State space equations
	virtual void calc_ssvDot() = 0;

	// Calculates state-space variables which serves as input for state-space equations method
	void calc_ssvApp(InputFileReader_Simulation* inputFileReader_Simulation);

	// Calculates current step 'k' value
	void calc_kVals();

	// Calculates fourth order result
	void calc_res4();

	// Calculates fifth order result
	void calc_res5();

	// Calculates error estimates (returns 'true' if estimated error is too high)
	double calc_ee();

	// Updates state space variables
	void update_ssv();

protected:

	// Derivative of state-space variables (NOTE: The derivative of state-space variables represents the state-space equations)
	double* ssvDot;

private:

	// Number of state-space variables
	int ssvSize;

	// 'k' values (used in Runge-Kutta and similar explicit algorithms)
	double** kVals;

	// 'k' values space allocated boolean
	bool kValsBool;

	// Fourth order result
	double* res4;

	// Fifth order result
	double* res5;

	// Error estimates
	double* ee;

};

#endif
