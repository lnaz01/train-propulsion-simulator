//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROL_VALVE_DEF
#define CONTROL_VALVE_DEF

class Function;

class ControlValve {

public:

	ControlValve();

	virtual ~ControlValve();

	// Function defining brake cylinder pressure as function of brake pipe pressure for car or independent control valve setting for locomotive
	Function* brakeCylinderPressureFunction;

protected:

	// Function domain -- brake pipe pressure for car or independent brake control valve setting for locomotive (pascals)
	double* brakeCylinderPressureFunction_Domain;

	// Function range -- brake cylinder pressure (pascals)
	double* brakeCylinderPressureFunction_Range;

	// Calculates function defining brake cylinder pressure as function of brake pipe pressure for car or independent control valve setting for locomotive
	void calc_brakeCylinderPressureFunction(int brakeCylinderPressureFunction_NumberDataPoints);

};

#endif
