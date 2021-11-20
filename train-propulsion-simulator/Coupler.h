//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef COUPLER_DEF
#define COUPLER_DEF

#include <string>
#include "UnitConverter.h"
#include "UserDefinedRRComponent.h"

class CouplingSystem;
class Function;
class RailVehicle;

class Coupler : public UserDefinedRRComponent {

public:

	Coupler(InputFileReader_Simulation* ifrsim);

	virtual ~Coupler();

	// Rail vehicle
	RailVehicle* railVehicle;

	// Coupling system
	CouplingSystem* couplingSystem;

	// Maximum displacement for first point in US units (inches)
	const double MAXD0_US[1] = { -3.5 };

	// Maximum force for first point in US units (kips)
	const double MAXF0_US[1] = { -350.0 };

	// Minimum force for last point in US units (kips)
	const double MINFF_US[1] = { 350.0 };

	// Minimum slope in US units (kips/inch)
	const double MINS_US = 1.0;

	// Maximum slope in US units (kips/inch)
	const double MAXS_US = 1000.0;

	// Displacement (meters)
	double displacement;

	// Damping constant (kilograms/second)
	const double DAMPING_CONST = 500.0;

	void convertToSI() override;

protected:

	void loadPhysicalConstantAlternateNames() override;

};

#endif
