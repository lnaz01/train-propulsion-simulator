//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef COUPLING_SYSTEM_DEF
#define COUPLING_SYSTEM_DEF

#include <vector>
#include "UnitConverter.h"

class Coupler;

class CouplingSystem {

public:

	// (Note: CouplerSystem should be created after conversion to SI)
	// leadingCoupler	-->	Leading coupler (trailing coupler of leading rail vehicle)
	// trailingCoupler	-->	Trailing coupler (leading coupler of trailing rail vehicle)
	CouplingSystem(Coupler* leadingCoupler, Coupler* trailingCoupler);

	virtual ~CouplingSystem();

	// Array list of couplers
	// (Note: There will be two couplers in list.  Coupler at index '0' is leading coupler, which is trailing coupler of leading rail vehicle.  
	// Coupler at index '1' is trailing coupler, which is leading coupler of trailing rail vehicle.)
	std::vector<Coupler*> couplers;

	// Loading integer
	static const int LI = 0;

	// Unloading integer
	static const int UI = 1;

	// Loading/unloading condition
	int loadingUnloadingCondition;

	// Force on leading rail vehicle due to stiffness and damping (newtons)
	double totalForceOnLeadingRailVehicle;

	// Calculates total force on leading rail vehicle
	void calc_totalForceOnLeadingRailVehicle();

	// Calculates neutral (unstressed) center-to-center distance between leading rail vehicle and trailing rail vehicle
	void calc_centerToCenterDistance_Unstressed();

	// Calculates current center-to-center distance between leading rail vehicle and trailing rail vehicle
	// staticBool	-->		Determines whether to calculate center-to-center distance statically or dynamically
	//						(If 'true', then static center-to-center distance is calculated using 'ssv' variable in 'ExplicitSSComponent' class)
	//						(If 'false', then dynamic center-to-center distance is calculated using 'ssvApp' variable in 'ExplicitSSComponent' class)
	void calc_centerToCenterDistance(bool staticBool);

	// Calculates displacement of coupler system (Note: The funciton 'calc_centerToCenterDistance' should be called before this function)
	void calc_displacement();

	// Calculates effective damping constant
	void calc_effectiveDampingConstant();

	// Calculates initial coupler displacements
	void initialCouplerDisplacements();

private:

	// Displacement of coupling system (meters)
	double displacement;

	// Neutral (unstressed) center-to-center distance between leading rail vehicle and trailing rail vehicle
	double centerToCenterDistance_Unstressed;

	// Current center-to-center distance between leading rail vehicle and trailing rail vehicle
	double centerToCenterDistance;

	// Acceptable tolerance for force calculation (newtons)
	const double FORCE_CALCULATION_TOLERANCE = UnitConverter::kip_To_N(0.0001);

	// Effective damping constant
	double effectiveDampingConstant;

	// Calculates total (not tangential) force on leading rail vehicle due ot stiffness and calculates coupler displacement
	// (Note: Returns force on leading rail vehicle due to stiffness)
	double calculateForceOnLeadingRVDueToStiffnessAndCalculateCouplerDisplacements();

	// Calculates total (not tangential) force on leading rail vehicle due to damping
	double calculateForceOnLeadingRVDueToDamping();

};

#endif
