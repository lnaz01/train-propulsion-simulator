//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "CouplingSystem.h"
#include "Coupler.h"
#include "Function.h"
#include "RailVehicle.h"


CouplingSystem::CouplingSystem(Coupler* leadingCoupler, Coupler* trailingCoupler) {
	couplers.push_back(leadingCoupler);
	couplers.push_back(trailingCoupler);
}


CouplingSystem::~CouplingSystem() {
	// No need to delete 'couplers' as memory for them are not allocated in this class.  Memory allocation and deletion for 'couplers' occurs in 'RailVehicle' class.
}


void CouplingSystem::calc_centerToCenterDistance_Unstressed() {
	centerToCenterDistance_Unstressed = (couplers[0]->railVehicle->length / 2.0) + (couplers[1]->railVehicle->length / 2.0);
}


void CouplingSystem::calc_centerToCenterDistance(bool staticBool) {
	if (staticBool == true) {
		centerToCenterDistance = couplers[0]->railVehicle->ssv[0] - couplers[1]->railVehicle->ssv[0];
	}
	else {
		centerToCenterDistance = couplers[0]->railVehicle->ssvApp[0] - couplers[1]->railVehicle->ssvApp[0];
	}
}


void CouplingSystem::calc_displacement() {
	displacement = centerToCenterDistance - centerToCenterDistance_Unstressed;
}


void CouplingSystem::calc_effectiveDampingConstant() {
	effectiveDampingConstant = couplers[0]->DAMPING_CONST + couplers[1]->DAMPING_CONST;
}


void CouplingSystem::initialCouplerDisplacements() {
	couplers[0]->displacement = 0.0;
	//couplers[0]->displacement = UnitConverter::in_To_M(1.0);  // used for closed form testing
	couplers[1]->displacement = 0.0;
	//couplers[1]->displacement = UnitConverter::in_To_M(1.0);  // used for closed form testing
}


void CouplingSystem::calc_totalForceOnLeadingRailVehicle() {
	totalForceOnLeadingRailVehicle = calculateForceOnLeadingRVDueToStiffnessAndCalculateCouplerDisplacements() + calculateForceOnLeadingRVDueToDamping();
}


double CouplingSystem::calculateForceOnLeadingRVDueToStiffnessAndCalculateCouplerDisplacements() {
	double deltax = std::abs(displacement / 10.0);
	double d1 = 0.0; // displacement of trailing vehicle's leading coupler
	double d2 = displacement; // displacement of leading vehicle's traling coupler
	double fk1 = 0.0; // force of trailing vehicle's leading coupler due to stiffness
	double fk2 = couplers[0]->PVDMAX_SI[0]; // force of leading vehicle's trailing coupler due to stiffness
	do {
		fk1 = couplers[1]->physicalVariables[0]->interpolate(d1, couplers[1]->PVDMIN_SI[0], couplers[1]->PVDMAX_SI[0]);
		fk2 = couplers[0]->physicalVariables[0]->interpolate(d2, couplers[0]->PVDMIN_SI[0], couplers[0]->PVDMAX_SI[0]);
		if (displacement > 0.0) {
			if ((fk2 > fk1) && (std::abs(fk1 - fk2) > FORCE_CALCULATION_TOLERANCE)) {
				d1 = d1 + deltax;
				d2 = d2 - deltax;
			}
			else if ((fk2 < fk1) && (std::abs(fk1 - fk2) > FORCE_CALCULATION_TOLERANCE)) {
				deltax = deltax / 1.5;
				d1 = d1 - deltax;
				d2 = d2 + deltax;
			}
		}
		else if (displacement < 0.0) {
			if ((fk2 < fk1) && (std::abs(fk1 - fk2) > FORCE_CALCULATION_TOLERANCE)) {
				d1 = d1 - deltax;
				d2 = d2 + deltax;
			}
			else if ((fk2 > fk1) && (std::abs(fk1 - fk2) > FORCE_CALCULATION_TOLERANCE)) {
				deltax = deltax / 1.5;
				d1 = d1 + deltax;
				d2 = d2 - deltax;
			}
		}
		else {
			d1 = 0.0;
			d2 = 0.0;
			fk1 = 0.0;
			fk2 = 0.0;
		}
	} while (std::abs(fk1 - fk2) > FORCE_CALCULATION_TOLERANCE);
	// Set displacement for both couplers
	couplers[1]->displacement = d1;
	couplers[0]->displacement = d2;
	// Force on leading rail vehicle due to stiffness
	// (note that polarity change is necessary for 'fk1' and 'fk2' due to sign convention in coupler force-displacement functions)
	double forceOnLeadingRVDueToStiffness = (-fk1 + -fk2) / 2.0;
	return forceOnLeadingRVDueToStiffness;
}


double CouplingSystem::calculateForceOnLeadingRVDueToDamping() {
	// Calculate force due to damping
	double fc; // force due to damping
	fc = effectiveDampingConstant * (couplers[0]->railVehicle->ssv[1] - couplers[1]->railVehicle->ssv[1]);
	double forceOnLeadingRVDueToDamping = -fc;
	return forceOnLeadingRVDueToDamping;
}

