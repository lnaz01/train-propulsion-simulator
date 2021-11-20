//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "RailVehicle.h"
#include "BrakeCylinder.h"
#include "Coupler.h"
#include "CouplingSystem.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "LocomotiveOperator.h"
#include "Track.h"
#include "TrainConsist.h"
#include "VectorRotator.h"


RailVehicle::RailVehicle(InputFileReader_Simulation* inputFileReader_Simulation, int npcs, int npvs, int nssv) : ExplicitSSComponent(inputFileReader_Simulation, npcs, npvs, nssv) {
	printResultsBool = false;
	ssv[0] = 0.0;
	ssv[1] = 0.0;
	resultsWriter = NULL;
	couplers.push_back(new Coupler(inputFileReader_Simulation));  // the first coupler (index 0) is the leading coupler
	couplers.push_back(new Coupler(inputFileReader_Simulation));  // the second coupler (index 1) is the trailing coupler
	globalVector_globalGravitationalForce = new double[3];
	vectorRotator_globalGravitationalForce = new VectorRotator();
	globalVector_globalReactiveCentrifugalForce = new double[3];
	vectorRotator_globalReactiveCentrifugalForce = new VectorRotator();
	brakePipeAirPressure = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
	brakePipeAirVelocity = 0.0;
	alpha_leadingRailVehicle = 0.0;
	alpha_trailingRailVehicle = 0.0;
}


RailVehicle::~RailVehicle() {
	delete[] globalVector_globalGravitationalForce;
	delete vectorRotator_globalGravitationalForce;
	delete[] globalVector_globalReactiveCentrifugalForce;
	delete vectorRotator_globalReactiveCentrifugalForce;
	if (resultsWriter != NULL) {
		delete resultsWriter;
	}
	delete resultsWriter;
	for (size_t i = 0; i < couplers.size(); i++) {
		delete couplers[i];
	}
	delete brakeCylinder;
	delete controlValve;
}


void RailVehicle::calc_angleBetweenTopOfRailAndCoupler() {
	angleBetweenTopOfRailAndCoupler = atan(couplerHeight / (0.5 * Track::TrackGaugeSI));
}


void RailVehicle::calc_angleBetweenTopOfRailAndCenterOfGravity() {
	angleBetweenTopOfRailAndCenterOfGravity = atan(centerOfGravityHeight / (0.5 * Track::TrackGaugeSI));
}


void RailVehicle::calc_firstSquareRootTermForLOverVCalc() {
	firstSquareRootTermForLOverVCalc = pow(pow(Track::TrackGaugeSI / 2.0, 2.0) + pow(couplerHeight, 2.0), 0.5);
}


void RailVehicle::calc_secondSquareRootTermForLOverVCalc() {
	secondSquareRootTermForLOverVCalc = pow(pow(Track::TrackGaugeSI / 2.0, 2.0) + pow(centerOfGravityHeight, 2.0), 0.5);
}


void RailVehicle::calc_forceOnTrackDueToLeadingTruck() {
	double numeratorTerm1 = lateralForceDueToTrailingCoupler * ((length - truckCenterSpacing) / 2.0);
	double numeratorTerm2 = lateralForceDueToLeadingCoupler * (truckCenterSpacing + ((length - truckCenterSpacing) / 2.0));
	forceOnTrackDueToLeadingTruck = (numeratorTerm1 - numeratorTerm2) / truckCenterSpacing;
	forceOnTrackDueToLeadingTruck = -forceOnTrackDueToLeadingTruck;
}


void RailVehicle::calc_forceOnTrackDueToTrailingTruck() {
	double numeratorTerm1 = lateralForceDueToLeadingCoupler * ((length - truckCenterSpacing) / 2.0);
	double numeratorTerm2 = lateralForceDueToTrailingCoupler * (truckCenterSpacing + ((length - truckCenterSpacing) / 2.0));
	forceOnTrackDueToTrailingTruck = (numeratorTerm1 - numeratorTerm2) / truckCenterSpacing;
	forceOnTrackDueToTrailingTruck = -forceOnTrackDueToTrailingTruck;
}


void RailVehicle::calc_verticalForceRightSideLeadingTruck() {
	double term1_tot = (firstSquareRootTermForLOverVCalc * forceOnTrackDueToLeadingTruck * sin(angleBetweenTopOfRailAndCoupler)) / Track::TrackGaugeSI;
	double term2_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0) * sin(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	double term3_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[2] + vectorRotator_globalGravitationalForce->rotatedVector[2]) / 2.0) * cos(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	verticalForceRightSideLeadingTruck = -term1_tot - term2_tot - term3_tot;
}


void RailVehicle::calc_verticalForceLeftSideLeadingTruck() {
	double term1_tot = (firstSquareRootTermForLOverVCalc * forceOnTrackDueToLeadingTruck * sin(angleBetweenTopOfRailAndCoupler)) / Track::TrackGaugeSI;
	double term2_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0) * sin(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	double term3_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[2] + vectorRotator_globalGravitationalForce->rotatedVector[2]) / 2.0) * cos(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	verticalForceLeftSideLeadingTruck = term1_tot + term2_tot - term3_tot;
}


void RailVehicle::calc_verticalForceRightSideTrailingTruck() {
	double term1_tot = (firstSquareRootTermForLOverVCalc * forceOnTrackDueToTrailingTruck * sin(angleBetweenTopOfRailAndCoupler)) / Track::TrackGaugeSI;
	double term2_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0) * sin(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	double term3_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[2] + vectorRotator_globalGravitationalForce->rotatedVector[2]) / 2.0) * cos(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	verticalForceRightSideTrailingTruck = -term1_tot - term2_tot - term3_tot;
}


void RailVehicle::calc_verticalForceLeftSideTrailingTruck() {
	double term1_tot = (firstSquareRootTermForLOverVCalc * forceOnTrackDueToTrailingTruck * sin(angleBetweenTopOfRailAndCoupler)) / Track::TrackGaugeSI;
	double term2_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0) * sin(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	double term3_tot = (secondSquareRootTermForLOverVCalc * ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[2] + vectorRotator_globalGravitationalForce->rotatedVector[2]) / 2.0) * cos(angleBetweenTopOfRailAndCenterOfGravity)) / Track::TrackGaugeSI;
	verticalForceLeftSideTrailingTruck = term1_tot + term2_tot - term3_tot;
}


void RailVehicle::calc_lOverVForRightSideOfLeadingTruck() {
	double numerator_term_1 = forceOnTrackDueToLeadingTruck;
	double numerator_term_2 = (vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0;
	double denominator_term = verticalForceRightSideLeadingTruck;
	if (denominator_term <= 0.0) {
		lOverVForRightSideOfLeadingTruck = LV_WHEEL_UNLOADING;
	}
	else {
		lOverVForRightSideOfLeadingTruck = abs(numerator_term_1 + numerator_term_2) / denominator_term;
	}
}


void RailVehicle::calc_lOverVForLeftSideOfLeadingTruck() {
	double numerator_term_1 = forceOnTrackDueToLeadingTruck;
	double numerator_term_2 = ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0);
	double denominator_term = verticalForceLeftSideLeadingTruck;
	if (denominator_term <= 0.0) {
		lOverVForLeftSideOfLeadingTruck = LV_WHEEL_UNLOADING;
	}
	else {
		lOverVForLeftSideOfLeadingTruck = abs(numerator_term_1 + numerator_term_2) / denominator_term;
	}
}


void RailVehicle::calc_lOverVForRightSideOfTrailingTruck() {
	double numerator_term_1 = forceOnTrackDueToTrailingTruck;
	double numerator_term_2 = ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0);
	double denominator_term = verticalForceRightSideTrailingTruck;
	if (denominator_term <= 0.0) {
		lOverVForRightSideOfTrailingTruck = LV_WHEEL_UNLOADING;
	}
	else {
		lOverVForRightSideOfTrailingTruck = abs(numerator_term_1 + numerator_term_2) / denominator_term;
	}
}


void RailVehicle::calc_lOverVForLeftSideOfTrailingTruck() {
	double numerator_term_1 = forceOnTrackDueToTrailingTruck;
	double numerator_term_2 = ((vectorRotator_globalReactiveCentrifugalForce->rotatedVector[1] + vectorRotator_globalGravitationalForce->rotatedVector[1]) / 2.0);
	double denominator_term = verticalForceLeftSideTrailingTruck;
	if (denominator_term <= 0.0) {
		lOverVForLeftSideOfTrailingTruck = LV_WHEEL_UNLOADING;
	}
	else {
		lOverVForLeftSideOfTrailingTruck = abs(numerator_term_1 + numerator_term_2) / denominator_term;
	}
}


void RailVehicle::calc_lOverVMax() {
	double tempvar = lOverVForRightSideOfLeadingTruck;
	if (abs(lOverVForLeftSideOfLeadingTruck) > abs(tempvar)) {
		tempvar = lOverVForLeftSideOfLeadingTruck;
	}
	if (abs(lOverVForRightSideOfTrailingTruck) > abs(tempvar)) {
		tempvar = lOverVForRightSideOfTrailingTruck;
	}
	if (abs(lOverVForLeftSideOfTrailingTruck) > abs(tempvar)) {
		tempvar = lOverVForLeftSideOfTrailingTruck;
	}
	lOverVMax = abs(tempvar);
}


void RailVehicle::calc_brakePipeLength() {
	brakePipeLength = length * BRAKE_PIPE_LENGTH_OVER_CAR_LENGTH;
}


void RailVehicle::calc_brakeRiggingLeverageRatio() {
	brakeRiggingLeverageRatio = (BRAKE_RIGGING_SAFETY_FACTOR * maxNetBrakingRatio * TrainConsist::GRAVITATIONAL_CONSTANT * mass) /
		(((brakeCylinder->maxPressureForFullServiceBraking - TrainConsist::ATMOSPHERIC_PRESSURE) * brakeCylinder->PISTON_AREA) -
			(brakeCylinder->SPRING_STIFFNESS * brakeCylinder->PISTON_DISP_WHEEL_CONTACT) - brakeCylinder->SPRING_PRE_LOAD - brakeCylinder->FRICTION_FORCE);
}


void RailVehicle::calc_ssvDot() {
	// State space equations
	ssvDot[0] = ssvApp[1];
	ssvDot[1] = (sumExternalTangentialForces + tangentialForceDueToLeadingCoupler + tangentialForceDueToTrailingCoupler + brakingForce + throttleForce) / mass;
}


void RailVehicle::calc_sumExternalTangentialForces() {
	// Propulsion resistance
	double wit;  // weight in tons
	wit = UnitConverter::kg_To_Ton(mass);
	double term1 = 1.5;
	double term2 = (18.0 * numberOfAxles) / wit;
	double term3 = 0.03 * std::abs(UnitConverter::mps_To_Miph(ssv[1]));
	double term4 = ((UnitConverter::m2_To_Ft2(crossSectionalArea) * streamliningCoefficient) / (10000.0 * wit)) *
		pow(UnitConverter::mps_To_Miph(ssv[1]), 2.0);
	double propulsionResistance = term1 + term2 + term3 + term4;
	propulsionResistance = propulsionResistance * wit;
	propulsionResistance = UnitConverter::lb_To_N(propulsionResistance);
	if (ssv[1] > 0.0) {
		propulsionResistance = -propulsionResistance;
	}
	// Curving resistance
	/*
	double curvingResistance = 0.0004 * UnitConverter::kg_To_Lb(mass) *
		std::abs(UnitConverter::gCurv_To_TrkCurv(inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[1]->
			interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[1], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[1])));
	curvingResistance = UnitConverter::lb_To_N(curvingResistance);
	*/
	double curvingResistance = 0.0004 * mass * TrainConsist::GRAVITATIONAL_CONSTANT *
		std::abs(UnitConverter::gCurv_To_TrkCurv(inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[1]->
			interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[1], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[1])));
	if (ssv[1] > 0.0) {
		curvingResistance = -curvingResistance;
	}
	// Sum of external tangential forces
	sumExternalTangentialForces = vectorRotator_globalReactiveCentrifugalForce->rotatedVector[0] + vectorRotator_globalGravitationalForce->rotatedVector[0] + (propulsionResistance + curvingResistance);
}


void RailVehicle::calc_theta() {
	theta = inputFileReader_Simulation->userDefinedTracks[0]->angle(ssv[0]);
}


void RailVehicle::calc_globalGravitationalForce() {
	globalGravitationalForce = mass * (-TrainConsist::GRAVITATIONAL_CONSTANT);
}


void RailVehicle::calc_globalVector_globalGravitationalForce() {
	globalVector_globalGravitationalForce[0] = 0.0;
	globalVector_globalGravitationalForce[1] = 0.0;
	globalVector_globalGravitationalForce[2] = globalGravitationalForce;
}


void RailVehicle::calcLocal3DVecFromGlobalGravitationalForce() {
	// Rotation due to superelevation
	double xrot = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[2]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[2], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[2]);
	// Rotation due to profile (grade)
	double yrot = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[0]);
	vectorRotator_globalGravitationalForce->rotateVector(globalVector_globalGravitationalForce, xrot, yrot, 0.0);
}


void RailVehicle::calc_globalReactiveCentrifugalForce() {
	globalReactiveCentrifugalForce = (mass * pow(ssv[1], 2.0)) * inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[1]->interpolate(ssv[0],
		inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[1], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[1]);
}


void RailVehicle::calc_globalVector_globalReactiveCentrifugalForce() {
	globalVector_globalReactiveCentrifugalForce[0] = 0.0;
	globalVector_globalReactiveCentrifugalForce[1] = globalReactiveCentrifugalForce;
	globalVector_globalReactiveCentrifugalForce[2] = 0.0;
}


void RailVehicle::calcLocal3DVecFromGlobalReactiveCentrifugalForce() {
	// Rotation due to superelevation
	double xrot = inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[2]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[2], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[2]);
	vectorRotator_globalReactiveCentrifugalForce->rotateVector(globalVector_globalReactiveCentrifugalForce, xrot, 0.0, 0.0);
}


void RailVehicle::calc_tangentialForceDueToLeadingCoupler() {
	if (positionInTrainConsist == 0) {
		tangentialForceDueToLeadingCoupler = 0.0;
	}
	else {
		tangentialForceDueToLeadingCoupler = -inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[positionInTrainConsist - 1]->totalForceOnLeadingRailVehicle *
			cos(alpha_leadingRailVehicle);
	}
}


void RailVehicle::calc_tangentialForceDueToTrailingCoupler() {
	if (positionInTrainConsist == inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1) {
		tangentialForceDueToTrailingCoupler = 0.0;
	}
	else {
		tangentialForceDueToTrailingCoupler = inputFileReader_Simulation->userDefinedTrainConsists[0]->couplingSystems[positionInTrainConsist]->totalForceOnLeadingRailVehicle *
			cos(alpha_trailingRailVehicle);
	}
}


void RailVehicle::calc_lateralForceDueToLeadingCoupler() {
	if (positionInTrainConsist == 0) {
		lateralForceDueToLeadingCoupler = 0.0;
	}
	else {
		lateralForceDueToLeadingCoupler = tangentialForceDueToLeadingCoupler * tan(alpha_leadingRailVehicle);
	}
}


void RailVehicle::calc_lateralForceDueToTrailingCoupler() {
	if (positionInTrainConsist == inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1) {
		lateralForceDueToTrailingCoupler = 0.0;
	}
	else {
		lateralForceDueToTrailingCoupler = tangentialForceDueToTrailingCoupler * tan(alpha_trailingRailVehicle);
	}
}


void RailVehicle::calc_alpha_leadingRailVehicle() {
	if (positionInTrainConsist == 0) {
		alpha_leadingRailVehicle = 0.0;
	}
	else {
		alpha_leadingRailVehicle = theta - inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[positionInTrainConsist - 1]->theta;
	}
}


void RailVehicle:: calc_alpha_trailingRailVehicle() {
	if (positionInTrainConsist == inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1) {
		alpha_trailingRailVehicle = 0.0;
	}
	else {
		alpha_trailingRailVehicle = theta - inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles[positionInTrainConsist + 1]->theta;
	}
}

