//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Car.h"
#include "AuxiliaryReservoir.h"
#include "BrakeCylinder.h"
#include "ControlValve_Car.h"
#include "Coupler.h"
#include "EmergencyReservoir.h"
#include "EndOfTrainDevice.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "IntervalSmooth.h"
#include "LocomotiveOperator.h"
#include "ResultsWriter.h"
#include "Simulation.h"
#include "Track.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


Car::Car(InputFileReader_Simulation* ifrsim) : RailVehicle(ifrsim, 11, 2, 2) {
	componentType = 2;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables.push_back(new Function(this, i));
	}
	// State space variables
	// 0	--> Position (feet)
	// 1	--> Velocity (mph)
	// User-inputted physical constants
	// 0	-->	Mass (kips)
	// 1	-->	Length (feet)
	// 2	-->	Number of axles
	// 3	-->	Cross sectional area (feet^2)
	// 4	-->	Streamlining coefficient
	// 5	-->	Maximum net braking ratio
	// 6	-->	Hand brake status
	// 7	-->	Hand brake efficiency ratio
	// 8	-->	Truck center spacing (feet)
	// 9	-->	Coupler height (feet)
	// 10	-->	Center-of-gravity height (feet)
	// User-inputted physical variables
	// 0	-->	Brake rigging efficiency
	// 1	-->	Brake shoe friction coefficient
	brakeCylinder = new BrakeCylinder(this);
	controlValve = new ControlValve_Car(this);
	endOfTrainDevice = NULL;
	auxiliaryReservoir = new AuxiliaryReservoir(this);
	emergencyReservoir = new EmergencyReservoir(this);
	START_UDRRC_STRING = "Car_";
	END_UDRRC_STRING = "_Car";
	// Physical constants US units
	// Index 0		--> Mass (kips)
	// Index 1		-->	Length (feet)
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area (feet^2)
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing (feet)
	// Index 9		-->	Coupler height (feet)
	// Index 10		-->	Center-of-gravity height (feet)
	PCSTR_US[0] = "kips";
	PCSTR_US[1] = "feet";
	PCSTR_US[2] = "";
	PCSTR_US[3] = "square feet";
	PCSTR_US[4] = "";
	PCSTR_US[5] = "";
	PCSTR_US[6] = "";
	PCSTR_US[7] = "";
	PCSTR_US[8] = "feet";
	PCSTR_US[9] = "feet";
	PCSTR_US[10] = "feet";
	// Physical constants integer boolean
	// Index 0		--> Mass
	// Index 1		-->	Length
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing
	// Index 9		-->	Coupler height
	// Index 10		-->	Center-of-gravity height
	PCINT[0] = false;
	PCINT[1] = false;
	PCINT[2] = true;
	PCINT[3] = false;
	PCINT[4] = false;
	PCINT[5] = false;
	PCINT[6] = true;
	PCINT[7] = false;
	PCINT[8] = false;
	PCINT[9] = false;
	PCINT[10] = false;
	// Physical constant minimum threshold values in US units
	// Index 0		--> Mass (kips)
	// Index 1		-->	Length (feet)
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area (feet^2)
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing (feet)
	// Index 9		-->	Coupler height (feet)
	// Index 10		-->	Center-of-gravity height (feet)
	PCMIN_US[0] = 30.0;
	PCMIN_US[1] = 40.0;
	PCMIN_US[2] = 4.0;
	PCMIN_US[3] = 20.0;
	PCMIN_US[4] = 1.0;
	PCMIN_US[5] = 0.01;
	PCMIN_US[6] = 0.0;
	PCMIN_US[7] = 0.01;
	PCMIN_US[8] = RailVehicle::MIN_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * PCMIN_US[1];
	PCMIN_US[9] = 1.0;
	PCMIN_US[10] = 1.0;
	// Physical constant minimum threshold values in SI units
	// Index 0		--> Mass (kg)
	// Index 1		-->	Length (meters)
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area (meters^2)
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing (meters)
	// Index 9		-->	Coupler height (meters)
	// Index 10		-->	Center-of-gravity height (meters)
	PCMIN_SI[0] = UnitConverter::lb_To_Kg(PCMIN_US[0]);
	PCMIN_SI[1] = UnitConverter::ft_To_M(PCMIN_US[1]);
	PCMIN_SI[2] = PCMIN_US[2];
	PCMIN_SI[3] = UnitConverter::ft2_To_M2(PCMIN_US[3]);
	PCMIN_SI[4] = PCMIN_US[4];
	PCMIN_SI[5] = PCMIN_US[5];
	PCMIN_SI[6] = PCMIN_US[6];
	PCMIN_SI[7] = PCMIN_US[7];
	PCMIN_SI[8] = UnitConverter::ft_To_M(PCMAX_US[8]);
	PCMIN_SI[9] = UnitConverter::ft_To_M(PCMAX_US[9]);
	PCMIN_SI[10] = UnitConverter::ft_To_M(PCMAX_US[10]);
	// Physical constant maximum threshold values in US units
	// Index 0		--> Mass (kips)
	// Index 1		-->	Length (feet)
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area (feet^2)
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing (feet)
	// Index 9		-->	Coupler height (feet)
	// Index 10		-->	Center-of-gravity height (feet)
	PCMAX_US[0] = 600.0;
	PCMAX_US[1] = 110.0;
	PCMAX_US[2] = 6.0;
	PCMAX_US[3] = 200.0;
	PCMAX_US[4] = 30.0;
	PCMAX_US[5] = 0.2;
	PCMAX_US[6] = 1.0;
	PCMAX_US[7] = 0.2;
	PCMAX_US[8] = RailVehicle::MAX_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * PCMAX_US[1];
	PCMAX_US[9] = 5.0;
	PCMAX_US[10] = 15.0;
	// Physical constant maximum threshold values in SI units
	// Index 0		--> Mass (kg)
	// Index 1		-->	Length (meters)
	// Index 2		-->	Number of axles
	// Index 3		-->	Cross-sectional area (meters^2)
	// Index 4		-->	Streamlining coefficient
	// Index 5		-->	Maximum net braking ratio
	// Index 6		-->	Hand brake status
	// Index 7		-->	Hand brake efficiency ratio
	// Index 8		-->	Truck center spacing (meters)
	// Index 9		-->	Coupler height (meters)
	// Index 10		-->	Center-of-gravity height (meters)
	PCMAX_SI[0] = UnitConverter::lb_To_Kg(PCMAX_US[0]);
	PCMAX_SI[1] = UnitConverter::ft_To_M(PCMAX_US[1]);
	PCMAX_SI[2] = PCMAX_US[2];
	PCMAX_SI[3] = UnitConverter::ft2_To_M2(PCMAX_US[3]);
	PCMAX_SI[4] = PCMAX_US[4];
	PCMAX_SI[5] = PCMAX_US[5];
	PCMAX_SI[6] = PCMAX_US[6];
	PCMAX_SI[7] = PCMAX_US[7];
	PCMAX_SI[8] = UnitConverter::ft_To_M(PCMAX_US[8]);
	PCMAX_SI[9] = UnitConverter::ft_To_M(PCMAX_US[9]);
	PCMAX_SI[10] = UnitConverter::ft_To_M(PCMAX_US[10]);
	// Physical variable independent component US units
	// Index 0		-->	Brake cylinder pressure (psi)
	// Index 1		--> Rail vehicle velocity (mph)
	PVISTR_US[0] = "pounds per square inch";
	PVISTR_US[1] = "miles per hour";
	// Physical variable dependent component US units
	// Index 0		-->	Brake rigging efficiency (unitless)
	// Index 1		--> Brake shoe friction coefficient (unitless)
	PVDSTR_US[0] = "";
	PVDSTR_US[1] = "";
	// Physical variable dependent component integer boolean ('true' for integer; 'false' for double)
	// Index 0		-->	Brake rigging efficiency
	// Index 1		--> Brake shoe friction coefficient
	PVDINT[0] = false;
	PVDINT[1] = false;
	// Physical variable independent component minimum threshold values in US units
	// Index 0		-->	Brake cylinder pressure (psi)
	// Index 1		--> Rail vehicle velocity (mph)
	PVIMIN_US[0] = round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE));
	PVIMIN_US[1] = 0.0;
	// Physical variable independent component minimum threshold values in SI units
	// Index 0		-->	Brake cylinder pressure (pascals)
	// Index 1		--> Rail vehicle velocity (meters/second)
	PVIMIN_SI[0] = UnitConverter::psi_To_Pa(PVIMIN_US[0]);
	PVIMIN_SI[1] = UnitConverter::miph_To_Mps(PVIMIN_US[1]);
	// Physical variable independent component minimum maximum threshold values in US units
	// Index 0		-->	Brake cylinder pressure (psi)
	// Index 1		--> Rail vehicle velocity (mph)
	PVIMINMAX_US[0] = round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE)) - 20.0;
	PVIMINMAX_US[1] = 70.0;
	// Physical variable independent component maximum threshold values in US units
	// Index 0		-->	Brake cylinder pressure (psi)
	// Index 1		--> Rail vehicle velocity (mph)
	PVIMAX_US[0] = round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE));
	PVIMAX_US[1] = 90.0;
	// Physical variable independent component maximum threshold values in SI units
	// Index 0		-->	Brake cylinder pressure (pascals)
	// Index 1		--> Rail vehicle velocity (meters/second)
	PVIMAX_SI[0] = UnitConverter::psi_To_Pa(PVIMAX_US[0]);
	PVIMAX_SI[1] = UnitConverter::miph_To_Mps(PVIMAX_US[1]);
	// Physical variable dependent component minimum threshold values in US units
	// Index 0		-->	Brake rigging efficiency (unitless)
	// Index 1		--> Brake shoe friction coefficient (unitless)
	PVDMIN_US[0] = 0.01;
	PVDMIN_US[1] = 0.01;
	// Physical variable dependent component minimum threshold values in SI units
	// Index 0		-->	Brake rigging efficiency (unitless)
	// Index 1		-->	Brake shoe friction coefficient (unitless)
	PVDMIN_SI[0] = PVDMIN_US[0];
	PVDMIN_SI[1] = PVDMIN_US[1];
	// Physical variable dependent component maximum threshold values in US units
	// Index 0		-->	Brake rigging efficiency (unitless)
	// Index 1		-->	Brake shoe friction coefficient (unitless)
	PVDMAX_US[0] = 1.0;
	PVDMAX_US[1] = 1.0;
	// Physical variable dependent component maximum threshold values in SI units
	// Index 0		-->	Brake rigging efficiency (unitless)
	// Index 1		-->	Brake shoe friction coefficient (unitless)
	PVDMAX_SI[0] = PVDMAX_US[0];
	PVDMAX_SI[1] = PVDMAX_US[1];
}


Car::~Car() {
	if (endOfTrainDevice != NULL) {
		delete endOfTrainDevice;
	}
	delete auxiliaryReservoir;
	delete emergencyReservoir;
}


void Car::initialize_endOfTrainDevice() {
	endOfTrainDevice = new EndOfTrainDevice(this);
}


void Car::convertToSI() {
	double param_us;
	double param_si;
	// Physical constant 0 -- Mass (kilograms)
	param_us = physicalConstants[0];
	param_si = UnitConverter::kip_To_Kg(param_us);
	physicalConstants[0] = param_si;
	// Physical constant 1 -- Length (meters)
	param_us = physicalConstants[1];
	param_si = UnitConverter::ft_To_M(param_us);
	physicalConstants[1] = param_si;
	// Physical constant 2 -- Number of axles
	param_us = physicalConstants[2];
	param_si = param_us;
	physicalConstants[2] = param_si;
	// Physical constant 3 -- Cross-sectional area (meters^2)
	param_us = physicalConstants[3];
	param_si = UnitConverter::ft2_To_M2(param_us);
	physicalConstants[3] = param_si;
	// Physical constant 4 -- Streamlining coefficient
	param_us = physicalConstants[4];
	param_si = param_us;
	physicalConstants[4] = param_si;
	// Physical constant 5 -- Maximum net braking ratio
	param_us = physicalConstants[5];
	param_si = param_us;
	physicalConstants[5] = param_si;
	// Physical constant 6 -- Hand brake status
	param_us = physicalConstants[6];
	param_si = param_us;
	physicalConstants[6] = param_si;
	// Physical constant 7 -- Hand brake efficiency ratio
	param_us = physicalConstants[7];
	param_si = param_us;
	physicalConstants[7] = param_si;
	// Physical constant 8 -- Truck center spacing
	param_us = physicalConstants[8];
	param_si = UnitConverter::ft_To_M(param_us);
	physicalConstants[8] = param_si;
	// Physical constant 9 -- Coupler height
	param_us = physicalConstants[9];
	param_si = UnitConverter::ft_To_M(param_us);
	physicalConstants[9] = param_si;
	// Physical constant 10 -- Center-of-gravity height
	param_us = physicalConstants[10];
	param_si = UnitConverter::ft_To_M(param_us);
	physicalConstants[10] = param_si;
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Physical variable 0 -- Brake rigging efficiency
	for (size_t i = 0; i < physicalVariables[0]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[0]->intervals[i]->points.size(); j++) {
			double x_us = physicalVariables[0]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[0]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::psi_To_Pa(x_us);
			double y_si = y_us;
			physicalVariables[0]->intervals[i]->points[j]->x = x_si;
			physicalVariables[0]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[0]->intervals[i])->zvcb = false;
	}
	// Physical variable 1 -- Brake shoe friction coefficient
	for (size_t i = 0; i < physicalVariables[1]->intervals.size(); i++) {
		for (size_t j = 0; j < physicalVariables[1]->intervals[i]->points.size(); j++) {
			double x_us = physicalVariables[1]->intervals[i]->points[j]->x;
			double y_us = physicalVariables[1]->intervals[i]->points[j]->y;
			double x_si = UnitConverter::miph_To_Mps(x_us);
			double y_si = y_us;
			physicalVariables[1]->intervals[i]->points[j]->x = x_si;
			physicalVariables[1]->intervals[i]->points[j]->y = y_si;
		}
		((IntervalSmooth*)physicalVariables[1]->intervals[i])->zvcb = false;
	}
	// Couplers
	couplers[0]->convertToSI();
	couplers[1]->convertToSI();
	// Velocity
	ssv[1] = UnitConverter::miph_To_Mps(ssv[1]);
	// Brake pipe pressure
	brakePipeAirPressure = UnitConverter::psi_To_Pa(brakePipeAirPressure);
	// Auxiliary reservoir pressure
	auxiliaryReservoir->pressure = UnitConverter::psi_To_Pa(auxiliaryReservoir->pressure);
	// Emergency reservoir pressure
	emergencyReservoir->pressure = UnitConverter::psi_To_Pa(emergencyReservoir->pressure);
}


void Car::initialize_resultsWriter(Simulation* simulation) {
	std::string absoluteFilePath = inputFileReader_Simulation->inputFileDirectoryPath + "/" + inputFileReader_Simulation->inputFileName + "_" + std::to_string(positionInTrainConsist + 1) + "_car.csv";
	resultsWriter = new ResultsWriter(absoluteFilePath, false);
}


void Car::results(Simulation* simulation, bool whb) {
	if (whb == true) {
		std::vector<std::string> headerLabels;
		headerLabels.push_back("Time (s)");
		headerLabels.push_back("Position (ft)");
		headerLabels.push_back("Velocity (mph)");
		headerLabels.push_back("Track grade (%)");
		headerLabels.push_back("Track curvature (deg)");
		headerLabels.push_back("Track superelevation (in)");
		headerLabels.push_back("Deflection of trailing coupler (in)");
		headerLabels.push_back("Deflection of leading coupler (in)");
		headerLabels.push_back("Longitudinal force applied by trailing coupler (lb)");
		headerLabels.push_back("Longitudinal force applied by leading coupler (lb)");
		headerLabels.push_back("Lateral force applied by trailing coupler (lb)");
		headerLabels.push_back("Lateral force applied by leading coupler (lb)");
		headerLabels.push_back("Maximum L/V ratio");
		headerLabels.push_back("Control valve operating mode");
		headerLabels.push_back("Brake pipe pressure (psi)");
		headerLabels.push_back("Auxiliary reservoir pressure (psi)");
		headerLabels.push_back("Emergency reservoir pressure (psi)");
		headerLabels.push_back("Brake cylinder pressure (psi)");
		resultsWriter->writeLine(headerLabels);
	}
	double temp_var;
	std::vector<std::string> currResults;
	currResults.push_back(std::to_string(simulation->explicitSolverTime));
	temp_var = UnitConverter::m_To_Ft(ssv[0]);
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::mps_To_Miph(ssv[1]);
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::rad_To_Pct(inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[0]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[0]));
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::gCurv_To_TrkCurv(inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[1]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[1], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[1]));
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::rad_To_InSup(inputFileReader_Simulation->userDefinedTracks[0]->physicalVariables[2]->
		interpolate(ssv[0], inputFileReader_Simulation->userDefinedTracks[0]->PVDMIN_SI[2], inputFileReader_Simulation->userDefinedTracks[0]->PVDMAX_SI[2]));
	currResults.push_back(std::to_string(temp_var));
	if (positionInTrainConsist < ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1)) {
		temp_var = UnitConverter::m_To_In(couplers[1]->displacement);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	if (positionInTrainConsist > 0) {
		temp_var = UnitConverter::m_To_In(couplers[0]->displacement);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	if (positionInTrainConsist < ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1)) {
		temp_var = UnitConverter::n_To_Lb(tangentialForceDueToTrailingCoupler);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	if (positionInTrainConsist > 0) {
		temp_var = UnitConverter::n_To_Lb(tangentialForceDueToLeadingCoupler);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	if (positionInTrainConsist < ((int)inputFileReader_Simulation->userDefinedTrainConsists[0]->railVehicles.size() - 1)) {
		temp_var = UnitConverter::n_To_Lb(lateralForceDueToTrailingCoupler);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	if (positionInTrainConsist >= 1) {
		temp_var = UnitConverter::n_To_Lb(lateralForceDueToLeadingCoupler);
		currResults.push_back(std::to_string(temp_var));
	}
	else {
		currResults.push_back("N/A");
	}
	temp_var = lOverVMax;
	currResults.push_back(std::to_string(temp_var));
	temp_var = ((ControlValve_Car*)controlValve)->currentOperatingMode;
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::pa_To_Psi(brakePipeAirPressure);
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::pa_To_Psi(auxiliaryReservoir->pressure);
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::pa_To_Psi(emergencyReservoir->pressure);
	currResults.push_back(std::to_string(temp_var));
	temp_var = UnitConverter::pa_To_Psi(brakeCylinder->pressure);
	currResults.push_back(std::to_string(temp_var));
	resultsWriter->writeLine(currResults);
}


void Car::loadPhysicalConstantAlternateNames() {
	mass = physicalConstants[0];
	length = physicalConstants[1];
	brakePipeLength = physicalConstants[1] * 1.1;
	numberOfAxles = (int)physicalConstants[2];
	crossSectionalArea = physicalConstants[3];
	streamliningCoefficient = physicalConstants[4];
	maxNetBrakingRatio = physicalConstants[5];
	handBrakeStatus = (int)physicalConstants[6];
	handBrakeEfficiencyRatio = physicalConstants[7];
	truckCenterSpacing = physicalConstants[8];
	couplerHeight = physicalConstants[9];
	centerOfGravityHeight = physicalConstants[10];
}


void Car::calc_brakingForce() {
	double BF = brakeCylinder->retardingBrakeForce;
	if (handBrakeStatus == 1) {
		double HBF = handBrakeEfficiencyRatio * mass * TrainConsist::GRAVITATIONAL_CONSTANT;
		if (ssv[1] > 0.0) {
			HBF = -HBF;
		}
		BF = BF + HBF;
	}
	brakingForce = BF;
}


void Car::calc_throttleForce() {
	throttleForce = 0.0;
}

