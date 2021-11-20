//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "UserDefinedRRComponent.h"
#include "AuxiliaryReservoir.h"
#include "Car.h"
#include "Coupler.h"
#include "EmergencyReservoir.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "Interval.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "Track.h"
#include "TrainConsist.h"


UserDefinedRRComponent::UserDefinedRRComponent(InputFileReader_Simulation* inputFileReader_Simulation, int physicalConstantsSize, int physicalVariablesSize) {
	// Input file reader
	this->inputFileReader_Simulation = inputFileReader_Simulation;
	// Physical constants
	this->physicalConstantsSize = physicalConstantsSize;
	physicalConstants = new double[physicalConstantsSize];
	for (int i = 0; i < physicalConstantsSize; i++) {
		physicalConstants[i] = 0.0;
	}
	// Physical variables
	this->physicalVariablesSize = physicalVariablesSize;
	// Physical constants US units
	PCSTR_US = new std::string[physicalConstantsSize];
	// Physical constants integer boolean 
	// ('true' for integer; 'false' for double)
	PCINT = new bool[physicalConstantsSize];
	// Physical constant minimum threshold values in US units
	PCMIN_US = new double[physicalConstantsSize];
	// Physical constant minimum threshold values in SI units
	PCMIN_SI = new double[physicalConstantsSize];
	// Physical constant maximum threshold values in US units
	PCMAX_US = new double[physicalConstantsSize];
	// Physical constant maximum threshold values in SI units
	PCMAX_SI = new double[physicalConstantsSize];
	// Physical variable independent component US units
	PVISTR_US = new std::string[physicalVariablesSize];
	// Physical variable dependent component US units
	PVDSTR_US = new std::string[physicalVariablesSize];
	// Physical variable dependent component integer boolean ('true' for integer; 'false' for double)
	PVDINT = new bool[physicalVariablesSize];
	// Physical variable independent component minimum threshold values in US units
	PVIMIN_US = new double[physicalVariablesSize];
	// Physical variable independent component minimum threshold values in SI units
	PVIMIN_SI = new double[physicalVariablesSize];
	// Physical variable independent component minimum maximum threshold values in US units
	PVIMINMAX_US = new double[physicalVariablesSize];
	// Physical variable independent component maximum threshold values in US units
	PVIMAX_US = new double[physicalVariablesSize];
	// Physical variable independent component maximum threshold values in SI units
	PVIMAX_SI = new double[physicalVariablesSize];
	// Physical variable dependent component minimum threshold values in US units
	PVDMIN_US = new double[physicalVariablesSize];
	// Physical variable dependent component minimum threshold values in SI units
	PVDMIN_SI = new double[physicalVariablesSize];
	// Physical variable dependent component maximum threshold values in US units
	PVDMAX_US = new double[physicalVariablesSize];
	// Physical variable dependent component maximum threshold values in SI units
	PVDMAX_SI = new double[physicalVariablesSize];
}


UserDefinedRRComponent::~UserDefinedRRComponent() {
	// Physical constants US units
	delete[] PCSTR_US;
	// Physical constants integer boolean 
	// ('true' for integer; 'false' for double)
	delete[] PCINT;
	// Physical constant minimum threshold values in US units
	delete[] PCMIN_US;
	// Physical constant minimum threshold values in SI units
	delete[] PCMIN_SI;
	// Physical constant maximum threshold values in US units
	delete[] PCMAX_US;
	// Physical constant maximum threshold values in SI units
	delete[] PCMAX_SI;
	// Physical variable independent component US units
	delete[] PVISTR_US;
	// Physical variable dependent component US units
	delete[] PVDSTR_US;
	// Physical variable dependent component integer boolean ('true' for integer; 'false' for double)
	delete[] PVDINT;
	// Physical variable independent component minimum threshold values in US units
	delete[] PVIMIN_US;
	// Physical variable independent component minimum threshold values in SI units
	delete[] PVIMIN_SI;
	// Physical variable independent component minimum maximum threshold values in US units
	delete[] PVIMINMAX_US;
	// Physical variable independent component maximum threshold values in US units
	delete[] PVIMAX_US;
	// Physical variable independent component maximum threshold values in SI units
	delete[] PVIMAX_SI;
	// Physical variable dependent component minimum threshold values in US units
	delete[] PVDMIN_US;
	// Physical variable dependent component minimum threshold values in SI units
	delete[] PVDMIN_SI;
	// Physical variable dependent component maximum threshold values in US units
	delete[] PVDMAX_US;
	// Physical variable dependent component maximum threshold values in SI units
	delete[] PVDMAX_SI;
	// Physical constants
	delete[] physicalConstants;
	// Physical variables
	for (size_t i = 0; i < physicalVariables.size(); i++) {
		delete physicalVariables[i];
	}
}


void UserDefinedRRComponent::copy(UserDefinedRRComponent* userDefinedRRComponent) {
	// Physical constants
	physicalConstantsSize = userDefinedRRComponent->physicalConstantsSize;
	for (int i = 0; i < physicalConstantsSize; i++) {
		physicalConstants[i] = userDefinedRRComponent->physicalConstants[i];
	}
	// Physical variables
	physicalVariablesSize = userDefinedRRComponent->physicalVariablesSize;
	for (int i = 0; i < physicalVariablesSize; i++) {
		physicalVariables[i]->copy(userDefinedRRComponent->physicalVariables[i]);
	}
	// Locomotive operator-specific requirement
	if (this->componentType == 4) {
		((LocomotiveOperator*)this)->definePhysicalVariableLimits();
	}
}


std::string UserDefinedRRComponent::load() {
	// Load physical constants
	if (physicalConstantsSize > 0) {
		bool physicalConstants_Loaded = false;
		do {
			inputFileReader_Simulation->nextLine();
			if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
				std::vector<std::string> strvec = InputFileReader::split_string(inputFileReader_Simulation->currentLine, ',');
				if (strvec.size() != physicalConstantsSize) {
					return std::string("Wrong number of physical constants entered.  ") + std::to_string(physicalConstantsSize) + std::string(" physical constants expected.");
				}
				size_t numcommas = std::count(inputFileReader_Simulation->currentLine.begin(), inputFileReader_Simulation->currentLine.end(), ',');
				if (numcommas != physicalConstantsSize - 1) {
					return std::string("Wrong number of commas entered.  Only ") + std::to_string(physicalConstantsSize - 1) + std::string(" commas expected.");
				}
				for (size_t i = 0; i < strvec.size(); i++) {
					try {
						if (PCINT[i] == true) {
							physicalConstants[i] = std::stoi(strvec[i]);
							double y_check = std::stod(strvec[i]);
							if (physicalConstants[i] != y_check) {
								return std::string("Physical constant ") + std::to_string(i + 1) + std::string(" could not be parsed into an integer.");
							}
						}
						else {
							physicalConstants[i] = std::stod(strvec[i]);
						}
					}
					catch (const std::invalid_argument& ia) {
						if (PCINT[i] == true) {
							return std::string("Physical constant ") + std::to_string(i + 1) + std::string(" could not be parsed into an integer.");
						}
						else {
							return std::string("Physical constant ") + std::to_string(i + 1) + std::string(" could not be parsed into a number.");
						}
					}
					if (((this->componentType == 2) || (this->componentType == 3)) && (i == 8)) {
						// do nothing -- physical constant 8 (truck center spacing) is checked below
					}
					else {
						if ((physicalConstants[i] < PCMIN_US[i]) || (physicalConstants[i] > PCMAX_US[i])) {
							std::string temp_string_2;
							if (PCSTR_US[i].compare("") == 0) {
								temp_string_2 = "";
							}
							else {
								temp_string_2 = " ";
							}
							return std::string("Value for physical constant ") + std::to_string(i + 1) + std::string(" must be greater than or equal to ") +
								std::to_string(PCMIN_US[i]) + temp_string_2 + PCSTR_US[i] + std::string(" and less than or equal to ") +
								std::to_string(PCMAX_US[i]) + temp_string_2 + PCSTR_US[i] + std::string(".");
						}
					}
				}
				physicalConstants_Loaded = true;
			}
		} while (physicalConstants_Loaded == false);
	}
	// Car-specific physical constant requirements
	if (this->componentType == 2) {
		if (physicalConstants[8] < RailVehicle::MIN_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * physicalConstants[1]) {
			return std::string("Truck center spacing must be greater than or equal to ") +
				std::to_string((int)(RailVehicle::MIN_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * 100)) + std::string("% of the length of the car.");
		}
		else if (physicalConstants[8] > RailVehicle::MAX_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * physicalConstants[1]) {
			return std::string("Truck center spacing must be less than or equal to ") +
				std::to_string((int)(RailVehicle::MAX_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH * 100)) + std::string("% of the length of the car.");
		}
	}
	// Locomotive-specific physical constant requirements
	if (this->componentType == 3) {
		if (physicalConstants[8] < 0.5 * physicalConstants[1]) {
			return std::string("Truck center spacing must be greater than or equal to 50% of the length of the locomotive.");
		}
		else if (physicalConstants[8] > 0.9 * physicalConstants[1]) {
			return std::string("Truck center spacing must be less than or equal to 90% of the length of the locomotive.");
		}
	}
	// Load physical constants alternative names
	loadPhysicalConstantAlternateNames();
	// Locomotive operator-specific requirement
	if (this->componentType == 4) {
		((LocomotiveOperator*)this)->definePhysicalVariableLimits();
	}
	// Train consist-specific requirement
	if (this->componentType == 5) {
		// Load rail vehicles
		inputFileReader_Simulation->nextLine();
		do {
			if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
				// Check number of inputs
				std::vector<std::string> strarr = InputFileReader::split_string(inputFileReader_Simulation->currentLine, ',');
				if ((strarr.size() != ((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS) && (strarr.size() != ((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS)) {
					return std::string("Wrong number of inputs.  ") + std::to_string(((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS) + std::string(" inputs expected for a locomotive, and " +
						std::to_string(((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS) + std::string(" inputs expected for a car."));
				}
				// First element -- rail vehicle type indicator
				std::string rvtyp = strarr[0];	// rail vehicle type ("C" for car, "L" for locomotive)
				if ((rvtyp.compare("C") != 0) && (rvtyp.compare("L") != 0)) {
					return std::string("First element must be capital 'C' for car or capital 'L' for locomotive.");
				}
				// Check number of inputs again based on whether rail vehicle is a car or a locomotive
				if (rvtyp.compare("C") == 0) {
					if (strarr.size() != ((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS) {
						return std::string("Wrong number of inputs.  ") + std::to_string(((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS) + std::string(" inputs expected for a car.");
					}
				}
				else if (rvtyp.compare("L") == 0) {
					if (strarr.size() != ((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS) {
						return std::string("Wrong number of inputs.  ") + std::to_string(((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS) + std::string(" inputs expected for a locomotive.");
					}
				}
				// Check number of commas based on whether rail vehicle is a car or a locomotive
				size_t numcommas = std::count(inputFileReader_Simulation->currentLine.begin(), inputFileReader_Simulation->currentLine.end(), ',');
				if (rvtyp.compare("C") == 0) {
					if (numcommas != ((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS - 1) {
						return std::string("Wrong number of commas entered.  Only ") + std::to_string(((TrainConsist*)this)->NUMBER_OF_CAR_INPUTS - 1) +
							std::string(" commas expected.");
					}
				}
				else if (rvtyp.compare("L") == 0) {
					if (numcommas != ((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS - 1) {
						return std::string("Wrong number of commas entered.  Only ") + std::to_string(((TrainConsist*)this)->NUMBER_OF_LOCOMOTIVE_INPUTS - 1) +
							std::string(" commas expected.");
					}
				}
				// Second element -- rail vehicle index
				int rvind = 0;  // rail vehicle index
				try {
					rvind = std::stoi(strarr[1]);
					double rvind_check = std::stod(strarr[1]);
					if (rvind != rvind_check) {
						if (rvtyp.compare("C") == 0) {
							return std::string("Second element representing car index could not be parsed into an integer.");
						}
						else if (rvtyp.compare("L") == 0) {
							return std::string("Second element representing locomotive index could not be parsed into an integer.");
						}
					}
				}
				catch (const std::invalid_argument& ia) {
					if (rvtyp.compare("C") == 0) {
						return std::string("Second element representing car index could not be parsed into an integer.");
					}
					else if (rvtyp.compare("L") == 0) {
						return std::string("Second element representing locomotive index could not be parsed into an integer.");
					}
				}
				if ((rvtyp.compare("C") == 0) && ((rvind < 1) || (rvind > (int)inputFileReader_Simulation->userDefinedCars.size()))) {
					return std::string("Invalid car index entered.  Only ") + std::to_string(inputFileReader_Simulation->userDefinedCars.size()) + std::string(" cars defined.");
				}
				else if ((rvtyp.compare("L") == 0) && ((rvind < 1) || (rvind > (int)inputFileReader_Simulation->userDefinedLocomotives.size()))) {
					return std::string("Invalid locomotive index entered.  Only ") + std::to_string(inputFileReader_Simulation->userDefinedLocomotives.size()) +
						std::string(" locomotives defined.");
				}
				if (strarr[0].compare("C") == 0) {
					((TrainConsist*)this)->add_railVehicle(new Car(inputFileReader_Simulation));
					((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->copy(inputFileReader_Simulation->userDefinedCars[rvind - 1]);
				}
				else if (strarr[0].compare("L") == 0) {
					((TrainConsist*)this)->add_railVehicle(new Locomotive(inputFileReader_Simulation));
					((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->copy(inputFileReader_Simulation->userDefinedLocomotives[rvind - 1]);
				}
				// Third element -- coupler index
				int cplrind = 0;  // coupler index
				try {
					cplrind = std::stoi(strarr[2]);
					double cplrind_check = std::stod(strarr[2]);
					if (cplrind != cplrind_check) {
						return std::string("Third element representing coupler index could not be parsed into an integer.");
					}
				}
				catch (const std::invalid_argument& ia) {
					return std::string("Third element representing coupler index could not be parsed into an integer.");
				}
				if ((cplrind < 1) || (cplrind > (int)inputFileReader_Simulation->userDefinedCouplers.size())) {
					return std::string("Invalid coupler index entered.  Only ") + std::to_string(inputFileReader_Simulation->userDefinedCars.size()) + std::string(" couplers defined.");
				}
				((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->couplers[0]->copy(inputFileReader_Simulation->userDefinedCouplers[cplrind - 1]);
				((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->couplers[1]->copy(inputFileReader_Simulation->userDefinedCouplers[cplrind - 1]);
				// Fourth element -- velocity
				double rvvel = 0;  // rail vehicle velocity
				try {
					rvvel = stod(strarr[3]);
				}
				catch (const std::invalid_argument& ia) {
					if (rvtyp.compare("C") == 0) {
						return std::string("Fourth element representing car velocity could not be parsed into a number.");
					}
					else if (rvtyp.compare("L") == 0) {
						return std::string("Fourth element representing locomotive velocity could not be parsed into a number.");
					}
				}
				if ((rvvel < ((TrainConsist*)this)->MINRVVEL) || (rvvel > ((TrainConsist*)this)->MAXRVVEL)) {
					if (rvtyp.compare("C") == 0) {
						return std::string("Fourth element representing car velocity must be greater than or equal to ") + std::to_string(((TrainConsist*)this)->MINRVVEL) +
							std::string(" miles per hour and less than or equal to ") + std::to_string(((TrainConsist*)this)->MAXRVVEL) + std::string(" miles per hour.");
					}
					else if (rvtyp.compare("L") == 0) {
						return std::string("Fourth element representing locomotive velocity must be greater than or equal to ") + std::to_string(((TrainConsist*)this)->MINRVVEL) +
							std::string(" miles per hour and less than or equal to ") + std::to_string(((TrainConsist*)this)->MAXRVVEL) + std::string(" miles per hour.");
					}
				}
				((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->ssv[1] = rvvel;
				// Fifth element --	brake pipe pressure for car or locomotive operator index for locomotive
				if (rvtyp.compare("C") == 0) {
					double fifthel = 0;  // car brake pipe pressure
					try {
						fifthel = stod(strarr[4]);
					}
					catch (const std::invalid_argument& ia) {
						return std::string("Fifth element representing car brake pipe pressure could not be parsed into a number.");
					}
					if ((fifthel < round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) || (fifthel > round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE)))) {
						return std::string("Fifth element represents invalid car initial brake pipe pressure.  Value must be greater than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) +
							std::string(" pounds per square inch and less than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE))) +
							std::string(" pounds per square inch.");
					}
					((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1]->brakePipeAirPressure = fifthel;
				}
				else if (rvtyp.compare("L") == 0) {
					int fifthel = 0;  // locomotive operator index
					try {
						fifthel = std::stoi(strarr[4]);
						double fifthel_check = std::stod(strarr[4]);
						if (fifthel != fifthel_check) {
							return std::string("Fifth element representing locomotive operator index could not be parsed into an integer.");
						}
					}
					catch (const std::invalid_argument& ia) {
						return std::string("Fifth element representing locomotive operator index could not be parsed into an integer.");
					}
					if ((fifthel < 1.0) || (fifthel > (int)inputFileReader_Simulation->userDefinedLocomotiveOperators.size())) {
						return std::string("Fifth element represents invalid locomotive operator index.  Only ") +
							std::to_string(inputFileReader_Simulation->userDefinedLocomotiveOperators.size()) + std::string(" locomotive operators defined.");
					}
					((Locomotive*)((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1])->locomotiveOperator->copy(inputFileReader_Simulation->userDefinedLocomotiveOperators[fifthel - 1]);
				}
				// Sixth element for car -- auxiliary reservoir pressure
				if (rvtyp.compare("C") == 0) {
					double sixthel = 0;  // car auxiliary reservoir pressure
					try {
						sixthel = stod(strarr[5]);
					}
					catch (const std::invalid_argument& ia) {
						return std::string("Sixth element representing car auxiliary reservoir pressure could not be parsed into a number.");
					}
					if ((sixthel < round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) || (sixthel > round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE)))) {
						return std::string("Sixth element represents invalid car initial auxiliary reservoir pressure.  Value must be greater than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) +
							std::string(" pounds per square inch and less than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE))) +
							std::string(" pounds per square inch.");
					}
					((Car*)((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1])->auxiliaryReservoir->pressure = sixthel;
				}
				// Seventh element for car -- emergency reservoir pressure
				if (rvtyp.compare("C") == 0) {
					double seventhel = 0;  // car emergency reservoir pressure
					try {
						seventhel = stod(strarr[6]);
					}
					catch (const std::invalid_argument& ia) {
						return std::string("Seventh element representing car emergency reservoir pressure could not be parsed into a number.");
					}
					if ((seventhel < round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) || (seventhel > round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE)))) {
						return std::string("Seventh element represents invalid car initial emergency reservoir pressure.  Value must be greater than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) +
							std::string(" pounds per square inch and less than or equal to ") +
							std::to_string(round(UnitConverter::pa_To_Psi(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE))) +
							std::string(" pounds per square inch.");
					}
					((Car*)((TrainConsist*)this)->railVehicles[((TrainConsist*)this)->railVehicles.size() - 1])->emergencyReservoir->pressure = seventhel;
				}
			}
			inputFileReader_Simulation->nextLine();
		} while (inputFileReader_Simulation->currentLine.compare(END_UDRRC_STRING) != 0);
		if (((TrainConsist*)this)->railVehicles.size() == 0) {
			return std::string("At least one car or locomotive must be added to train consist.");
		}
		else if (((TrainConsist*)this)->railVehicles.size() > ((TrainConsist*)this)->MAX_NUMBER_RAIL_VEHICLES) {
			return std::string("The number of cars and locomotives combined must not exceed ") + std::to_string(((TrainConsist*)this)->MAX_NUMBER_RAIL_VEHICLES) + std::string(".");
		}
	}
	else {
		// Load physical variables
		inputFileReader_Simulation->nextLine();
		int physicalVariables_Loaded = 0;	// number of physical variables loaded
		if (physicalVariablesSize > 0) {
			// Find all physical variables
			do {
				if ((inputFileReader_Simulation->currentLine.compare("") != 0) && (inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
					if ((inputFileReader_Simulation->currentLine.compare(Function::START_FUNCTION_STRING) == 0) && (physicalVariables_Loaded < physicalVariablesSize)) {
						std::string fnstr = physicalVariables[physicalVariables_Loaded]->load(inputFileReader_Simulation, this, physicalVariables_Loaded);
						if (fnstr.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
							return fnstr;
						}
						physicalVariables_Loaded++;
					}
					else {
						return std::string("Comment line or line indicating beginning of function ('") + std::string(Function::START_FUNCTION_STRING) + std::string("') expected.");
					}
				}
				inputFileReader_Simulation->nextLine();
			} while (physicalVariables_Loaded < physicalVariablesSize);
		}
		// Find end-of-component string
		do {
			if ((inputFileReader_Simulation->currentLine.compare("") != 0) &&
				(inputFileReader_Simulation->currentLine[0] != InputFileReader::COMMENT_CHARACTER)) {
				return std::string("Comment line or line indicating end of component expected.");
			}
			inputFileReader_Simulation->nextLine();
		} while (inputFileReader_Simulation->currentLine.compare(UserDefinedRRComponent::END_UDRRC_STRING) != 0);
		// Track-specific physical variable requirements
		if (this->componentType == 0) {
			for (int i = 0; i < (physicalVariables_Loaded - 1); i++) {
				int nils1 = physicalVariables[i]->intervals.size();
				int npts1 = physicalVariables[i]->intervals[nils1 - 1]->points.size();
				double tl1 = physicalVariables[i]->intervals[nils1 - 1]->points[npts1 - 1]->x;
				int nils2 = physicalVariables[i + 1]->intervals.size();
				int npts2 = physicalVariables[i + 1]->intervals[nils2 - 1]->points.size();
				double tl2 = physicalVariables[i + 1]->intervals[nils2 - 1]->points[npts2 - 1]->x;
				if (tl1 != tl2) {
					return std::string("Independent component of last point of last interval must be identical for all track functions defined.");
				}
			}
		}
	}
	return InputFileReader::VALID_INPUT_STRING;
}


void UserDefinedRRComponent::console() {
	// Simulation-specific portion
	if (this->componentType == 6) {
		// Method of integration
		std::cout << "Explicit Solver Type ('0' for fixed time step; '1' for variable time step): " << ((Simulation*)this)->explicitSolverType << std::endl;
		std::cout << std::endl;
		// Sampling rate
		std::cout << "Sampling Rate: " << ((Simulation*)this)->sampleRate << std::endl;
		std::cout << std::endl;
	}
	// Physical constants
	std::cout << "Physical Constants" << std::endl;
	for (int i = 0; i < physicalConstantsSize; i++) {
		std::cout << physicalConstants[i];
		if (i < (physicalConstantsSize - 1)) {
			std::cout << ", ";
		}
		else {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;
	// Physical variables
	for (int i = 0; i < physicalVariablesSize; i++) {
		int currindex = i + 1;
		std::cout << "Physical Variable " << currindex << ": " << std::endl;
		physicalVariables[i]->console();
		std::cout << std::endl;
	}
	// Train consist-specific portion
	if (this->componentType == 5) {
		// Rail vehicles
		for (size_t i = 0; i < ((TrainConsist*)this)->railVehicles.size(); i++) {
			if (((TrainConsist*)this)->railVehicles[i]->componentType == 3) {
				std::cout << (i + 1) << ". Locomotive" << std::endl;
			}
			else if (((TrainConsist*)this)->railVehicles[i]->componentType == 2) {
				std::cout << (i + 1) << ". Car" << std::endl;
			}
			std::cout << std::endl;
			((TrainConsist*)this)->railVehicles[i]->console();
			std::cout << std::endl;
			std::cout << (i + 1) << ". Coupler" << std::endl;
			std::cout << std::endl;
			((TrainConsist*)this)->railVehicles[i]->couplers[0]->console();
			std::cout << std::endl << std::endl << std::endl << std::endl;
		}
	}
}

