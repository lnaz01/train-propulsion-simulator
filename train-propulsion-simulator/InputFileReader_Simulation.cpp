//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include <vector>
#include "InputFileReader_Simulation.h"
#include "Car.h"
#include "Coupler.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "Track.h"
#include "TrainConsist.h"


InputFileReader_Simulation::InputFileReader_Simulation(std::string inputFileAbsolutePath) : InputFileReader(inputFileAbsolutePath) {}


InputFileReader_Simulation::InputFileReader_Simulation() : InputFileReader() {}


InputFileReader_Simulation::~InputFileReader_Simulation() {
	for (size_t i = 0; i < userDefinedTracks.size(); i++) {
		delete userDefinedTracks[i];
	}
	for (size_t i = 0; i < userDefinedCouplers.size(); i++) {
		delete userDefinedCouplers[i];
	}
	for (size_t i = 0; i < userDefinedCars.size(); i++) {
		delete userDefinedCars[i];
	}
	for (size_t i = 0; i < userDefinedLocomotives.size(); i++) {
		delete userDefinedLocomotives[i];
	}
	for (size_t i = 0; i < userDefinedLocomotiveOperators.size(); i++) {
		delete userDefinedLocomotiveOperators[i];
	}
	for (size_t i = 0; i < userDefinedTrainConsists.size(); i++) {
		delete userDefinedTrainConsists[i];
	}
	for (size_t i = 0; i < userDefinedSimulations.size(); i++) {
		delete userDefinedSimulations[i];
	}
}


std::string InputFileReader_Simulation::load() {
	// Check for empty file
	bool fileEmpty = fileIsEmpty();
	if (fileIsEmpty() == true) {
		return std::string("Input data file appears to be empty.");
	}
	// Re-initialize input file reader
	reinitialize();
	// Initialize loop counter
	int loopcount = 0;
	// Do loop
	Track track_temp(this);
	Coupler coupler_temp(this);
	Car car_temp(this);
	Locomotive locomotive_temp(this);
	LocomotiveOperator locomotiveOperator_temp(this);
	TrainConsist trainConsist_temp(this);
	Simulation simulation_temp(this);
	do {
		// Invalid line
		if ((currentLine.compare("") != 0) &&
			(currentLine[0] != InputFileReader::COMMENT_CHARACTER) &&
			(currentLine.compare(track_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(coupler_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(car_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(locomotive_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(locomotiveOperator_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(trainConsist_temp.START_UDRRC_STRING) != 0) &&
			(currentLine.compare(simulation_temp.START_UDRRC_STRING) != 0)) {
			return std::string("Comment line or line indicating start of ") +
				std::string("track ('") +
				std::string(track_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("coupler ('") +
				std::string(coupler_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("car ('") +
				std::string(car_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("locomotive ('") +
				std::string(locomotive_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("locomotive operator ('") +
				std::string(locomotiveOperator_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("train consist ('") +
				std::string(trainConsist_temp.START_UDRRC_STRING) +
				std::string("'), ") +
				std::string("or simulation ('") +
				std::string(simulation_temp.START_UDRRC_STRING) +
				std::string("') expected.");
		}
		// Track
		if (currentLine.compare(track_temp.START_UDRRC_STRING) == 0) {
			if (userDefinedTracks.size() == 0) {
				userDefinedTracks.push_back(new Track(this));
				std::string str1 = userDefinedTracks[userDefinedTracks.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else {
				return std::string("Track already defined.  Only one track can be defined.");
			}
		}
		// Coupler
		if (currentLine.compare(coupler_temp.START_UDRRC_STRING) == 0) {
			if ((userDefinedTrainConsists.size() == 0) && (userDefinedCouplers.size() < TrainConsist::MAX_NUMBER_RAIL_VEHICLES)) {
				userDefinedCouplers.push_back(new Coupler(this));
				std::string str1 = userDefinedCouplers[userDefinedCouplers.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else if (userDefinedTrainConsists.size() > 0) {
				return std::string("Train consist already defined.  Coupler can only be defined before train consist is defined.");
			}
			else if (userDefinedCouplers.size() >= TrainConsist::MAX_NUMBER_RAIL_VEHICLES) {
				return std::string("Maximum allowable number of couplers (") + std::to_string(TrainConsist::MAX_NUMBER_RAIL_VEHICLES) + std::string(") already defined.");
			}
		}
		// Car
		if (currentLine.compare(car_temp.START_UDRRC_STRING) == 0) {
			if ((userDefinedTrainConsists.size() == 0) && ((userDefinedCars.size() + userDefinedLocomotives.size()) < TrainConsist::MAX_NUMBER_RAIL_VEHICLES)) {
				userDefinedCars.push_back(new Car(this));
				std::string str1 = userDefinedCars[userDefinedCars.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else if (userDefinedTrainConsists.size() > 0) {
				return std::string("Train consist already defined.  Car can only be defined before train consist is defined.");
			}
			else if (userDefinedCouplers.size() >= TrainConsist::MAX_NUMBER_RAIL_VEHICLES) {
				return std::string("Maximum allowable number of rail vehicles (") + std::to_string(TrainConsist::MAX_NUMBER_RAIL_VEHICLES) + std::string(") already defined.");
			}
		}
		// Locomotive
		if (currentLine.compare(locomotive_temp.START_UDRRC_STRING) == 0) {
			if ((userDefinedTrainConsists.size() == 0) && ((userDefinedCars.size() + userDefinedLocomotives.size()) < TrainConsist::MAX_NUMBER_RAIL_VEHICLES)) {
				userDefinedLocomotives.push_back(new Locomotive(this));
				std::string str1 = userDefinedLocomotives[userDefinedLocomotives.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else if (userDefinedTrainConsists.size() > 0) {
				return std::string("Train consist already defined.  Locomotive can only be defined before train consist is defined.");
			}
			else if (userDefinedCouplers.size() >= TrainConsist::MAX_NUMBER_RAIL_VEHICLES) {
				return std::string("Maximum allowable number of rail vehicles (") + std::to_string(TrainConsist::MAX_NUMBER_RAIL_VEHICLES) + std::string(") already defined.");
			}
		}
		// Locomotive operator
		if (currentLine.compare(locomotiveOperator_temp.START_UDRRC_STRING) == 0) {
			if ((userDefinedTracks.size() == 1) && (userDefinedTrainConsists.size() == 0) && (userDefinedLocomotiveOperators.size() < TrainConsist::MAX_NUMBER_RAIL_VEHICLES)) {
				userDefinedLocomotiveOperators.push_back(new LocomotiveOperator(this));
				std::string str1 = userDefinedLocomotiveOperators[userDefinedLocomotiveOperators.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else if (userDefinedTracks.size() == 0) {
				return std::string("Track not yet defined.  Locomotive operator can only be defined after track is defined.");
			}
			else if (userDefinedTrainConsists.size() > 0) {
				return std::string("Train consist already defined.  Locomotive operator can only be defined before train consist is defined.");
			}
			else if (userDefinedCouplers.size() >= TrainConsist::MAX_NUMBER_RAIL_VEHICLES) {
				return std::string("Maximum allowable number of locomotive operators (") + std::to_string(TrainConsist::MAX_NUMBER_RAIL_VEHICLES) +
					std::string(") already defined.");
			}
		}
		// Train consist
		if (currentLine.compare(trainConsist_temp.START_UDRRC_STRING) == 0) {
			if (userDefinedCouplers.size() == 0) {
				return std::string("No couplers previously defined.  Train consist can only be defined after at least one coupler is defined.");
			}
			else if ((userDefinedCars.size() + userDefinedLocomotives.size()) == 0) {
				return std::string("No rail vehicles previously defined.  Train consist can only be defined after at least one rail vehicle ") +
					std::string("(car or locomotive) is defined.");
			}
			else if ((userDefinedLocomotives.size() > 0) && (userDefinedLocomotiveOperators.size() == 0)) {
				return std::string("Locomotives defined but no locomotive operators previously defined.");
			}
			else if ((userDefinedLocomotives.size() == 0) && (userDefinedLocomotiveOperators.size() > 0)) {
				return std::string("Locomotive operators defined but no locomotives previously defined.");
			}
			else if (userDefinedTrainConsists.size() > 0) {
				return std::string("Train consist already defined.  Only one train consist can be defined.");
			}
			else {
				userDefinedTrainConsists.push_back(new TrainConsist(this));
				std::string str1 = userDefinedTrainConsists[userDefinedTrainConsists.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
		}
		// Simulation
		if (currentLine.compare(simulation_temp.START_UDRRC_STRING) == 0) {
			if ((userDefinedTrainConsists.size() > 0) && (userDefinedSimulations.size() == 0)) {
				userDefinedSimulations.push_back(new Simulation(this));
				std::string str1 = userDefinedSimulations[userDefinedSimulations.size() - 1]->load();
				if (str1.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
					return std::string(str1);
				}
			}
			else if (userDefinedTrainConsists.size() == 0) {
				return std::string("Train consist not yet defined.");
			}
			else if (userDefinedSimulations.size() > 0) {
				return std::string("Simulation parameters already defined.");
			}
		}
		// Read next line
		if (userDefinedSimulations.size() == 0) {
			nextLine();
		}
		// Iterate loop counter
		loopcount++;
		if (loopcount > 100000) {
			return std::string("Simulation parameters definition could not be found.");
		}
	} while (userDefinedSimulations.size() == 0);
	// Check that simulation is defined
	if (userDefinedSimulations.size() < 1) {
		return std::string("End of file reached without simulation parameters being defined.");
	}
	return InputFileReader::VALID_INPUT_STRING;
}

