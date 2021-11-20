//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Session_Simulation.h"
#include "InputFileReader_ForcedSpeed.h"
#include "InputFileReader_Simulation.h"
#include "Simulation.h"
#include "Track.h"


int main() {
	Session_Simulation* session_Simulation = new Session_Simulation();
	delete[] session_Simulation;
}


Session_Simulation::Session_Simulation() {
	std::cout << std::endl;
	std::cout << "TRAIN PROPULSION SIMULATOR  (build " << buildID << ")" << std::endl;
	std::cout << std::endl;
	// Prompt user and get input data file path
	std::cout << "Enter full path of your input data file:" << std::endl;
	std::cout << std::endl;
	std::string inputFilePath;
	std::cin >> inputFilePath;
	std::cout << std::endl;
	inputFileReader_Simulation = new InputFileReader_Simulation(inputFilePath);
	if (inputFileReader_Simulation->inputFileExistsBool == false) {
		std::cout << "File path entered is not valid." << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	// Load simulation parameters from file
	std::string simval = inputFileReader_Simulation->load();
	if (simval.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
		std::cout << "Error on line " << inputFileReader_Simulation->currentLineNumber << ": " << simval << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	// Calculate track length
	inputFileReader_Simulation->userDefinedTracks[0]->calc_trackLength();
	// Load forced speed file parameters
	inputFileReader_ForcedSpeed = new InputFileReader_ForcedSpeed(inputFileReader_Simulation);
	if (inputFileReader_ForcedSpeed->inputFileExistsBool == true) {
		std::string fsval = inputFileReader_ForcedSpeed->load();
		if (fsval.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
			std::cout << "Error on line " << inputFileReader_ForcedSpeed->currentLineNumber << " of forced speed file: " << fsval << std::endl;
			std::cout << std::endl;
			std::cout << "Press 'Enter' to end program." << std::endl;
			std::cin.ignore();
			exit(EXIT_SUCCESS);
		}
	}
	// Run simulation
	inputFileReader_Simulation->userDefinedSimulations[0]->simulate(inputFileReader_ForcedSpeed);
	std::cin.ignore();
	exit(EXIT_SUCCESS);
}


Session_Simulation::~Session_Simulation() {
	delete inputFileReader_Simulation;
	delete inputFileReader_ForcedSpeed;
}

