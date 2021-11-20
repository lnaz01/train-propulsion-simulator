//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "TrainConsist.h"
#include "Car.h"
#include "Coupler.h"
#include "InputFileReader_Simulation.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "Track.h"


int main() {

	// Track
	std::string inputFileAbsolutePath_TRACK = "C:/Users/Leith/Desktop/MyTrack__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_TRACK = new InputFileReader_Simulation(inputFileAbsolutePath_TRACK);
	inputFileReader_Simulation_TRACK->userDefinedTracks.push_back(new Track(inputFileReader_Simulation_TRACK));
	std::string validString_TRACK = inputFileReader_Simulation_TRACK->userDefinedTracks[0]->load();
	/*
	std::cout << "Line " << inputFileReader_Simulation_TRACK->currentLineNumber << ": " << validString_TRACK << std::endl;
	std::cout << std::endl;
	std::cout << "TRACK" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_TRACK->userDefinedTracks[0]->console();
	std::cout << std::endl;
	if (validString_TRACK.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
		exit(EXIT_FAILURE);
	}
	*/

	// Coupler
	std::string inputFileAbsolutePath_COUPLER = "C:/Users/Leith/Desktop/MyCoupler__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_COUPLER = new InputFileReader_Simulation(inputFileAbsolutePath_COUPLER);
	inputFileReader_Simulation_COUPLER->userDefinedCouplers.push_back(new Coupler(inputFileReader_Simulation_COUPLER));
	std::string validString_COUPLER = inputFileReader_Simulation_COUPLER->userDefinedCouplers[0]->load();
	/*
	std::cout << "Line " << inputFileReader_Simulation_COUPLER->currentLineNumber << ": " << validString_COUPLER << std::endl;
	std::cout << std::endl;
	std::cout << "COUPLER" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_COUPLER->userDefinedCouplers[0]->console();
	std::cout << std::endl;
	if (validString_COUPLER.compare(InputFileReader_Simulation::VALID_INPUT_STRING) != 0) {
		exit(EXIT_FAILURE);
	}
	*/

	// Car
	std::string inputFileAbsolutePath_CAR = "C:/Users/Leith/Desktop/MyCar__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_CAR = new InputFileReader_Simulation(inputFileAbsolutePath_CAR);
	inputFileReader_Simulation_CAR->userDefinedTrainConsists.push_back(new TrainConsist(inputFileReader_Simulation_CAR));
	inputFileReader_Simulation_CAR->userDefinedSimulations.push_back(new Simulation(inputFileReader_Simulation_CAR));
	inputFileReader_Simulation_CAR->userDefinedCars.push_back(new Car(inputFileReader_Simulation_CAR));
	std::string validString_CAR = inputFileReader_Simulation_CAR->userDefinedCars[0]->load();
	/*
	std::cout << "Line " << inputFileReader_Simulation_CAR->currentLineNumber << ": " << validString_CAR << std::endl;
	std::cout << std::endl;
	std::cout << "CAR" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_CAR->userDefinedCars[0]->console();
	std::cout << std::endl;
	if (validString_CAR.compare(InputFileReader_Simulation::VALID_INPUT_STRING) != 0) {
		exit(EXIT_FAILURE);
	}
	*/

	// Locomotive
	std::string inputFileAbsolutePath_LOCOMOTIVE = "C:/Users/Leith/Desktop/MyLocomotive__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_LOCOMOTIVE = new InputFileReader_Simulation(inputFileAbsolutePath_LOCOMOTIVE);
	inputFileReader_Simulation_LOCOMOTIVE->userDefinedTrainConsists.push_back(new TrainConsist(inputFileReader_Simulation_LOCOMOTIVE));
	inputFileReader_Simulation_LOCOMOTIVE->userDefinedSimulations.push_back(new Simulation(inputFileReader_Simulation_LOCOMOTIVE));
	inputFileReader_Simulation_LOCOMOTIVE->userDefinedLocomotives.push_back(new Locomotive(inputFileReader_Simulation_LOCOMOTIVE));
	std::string validString_LOCOMOTIVE = inputFileReader_Simulation_LOCOMOTIVE->userDefinedLocomotives[0]->load();
	/*
	std::cout << "Line " << inputFileReader_Simulation_LOCOMOTIVE->currentLineNumber << ": " << validString_LOCOMOTIVE << std::endl;
	std::cout << std::endl;
	std::cout << "LOCOMOTIVE" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_LOCOMOTIVE->userDefinedLocomotives[0]->console();
	std::cout << std::endl;
	if (validString_LOCOMOTIVE.compare(InputFileReader_Simulation::VALID_INPUT_STRING) != 0) {
		exit(EXIT_FAILURE);
	}
	*/

	// Locomotive operator
	std::string inputFileAbsolutePath_LOCOMOTIVE_OPERATOR = "C:/Users/Leith/Desktop/MyLocomotiveOperator__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_LOCOMOTIVE_OPERATOR = new InputFileReader_Simulation(inputFileAbsolutePath_LOCOMOTIVE_OPERATOR);
	inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->userDefinedTracks.push_back(inputFileReader_Simulation_TRACK->userDefinedTracks[0]);
	inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators.push_back(new LocomotiveOperator(inputFileReader_Simulation_LOCOMOTIVE_OPERATOR));
	std::string validString_LOCOMOTIVE_OPERATOR = inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators[0]->load();
	/*
	std::cout << "Line " << inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->currentLineNumber << ": " << InputFileReader::VALID_INPUT_STRING << std::endl;
	std::cout << std::endl;
	std::cout << "LOCOMOTIVE OPERATOR" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators[0]->console();
	std::cout << std::endl;
	if (validString_LOCOMOTIVE_OPERATOR.compare(InputFileReader_Simulation::VALID_INPUT_STRING) != 0) {
		exit(EXIT_FAILURE);
	}
	*/

	// Train consist
	std::string inputFileAbsolutePath_TRAIN_CONSIST = "C:/Users/Leith/Desktop/MyTrainConsist__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation_TRAIN_CONSIST = new InputFileReader_Simulation(inputFileAbsolutePath_TRAIN_CONSIST);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedTracks.push_back(inputFileReader_Simulation_TRACK->userDefinedTracks[0]);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedCouplers.push_back(inputFileReader_Simulation_COUPLER->userDefinedCouplers[0]);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedCars.push_back(inputFileReader_Simulation_CAR->userDefinedCars[0]);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedLocomotives.push_back(inputFileReader_Simulation_LOCOMOTIVE->userDefinedLocomotives[0]);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedLocomotiveOperators.push_back(inputFileReader_Simulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators[0]);
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedTrainConsists.push_back(new TrainConsist(inputFileReader_Simulation_TRAIN_CONSIST));
	std::string validString_TRAIN_CONSIST = inputFileReader_Simulation_TRAIN_CONSIST->userDefinedTrainConsists[0]->load();
	std::cout << "Line " << inputFileReader_Simulation_TRAIN_CONSIST->currentLineNumber << ": " << validString_TRAIN_CONSIST << std::endl;
	std::cout << std::endl;
	std::cout << "TRAIN CONSIST" << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation_TRAIN_CONSIST->userDefinedTrainConsists[0]->console();
	std::cout << std::endl;

}

