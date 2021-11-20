//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "LocomotiveOperator.h"
#include "InputFileReader_Simulation.h"
#include "Track.h"


int main() {

	// Track
	std::string inputFileAbsolutePath_TRACK = "C:/Users/Leith/Desktop/MyTrack__test.txt";
	InputFileReader_Simulation* inputFileReaderSimulation_TRACK = new InputFileReader_Simulation(inputFileAbsolutePath_TRACK);
	inputFileReaderSimulation_TRACK->userDefinedTracks.push_back(new Track(inputFileReaderSimulation_TRACK));
	std::string validString_TRACK = inputFileReaderSimulation_TRACK->userDefinedTracks[0]->load();
	std::cout << "Line " << inputFileReaderSimulation_TRACK->currentLineNumber << ": " << validString_TRACK << std::endl;
	std::cout << std::endl;
	inputFileReaderSimulation_TRACK->userDefinedTracks[0]->console();
	std::cout << std::endl;
	if (validString_TRACK.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
		std::cout << "Failed to load track" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Locomotive operator
	std::string inputFileAbsolutePath_LOCOMOTIVE_OPERATOR = "C:/Users/Leith/Desktop/MyLocomotiveOperator__test.txt";
	InputFileReader_Simulation* inputFileReaderSimulation_LOCOMOTIVE_OPERATOR = new InputFileReader_Simulation(inputFileAbsolutePath_LOCOMOTIVE_OPERATOR);
	inputFileReaderSimulation_LOCOMOTIVE_OPERATOR->userDefinedTracks.push_back(inputFileReaderSimulation_TRACK->userDefinedTracks[0]);
	inputFileReaderSimulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators.push_back(new LocomotiveOperator(inputFileReaderSimulation_LOCOMOTIVE_OPERATOR));
	std::string validString_LOCOMOTIVE_OPERATOR = inputFileReaderSimulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators[0]->load();
	std::cout << "Line " << inputFileReaderSimulation_LOCOMOTIVE_OPERATOR->currentLineNumber << ": " << validString_LOCOMOTIVE_OPERATOR << std::endl;
	std::cout << std::endl;
	inputFileReaderSimulation_LOCOMOTIVE_OPERATOR->userDefinedLocomotiveOperators[0]->console();

}

