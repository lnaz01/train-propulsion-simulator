//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Track.h"
#include "InputFileReader_Simulation.h"


int main() {

	std::string idfp = "C:/Users/Leith/Desktop/MyTrack__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation = new InputFileReader_Simulation(idfp);
	inputFileReader_Simulation->userDefinedTracks.push_back(new Track(inputFileReader_Simulation));
	std::string valstr = inputFileReader_Simulation->userDefinedTracks[0]->load();
	std::cout << "Line " << inputFileReader_Simulation->currentLineNumber << ": " << valstr << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation->userDefinedTracks[0]->console();

}

