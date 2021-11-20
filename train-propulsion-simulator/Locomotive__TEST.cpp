//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Locomotive.h"
#include "InputFileReader_Simulation.h"


int main() {

	std::string inputFileAbsolutePath = "C:/Users/Leith/Desktop/MyLocomotive__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation = new InputFileReader_Simulation(inputFileAbsolutePath);
	inputFileReader_Simulation->userDefinedLocomotives.push_back(new Locomotive(inputFileReader_Simulation));
	std::string valstr = inputFileReader_Simulation->userDefinedLocomotives[0]->load();
	std::cout << "Line " << inputFileReader_Simulation->currentLineNumber << ": " << valstr << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation->userDefinedLocomotives[0]->console();

}

