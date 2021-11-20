//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Coupler.h"
#include "InputFileReader_Simulation.h"


int main() {

	std::string inputFileAbsolutePath = "C:/Users/Leith/Desktop/MyCoupler__test.txt";
	InputFileReader_Simulation* inputFileReader_Simulation = new InputFileReader_Simulation(inputFileAbsolutePath);
	inputFileReader_Simulation->userDefinedCouplers.push_back(new Coupler(inputFileReader_Simulation));
	std::string valstr = inputFileReader_Simulation->userDefinedCouplers[0]->load();
	std::cout << "Line " << inputFileReader_Simulation->currentLineNumber << ": " << valstr << std::endl;
	std::cout << std::endl;
	inputFileReader_Simulation->userDefinedCouplers[0]->console();

}

