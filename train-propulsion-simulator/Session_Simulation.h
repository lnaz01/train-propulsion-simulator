//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SESSION_SIMULATION
#define SESSION_SIMULATION

#include <string>

class InputFileReader_ForcedSpeed;
class InputFileReader_Simulation;

class Session_Simulation {

public:

	Session_Simulation();

	virtual ~Session_Simulation();

private:

	// Simulation input file reader
	InputFileReader_Simulation* inputFileReader_Simulation;

	// Forced speed input file reader
	InputFileReader_ForcedSpeed* inputFileReader_ForcedSpeed;

	// Version number
	const std::string buildID = "1.01.01";

};

#endif
