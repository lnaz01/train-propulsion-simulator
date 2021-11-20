//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INPUT_FILE_READER_SIMULATION_DEF
#define INPUT_FILE_READER_SIMULATION_DEF

#include <string>
#include <vector>
#include "InputFileReader.h"

class Car;
class Coupler;
class Locomotive;
class LocomotiveOperator;
class Simulation;
class Track;
class TrainConsist;

class InputFileReader_Simulation : public InputFileReader {

public:

	InputFileReader_Simulation(std::string inputFileAbsolutePath);

	InputFileReader_Simulation();

	virtual ~InputFileReader_Simulation();

	// User-defined tracks
	std::vector<Track*> userDefinedTracks;

	// User-defined couplers
	std::vector<Coupler*> userDefinedCouplers;

	// User-defined cars
	std::vector<Car*> userDefinedCars;

	// User-defined locomotives
	std::vector<Locomotive*> userDefinedLocomotives;

	// User-defined locomotive operators
	std::vector<LocomotiveOperator*> userDefinedLocomotiveOperators;

	// User-defined train consists
	std::vector<TrainConsist*> userDefinedTrainConsists;

	// User-defined simulations
	std::vector<Simulation*> userDefinedSimulations;

	std::string load() override;

};

#endif
