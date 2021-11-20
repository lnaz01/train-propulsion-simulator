//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INPUT_FILE_READER_FORCED_SPEED_DEF
#define INPUT_FILE_READER_FORCED_SPEED_DEF

#include <string>
#include "InputFileReader.h"

class InputFileReader_Simulation;

class InputFileReader_ForcedSpeed : public InputFileReader {

public:

	InputFileReader_ForcedSpeed(InputFileReader_Simulation* ifrsim);

	virtual ~InputFileReader_ForcedSpeed();

	// Forced speed distance-based versus time-based indicator for forced speed file
	// ('0' for distance-based; '1' for time-based)
	int DBTBIFS;

	// Current index
	int curind;

	// Total number of forced speeds
	int totalNumberOfForcedSpeeds;

	// Forced speed independent values
	double* fsiv;

	// Forced speed dependent values
	double* fsdv;

	// Loads forced speed file
	std::string load() override;

	// Converts independent and dependent values to SI units
	void convertToSI();

private:

	// File extension for forced speed files
	const std::string FE_FSF = "fsf";

	// Maximum allowed forced speed (mph)
	const double MAXAFS = 100.0;

	// Simulation input file reader
	InputFileReader_Simulation* inputFileReader_Simulation;

};

#endif
