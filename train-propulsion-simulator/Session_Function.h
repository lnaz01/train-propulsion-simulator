//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SESSION_FUNCTION
#define SESSION_FUNCTION

#include <string>

class Car;
class Coupler;
class InputFileReader_Function;
class InputFileReader_Simulation;
class Locomotive;
class ResultsWriter;
class Track;

class Session_Function {

public:

	Session_Function();

	virtual ~Session_Function();

private:

	// Function input file reader
	InputFileReader_Function* inputFileReader_Function;

	// Simulation input file reader
	InputFileReader_Simulation* inputFileReader_Simulation;

	// Track
	Track* track;

	// Coupler
	Coupler* coupler;

	// Car
	Car* car;

	// Locomotive
	Locomotive* locomotive;

	// Output results file writer
	ResultsWriter* resultsWriter;

	// Output file name
	const std::string FILE_NAME = "function";

	// Version number
	const std::string BUILD_ID = "1.01.01";

};

#endif
