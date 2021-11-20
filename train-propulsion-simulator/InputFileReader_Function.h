//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INPUT_FILE_READER_FUNCTION_DEF
#define INPUT_FILE_READER_FUNCTION_DEF

#include <string>
#include "InputFileReader.h"

class UserDefinedRRComponent;

class InputFileReader_Function : public InputFileReader {

public:

	InputFileReader_Function(std::string inputFileAbsolutePath);

	virtual ~InputFileReader_Function();

	// User-defined railroad component
	UserDefinedRRComponent* userDefinedRailroadComponent;

	// Function index
	int functionIndex;

	// Loads input file
	std::string load() override;

};

#endif
