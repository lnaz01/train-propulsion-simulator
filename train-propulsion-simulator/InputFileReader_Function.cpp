//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include "InputFileReader_Function.h"
#include "Function.h"
#include "UserDefinedRRComponent.h"


InputFileReader_Function::InputFileReader_Function(std::string inputFileAbsolutePath) : InputFileReader(inputFileAbsolutePath) {}


InputFileReader_Function::~InputFileReader_Function() {}


std::string InputFileReader_Function::load() {
	// Check for empty file
	if (fileIsEmpty()) {
		return std::string("Input file appears to be empty.");
	}
	// Find start function string
	if (currentLine.compare(Function::START_FUNCTION_STRING) != 0) {
		do {
			if ((currentLine.compare("") != 0) && (currentLine[0] != InputFileReader::COMMENT_CHARACTER) &&
				(currentLine.compare(Function::START_FUNCTION_STRING) != 0)) {
				return std::string("Comment line or line indicating start of function ('") + std::string(Function::START_FUNCTION_STRING) + std::string("') expected.");
			}
			nextLine();
		} while (currentLine.compare(Function::START_FUNCTION_STRING) != 0);
	}
	return userDefinedRailroadComponent->physicalVariables[functionIndex]->load(this, userDefinedRailroadComponent, functionIndex);
}

