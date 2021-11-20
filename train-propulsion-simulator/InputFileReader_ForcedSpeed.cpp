//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include <vector>
#include "InputFileReader_ForcedSpeed.h"
#include "InputFileReader_Simulation.h"
#include "Simulation.h"
#include "Track.h"
#include "UnitConverter.h"


InputFileReader_ForcedSpeed::InputFileReader_ForcedSpeed(InputFileReader_Simulation* inputFileReader_Simulation) : InputFileReader() {
	this->inputFileReader_Simulation = inputFileReader_Simulation;
	inputFileAbsolutePath = inputFileReader_Simulation->inputFileAbsolutePath;
	calc_inputFileDirectoryPath();
	calc_inputFileName();
	inputFileAbsolutePath = inputFileDirectoryPath + "/" + "forced_speed" + "." + FE_FSF;
	curind = -1;
	// Check if file exists
	inputFileExistsBool = false;
	FILE* file;
	if (file = fopen(inputFileAbsolutePath.c_str(), "r")) {
		inputFileExistsBool = true;
		fclose(file);
	}
	/*
	FILE* file;
	errno_t err1;
	if (err1 = fopen_s(&file, inputFileAbsolutePath.c_str(), "r") == 0) {
		inputFileExistsBool = true;
		fclose(file);
	}
	*/
	// If file exists, calculate number of lines, file directory path, and file name
	if (inputFileExistsBool == true) {
		calc_totalNumberOfLines();
		totalNumberOfForcedSpeeds = totalNumberOfLines - 1;
		calc_inputFileDirectoryPath();
		calc_inputFileName();
		inputFileStream = new std::ifstream(inputFileAbsolutePath);
		inputFileStream->open(inputFileAbsolutePath);
		// Allocate space for independent and dependent variable arrays
		fsiv = new double[totalNumberOfLines - 1];
		fsdv = new double[totalNumberOfLines - 1];
	}
	// Reinitialize
	reinitialize();
}


InputFileReader_ForcedSpeed::~InputFileReader_ForcedSpeed() {
	if (inputFileExistsBool == true) {
		delete[] fsiv;
		delete[] fsdv;
	}
}


std::string InputFileReader_ForcedSpeed::load() {
	// Check that first row has one integer that is equal to '0' or '1'
	currentLineNumber = 0;
	nextLine();
	std::vector<std::string> strvec = InputFileReader::split_string(currentLine, ',');
	if (strvec.size() != 1) {
		return std::string("First line of forced speed file must have only one value equal to '0' for distance-based or '1' for time-based.");
	}
	try {
		DBTBIFS = std::stoi(strvec[0]);
		double DBTBIFS_check = std::stod(strvec[0]);
		if (DBTBIFS != DBTBIFS_check) {
			return std::string("Error reading integer on first line of forced speed file.");
		}
	}
	catch (const std::invalid_argument& ia) {
		return std::string("Error reading integer on first line of forced speed file.");
	}
	if ((DBTBIFS != 0) && (DBTBIFS != 1)) {
		return std::string("First line of forced speed file must be integer equal to '0' for distance-based or '1' for time-based.");
	}
	// Check that each row (after first row) has exactly two elements
	reinitialize();
	currentLineNumber = 0;
	for (int i = 0; i < totalNumberOfLines; i++) {
		nextLine();
		if (i > 0) {
			strvec.clear();
			strvec = InputFileReader::split_string(currentLine, ',');
			if (strvec.size() != 2) {
				return std::string("Each line in forced speed file (except for first line) must have two values separated by a comma.");
			}
			try {
				std::stod(strvec[0]);
				std::stod(strvec[1]);
			}
			catch (const std::invalid_argument& ia) {
				return std::string("Error reading number in forced speed file.");
			}
			size_t numcommas = std::count(currentLine.begin(), currentLine.end(), ',');
			if (numcommas != 1) {
				return std::string("Wrong number of commas entered.  Only 1 comma expected.");
			}
		}
	}
	// Check for positive and increasing domain values
	reinitialize();
	currentLineNumber = 0;
	std::vector<std::string> strvec2;
	for (int i = 0; i < (totalNumberOfLines - 1); i++) {
		nextLine();
		if (i > 0) {
			if (i == 1) {
				strvec.clear();
				strvec = InputFileReader::split_string(currentLine, ',');
				if (std::stod(strvec[0]) <= 0.0) {
					return std::string("Values of independent variable must be positive.");
				}
				nextLine();
				strvec2.clear();
				strvec2 = InputFileReader::split_string(currentLine, ',');
			}
			else {
				strvec[0] = strvec2[0];
				strvec[1] = strvec2[1];
				strvec2.clear();
				strvec2 = InputFileReader::split_string(currentLine, ',');
			}
			if (std::stod(strvec2[0]) <= 0.0) {
				return std::string("Values of independent variable must be positive.");
			}
			if ((std::stod(strvec2[0]) < std::stod(strvec[0]))) {
				return std::string("Values of independent variable must be increasing.");
			}
			if ((DBTBIFS == 0) && (std::stod(strvec2[0]) > inputFileReader_Simulation->userDefinedTracks[0]->trackLength)) {
				return std::string("Values of independent variable must be less than length of track (") +
					std::to_string(inputFileReader_Simulation->userDefinedTracks[0]->trackLength) + std::string(" feet).");
			}
			else if ((DBTBIFS == 1) && (std::stod(strvec2[0]) > Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS)) {
				return std::string("Values of independent variable must be less than ") + std::to_string(Simulation::MAX_NUMBER_OF_SIMULATED_SECONDS) + std::string(" seconds.");
			}
		}
	}
	// Check for allowable range values
	reinitialize();
	currentLineNumber = 0;
	for (int i = 0; i < totalNumberOfLines; i++) {
		nextLine();
		if (i > 0) {
			strvec.clear();
			strvec = InputFileReader::split_string(currentLine, ',');
			if ((std::stod(strvec[1]) < 0.0) || (std::stod(strvec[1]) > MAXAFS)) {
				return std::string("Values of dependent variable must be greater than or equal to 0.0 miles per hour and less than or equal to ") +
					std::to_string(MAXAFS) + std::string(" miles per hour.");
			}
		}
	}
	// Load data and convert from US units to SI units
	reinitialize();
	currentLineNumber = 0;
	for (int i = 0; i < totalNumberOfLines; i++) {
		nextLine();
		if (i > 0) {
			strvec.clear();
			strvec = InputFileReader::split_string(currentLine, ',');
			double d0 = std::stod(strvec[0]);
			fsiv[i - 1] = d0;
			double d1 = std::stod(strvec[1]);
			fsdv[i - 1] = d1;
		}
	}
	return InputFileReader::VALID_INPUT_STRING;
}


// Converts independent and dependent values to SI units
void InputFileReader_ForcedSpeed::convertToSI() {
	for (int i = 0; i < (totalNumberOfLines - 1); i++) {
		if (DBTBIFS == 0) {
			fsiv[i] = UnitConverter::ft_To_M(fsiv[i]);
		}
		fsdv[i] = UnitConverter::miph_To_Mps(fsdv[i]);
	}
}

