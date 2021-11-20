//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <sstream>
#include "InputFileReader.h"


const std::string InputFileReader::VALID_INPUT_STRING = "VALID";


InputFileReader::InputFileReader(std::string inputFileAbsolutePath) {
	this->inputFileAbsolutePath = inputFileAbsolutePath;
	// Check if file exists
	inputFileExistsBool = false;
	FILE* file1;
	if (file1 = fopen(inputFileAbsolutePath.c_str(), "r")) {
		inputFileExistsBool = true;
		fclose(file1);
	}
	/*
	FILE* file1;
	errno_t err1;
	if (err1 = fopen_s(&file1, inputFileAbsolutePath.c_str(), "r") == 0) {
		inputFileExistsBool = true;
		fclose(file1);
	}
	*/
	// If file exists, calculate number of lines, file directory path, and file name
	if (inputFileExistsBool == true) {
		calc_totalNumberOfLines();
		calc_inputFileDirectoryPath();
		calc_inputFileName();
		inputFileStream = new std::ifstream(inputFileAbsolutePath);
		inputFileStream->open(inputFileAbsolutePath);
	}
	// Reinitialize
	reinitialize();
}


InputFileReader::InputFileReader() {
	inputFileExistsBool = false;
	totalNumberOfLines = 0;
	currentLineNumber = 0;
}


InputFileReader::~InputFileReader() {
	if (inputFileExistsBool == true) {
		delete inputFileStream;
	}
}


std::vector<std::string> InputFileReader::split_string(std::string str, char c) {
	std::stringstream ss1(str);
	std::string tempstr1;
	std::vector<std::string> vecstr1;
	while (std::getline(ss1, tempstr1, c)) {
		vecstr1.push_back(tempstr1);
	}
	return vecstr1;
}


void InputFileReader::reinitialize() {
	if (inputFileExistsBool == true) {
		currentLineNumber = 0;
		inputFileStream->close();
		inputFileStream->open(inputFileAbsolutePath);
	}
}


void InputFileReader::nextLine() {
	currentLineNumber++;
	getline((*inputFileStream), currentLine);
	removeWhiteSpaceAndComments();
}


void InputFileReader::calc_inputFileDirectoryPath() {
	size_t numInstancesOfForwardSlash = count(inputFileAbsolutePath.begin(), inputFileAbsolutePath.end(), '/');
	size_t positionLastSlash = 0;
	if (numInstancesOfForwardSlash > 0) {
		positionLastSlash = inputFileAbsolutePath.find_last_of("/");
	}
	else {
		positionLastSlash = inputFileAbsolutePath.find_last_of("\\");
	}
	inputFileDirectoryPath = inputFileAbsolutePath.substr(0, positionLastSlash);
}


void InputFileReader::calc_inputFileName() {
	size_t numInstancesOfForwardSlash = count(inputFileAbsolutePath.begin(), inputFileAbsolutePath.end(), '/');
	size_t positionLastSlash = 0;
	if (numInstancesOfForwardSlash > 0) {
		positionLastSlash = inputFileAbsolutePath.find_last_of("/");
	}
	else {
		positionLastSlash = inputFileAbsolutePath.find_last_of("\\");
	}
	size_t positionLastPeriod = inputFileAbsolutePath.find_last_of(".");
	size_t lengthOfFileNameWithoutExtension = positionLastPeriod - positionLastSlash - 1;
	inputFileName = inputFileAbsolutePath.substr(positionLastSlash + 1, lengthOfFileNameWithoutExtension);
}


void InputFileReader::calc_totalNumberOfLines() {
	totalNumberOfLines = 1;
	int currentChar;
	FILE* file1;
	if (file1 = fopen(inputFileAbsolutePath.c_str(), "r")) {
		while ((currentChar = getc(file1)) != EOF) {
			if (currentChar == '\n') {
				totalNumberOfLines++;
			}
		}
		fclose(file1);
	}
	/*
	FILE* file1;
	errno_t err1;
	err1 = fopen_s(&file1, inputFileAbsolutePath.c_str(), "r");
	if (!err1) {
		while ((currentChar = getc(file1)) != EOF) {
			if (currentChar == '\n') {
				totalNumberOfLines++;
			}
		}
		fclose(file1);
	}
	*/
}


bool InputFileReader::fileIsEmpty() {
	bool result1 = false;
	if (inputFileStream->peek() == std::ifstream::traits_type::eof()) {
		result1 = true;
	}
	return result1;
}


void InputFileReader::removeWhiteSpaceAndComments() {
	// Remove white space
	std::string currentLine_withNoWhiteSpace;
	for (size_t i = 0; i < currentLine.length(); i++) {
		if (currentLine[i] == ' ') {
			continue;
		}
		else {
			currentLine_withNoWhiteSpace = currentLine_withNoWhiteSpace + currentLine[i];
		}
	}
	currentLine = currentLine_withNoWhiteSpace;
	// Remove tabs
	std::string currentLine_withNoTabs;
	for (size_t i = 0; i < currentLine.length(); i++) {
		if (currentLine[i] == '\t') {
			continue;
		}
		else {
			currentLine_withNoTabs = currentLine_withNoTabs + currentLine[i];
		}
	}
	currentLine = currentLine_withNoTabs;
	// Remove comments
	std::string currentLine_withCommentsRemoved;
	for (size_t i = 0; i < currentLine.length(); i++) {
		if (currentLine[i] == COMMENT_CHARACTER) {
			break;
		}
		else {
			currentLine_withCommentsRemoved = currentLine_withCommentsRemoved + currentLine[i];
		}
	}
	currentLine = currentLine_withCommentsRemoved;
}

