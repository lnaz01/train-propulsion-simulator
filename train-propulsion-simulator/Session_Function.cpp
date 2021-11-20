//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Session_Function.h"
#include "Car.h"
#include "Coupler.h"
#include "Function.h"
#include "InputFileReader_Function.h"
#include "InputFileReader_Simulation.h"
#include "Locomotive.h"
#include "Point.h"
#include "ResultsWriter.h"
#include "Track.h"


// main
int main() {
	Session_Function* session_Function = new Session_Function();
	delete session_Function;
}


Session_Function::Session_Function() {
	std::cout << std::endl;
	std::cout << "TRAIN PROPULSION SIMULATOR -- FUNCTION VISUALIZER (build " << BUILD_ID << ")" << std::endl;
	std::cout << std::endl;
	// Prompt user and get input data file path
	std::cout << "Enter full path of your input data file:" << std::endl;
	std::cout << std::endl;
	std::string inputFilePath;
	std::cin >> inputFilePath;
	std::cout << std::endl;
	inputFileReader_Function = new InputFileReader_Function(inputFilePath);
	if (inputFileReader_Function->inputFileExistsBool == false) {
		std::cout << "File path entered is not valid." << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	// Prompt user for component type
	std::cout << "Enter component type ('Track' for track, 'Coupler' for coupler, 'Car' for car, 'Locomotive' for locomotive):" << std::endl;
	std::cout << std::endl;
	std::string componentType;
	std::cin >> componentType;
	std::cout << std::endl;
	inputFileReader_Simulation = new InputFileReader_Simulation();
	track = new Track(inputFileReader_Simulation);
	coupler = new Coupler(inputFileReader_Simulation);
	car = new Car(inputFileReader_Simulation);
	locomotive = new Locomotive(inputFileReader_Simulation);
	if (componentType.compare("Track") == 0) {
		inputFileReader_Function->userDefinedRailroadComponent = track;
	}
	else if (componentType.compare("Coupler") == 0) {
		inputFileReader_Function->userDefinedRailroadComponent = coupler;
	}
	else if (componentType.compare("Car") == 0) {
		inputFileReader_Function->userDefinedRailroadComponent = car;
	}
	else if (componentType.compare("Locomotive") == 0) {
		inputFileReader_Function->userDefinedRailroadComponent = locomotive;
	}
	else {
		std::cout << "Invalid component type entered." << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	// Prompt user for function index
	std::cout << "Enter function index: " << std::endl;
	std::cout << std::endl;
	std::string strfni;
	std::cin >> strfni;
	std::cout << std::endl;
	int fni = 0;
	try {
		fni = std::stoi(strfni);
		double fni_check = std::stod(strfni);
		if (fni != fni_check) {
			std::cout << "Input could not be parsed to integer." << std::endl;
			std::cout << std::endl;
			std::cout << "Press 'Enter' to end program." << std::endl;
			std::cin.ignore();
			exit(EXIT_SUCCESS);
		}
	}
	catch (const std::invalid_argument& ia) {
		std::cout << "Input could not be parsed to integer." << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	fni--;
	if ((fni < 0) || (fni > ((int)inputFileReader_Function->userDefinedRailroadComponent->physicalVariables.size() - 1))) {
		std::cout << "Invalid function index entered." << std::endl;
		std::cout << std::endl;
		std::string userDefinedRailroadComponentType = "";
		if (componentType.compare("Track") == 0) {
			userDefinedRailroadComponentType = "Track";
		}
		else if (componentType.compare("Coupler") == 0) {
			userDefinedRailroadComponentType = "Coupler";
		}
		else if (componentType.compare("Car") == 0) {
			userDefinedRailroadComponentType = "Car";
		}
		else if (componentType.compare("Locomotive") == 0) {
			userDefinedRailroadComponentType = "Locomotive";
		}
		std::cout << "A " << userDefinedRailroadComponentType << " has " << inputFileReader_Function->userDefinedRailroadComponent->physicalVariables.size() <<
			" functions.  Therefore, a function index between 1 and " << inputFileReader_Function->userDefinedRailroadComponent->physicalVariables.size() <<
			" was expected." << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	inputFileReader_Function->functionIndex = fni;
	// Load function parameters from file
	std::string fnval = inputFileReader_Function->load();
	if (fnval.compare(InputFileReader::VALID_INPUT_STRING) != 0) {
		std::cout << "Error on line " << inputFileReader_Function->currentLineNumber << ": " << fnval << std::endl;
		std::cout << std::endl;
		std::cout << "Press 'Enter' to end program." << std::endl;
		std::cin.ignore();
		exit(EXIT_SUCCESS);
	}
	// Print results
	std::string outputFilePath = inputFileReader_Function->inputFileDirectoryPath + "/" + FILE_NAME + ".csv";
	resultsWriter = new ResultsWriter(outputFilePath, false);
	inputFileReader_Function->userDefinedRailroadComponent->physicalVariables[fni]->interpolateMult(inputFileReader_Function->userDefinedRailroadComponent->PVDMIN_US[fni],
		inputFileReader_Function->userDefinedRailroadComponent->PVDMAX_US[fni]);
	double** d = inputFileReader_Function->userDefinedRailroadComponent->physicalVariables[fni]->interpPoints;
	int dSize1 = inputFileReader_Function->userDefinedRailroadComponent->physicalVariables[fni]->numInterpPointsPlot;
	Point* points = new Point[dSize1];
	for (int i = 0; i < dSize1; i++) {
		points[i].x = d[i][0];
		points[i].y = d[i][1];
	}
	resultsWriter->writePoints(points, dSize1);
	resultsWriter->ofs->flush();
	resultsWriter->ofs->close();
	// Message user
	std::cout << "Interpolate function values successfully written to file titled '" + FILE_NAME + ".csv'." << std::endl;
	std::cout << std::endl;
	std::cout << "Press 'Enter' to end program." << std::endl;
	std::cin.ignore();
	exit(EXIT_SUCCESS);
	// Delete allocated memory
	delete[] points;
}


Session_Function::~Session_Function() {
	delete inputFileReader_Function;
	delete inputFileReader_Simulation;
	delete track;
	delete coupler;
	delete car;
	delete locomotive;
	delete resultsWriter;
}

