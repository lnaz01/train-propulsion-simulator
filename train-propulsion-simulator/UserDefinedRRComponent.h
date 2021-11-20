//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef USER_DEFINED_RR_COMPONENT_DEF
#define USER_DEFINED_RR_COMPONENT_DEF

#include <string>
#include <vector>

class Function;
class InputFileReader_Simulation;

class UserDefinedRRComponent {

public:

	// physicalConstantsSize		-->	Number of physical constants
	// physicalVariablesSize		-->	Number of physical variables
	UserDefinedRRComponent(InputFileReader_Simulation* inputFileReader_Simulation, int physicalConstantsSize, int physicalVariablesSize);

	virtual ~UserDefinedRRComponent();

	// Simulation input file reader
	InputFileReader_Simulation* inputFileReader_Simulation;

	// Start user-defined component string
	std::string START_UDRRC_STRING;

	// Physical variable independent component US units
	std::string* PVISTR_US;

	// Physical variable dependent component US units
	std::string* PVDSTR_US;

	// Physical variable dependent component integer boolean ('true' for integer; 'false' for double)
	bool* PVDINT;

	// Physical variable independent component minimum threshold values in US units
	double* PVIMIN_US;

	// Physical variable independent component minimum threshold values in SI units
	double* PVIMIN_SI;

	// Physical variable independent component minimum maximum threshold values in US units
	// NOTE: This variable stores the minimum allowed value for the maximum threshold value.  
	// For example, a user-defined track can be a maximum of 1,056,000 feet (200 miles), 
	// but it must be at least 52,800 feet (10 miles) long.  This value of 10 miles is an 
	// example of a value stored in this 'PVIMINMAX_US' variable.  The value of 200 miles 
	// would be stored in the 'PVIMAX_US' variable.
	double* PVIMINMAX_US;

	// Physical variable independent component maximum threshold values in US units
	double* PVIMAX_US;

	// Physical variable independent component maximum threshold values in SI units
	double* PVIMAX_SI;

	// Physical variable dependent component minimum threshold values in US units
	double* PVDMIN_US;

	// Physical variable dependent component minimum threshold values in SI units
	double* PVDMIN_SI;

	// Physical variable dependent component maximum threshold values in US units
	double* PVDMAX_US;

	// Physical variable dependent component maximum threshold values in SI units
	double* PVDMAX_SI;

	// Component type
	// 0	-->	Track
	// 1	-->	Coupler
	// 2	-->	Car
	// 3	-->	Locomotive
	// 4	-->	Locomotive operator
	// 5	-->	Train consist
	// 6	-->	Simulation
	int componentType;

	// Physical variables
	std::vector<Function*> physicalVariables;

	// Converts model to SI units
	virtual void convertToSI() = 0;

	// Loads from file
	// (NOTE: This function is 'virtual' because it is overloaded in the 'Simulation' class.)
	virtual std::string load();

	// Prints to console
	void console();

protected:

	// End user-defined component string
	std::string END_UDRRC_STRING;

	// Physical constants US units
	std::string* PCSTR_US;

	// Physical constants integer boolean ('true' for integer; 'false' for double)
	bool* PCINT;

	// Physical constant minimum threshold values in US units
	double* PCMIN_US;

	// Physical constant minimum threshold values in SI units
	double* PCMIN_SI;

	// Physical constant maximum threshold values in US units
	double* PCMAX_US;

	// Physical constant maximum threshold values in SI units
	double* PCMAX_SI;

	// Number of physical constants
	int physicalConstantsSize;

	// Physical constants
	double* physicalConstants;

	// Number of physical variables
	int physicalVariablesSize;

	// Loads alternate user-inputted physical constant names
	virtual void loadPhysicalConstantAlternateNames() = 0;

private:

	// Copies user-defined component
	void copy(UserDefinedRRComponent* userDefinedRRComponent);

};

#endif
