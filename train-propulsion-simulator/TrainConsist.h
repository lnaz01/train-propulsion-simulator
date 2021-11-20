//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAIN_CONSIST_DEF
#define TRAIN_CONSIST_DEF

#include <string>
#include <vector>
#include "UnitConverter.h"
#include "UserDefinedRRComponent.h"

class BrakePipe_Cumulative;
class CouplingSystem;
class InputFileReader_Simulation;
class Locomotive;
class RailVehicle;

class TrainConsist : public UserDefinedRRComponent {

public:

	TrainConsist(InputFileReader_Simulation* ifrsim);

	virtual ~TrainConsist();

	// Calculates mass flow rate from component with normalized pressure Pup (upstream component) into component with normalized pressure Pexp (downstream component)
	// px				-->	Pressure in component x (pascals)
	// py				-->	Pressure in component y (pascals)
	// restrictionArea	-->	Restriction area (meters^2)
	// airTemperature	-->	Air temperature (kelvin)
	static double massFlowRate(double Pexp, double Pup, double restrictionArea, double airTemperature);

	// Atmospheric pressure (pascals)
	static const double ATMOSPHERIC_PRESSURE;

	// Individual gas constant of air (joules / (kilogram * kelvin))
	static constexpr double GAS_CONSTANT_AIR = 287.0;

	// Gravitational constant of earth (meters / second)
	static constexpr double GRAVITATIONAL_CONSTANT = 9.8;

	// Maximum number of rail vehicles
	static const int MAX_NUMBER_RAIL_VEHICLES = 300;

	// Number of inputs per locomotive
	const int NUMBER_OF_LOCOMOTIVE_INPUTS = 5;

	// Number of inputs per car
	const int NUMBER_OF_CAR_INPUTS = 7;

	// Minimum initial rail vehicle velocity (mph)
	const double MINRVVEL = 0.0;

	// Maximum initial rail vehicle velocity (mph)
	const double MAXRVVEL = 90.0;

	// Vector of rail vehicles
	std::vector<RailVehicle*> railVehicles;

	// Vector of coupling systems
	std::vector<CouplingSystem*> couplingSystems;

	// Vector of locomotives
	std::vector<Locomotive*> locomotives;

	// Vector of cumulative brake pipes
	std::vector<BrakePipe_Cumulative*> brakePipes_Cumulative;

	// Array of rail vehicle types ('0' for car; '1' for locomotive)
	int* railVehicleTypes;

	// Train consist location on track, which is defined as location of first rail vehicle in train consist (feet)
	double locationOnTrack;

	// Air dynamic viscosity (kg / (m * s))
	double airViscosity;

	// Air temperature (kelvin)
	double airTemperature;

	// End-of-train device capability ('1' for one-way; '2' for two-way)
	int eotDeviceCapability;

	void convertToSI() override;

	// Calculates vector of locomotive models
	void calc_locomotives();

	// Calculates array of rail vehicle types
	void calc_railVehicleTypes();

	// Calculates vector of cumulative brake pipes
	void calc_brakePipes_Cumulative();

	// Sets initial pressure and 'm' value for brake pipe nodes
	void calculateInitialPressuresAndMValuesForBPFENodes();

	// Calculates train consist location on track
	void calc_locationOnTrack();

	// Calculates air dynamic viscosity which depends on air temperature
	void calc_airViscosity();

	// Adds rail vehicle to vector of rail vehicles
	void add_railVehicle(RailVehicle* railVehicle);

	// Configures coupling systems
	void configureCouplingSystems();

protected:

	void loadPhysicalConstantAlternateNames() override;

private:

	// Array of rail vehicle types defined boolean
	bool railVehicleTypesDefinedBool;

};

#endif
