//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LOCOMOTIVE_DEF
#define LOCOMOTIVE_DEF

#include "RailVehicle.h"
#include "UnitConverter.h"

class InputFileReader_Simulation;
class LocomotiveOperator;

class Locomotive : public RailVehicle {

public:

	Locomotive(InputFileReader_Simulation* inputFileReader_Simulation);

	virtual ~Locomotive();

	// Locomotive operator
	LocomotiveOperator* locomotiveOperator;

	// Current independent brake valve setting (pascals)
	double currentIndependentBrakeValveSetting;

	// Current automatic brake valve setting (pascals)
	double currentAutomaticBrakeValveSetting;

	// Idealized relay valve pressure (pascals)
	double idealizedRelayValvePressure;

	void convertToSI() override;

	void initialize_resultsWriter(Simulation* simulation) override;

	void results(Simulation* sim, bool whb) override;

	void calc_brakingForce() override;

	void calc_throttleForce() override;

	// Calculates current throttle setting
	void calc_currentThrottleSetting();

	// Calculates current dynamic brake setting
	void calc_currentDynamicBrakeSetting();

	// Calculates current independent brake valve setting
	void calc_currentIndependentBrakeValveSetting();

	// Calculates current automatic brake valve setting
	// (Note that the 'nextTimeStepBool' argument is only used if the associated Locomotive Operator has its 
	// 'DISTANCE_VS_TIME_INDICATOR' variable set to '1', which indicates that time is used as the independent 
	// variable for the associated Locomotive Operator.)
	// nextTimeStepBool		-->	Set to 'true' if automatic brake valve setting for next time step is desired, and 
	//							set to 'false' if automatic brake valve setting for current time step is desired
	void calc_currentAutomaticBrakeValveSetting(bool nextTimeStepBool);

	// Calculates current relay valve pressure
	void calc_idealizedRelayValvePressure();

protected:

	void loadPhysicalConstantAlternateNames() override;

private:

	// Automatic brake valve setting rate of change for service braking (pascals per second)
	const double ABVROC_SB = UnitConverter::psi_To_Pa(2.0);

	// Automatic brake valve setting rate of change for emergency braking (pascals per second)
	const double ABVROC_EB = UnitConverter::psi_To_Pa(20.0);

	// Previous automatic brake valve setting (pascals)
	double previousAutomaticBrakeValveSetting;

	// Engine effectiveness ratio
	double engineEffectivenessRatio;

	// Current throttle setting
	double currentThrottleSetting;

	// Current dynamic brake setting (percentage)
	double currentDynamicBrakeSetting;

	// Previous automatic brake valve setting dummy (pascals)
	double previousAutomaticBrakeValveSetting_dummy;

	// Automatic brake valve setting transition period (seconds)
	double automaticBrakeValveSettingTransitionPeriod;

	// Time of last automatic brake valve setting change (seconds)
	double timeofLastAutomaticBrakeValveSettingChange;

};

#endif
