//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LOCOMOTIVE_OPERATOR_DEF
#define LOCOMOTIVE_OPERATOR_DEF

#include "UserDefinedRRComponent.h"

class InputFileReader_Simulation;

class LocomotiveOperator : public UserDefinedRRComponent {

public:

	LocomotiveOperator(InputFileReader_Simulation* inputFileReader_Simulation);

	virtual ~LocomotiveOperator();

	// Brake pipe operating pressure (pascals)
	static const double BRAKE_PIPE_OPERATING_PRESSURE;

	// Threshold for emergency braking (pascals) -- also threshold for activating (releasing to atmosphere) two-way EOT device
	static const double EMERGENCY_BRAKING_THRESHOLD;

	// Distance-based versus time-based indicator ('0' for distance-based; '1' for time-based)
	int DISTANCE_VS_TIME_INDICATOR;

	void convertToSI() override;

	// Defines physical variable limits
	void definePhysicalVariableLimits();

protected:

	void loadPhysicalConstantAlternateNames() override;

private:

	// Minimum brake pipe reduction (pascals)
	static const double MINBPR;

};

#endif
