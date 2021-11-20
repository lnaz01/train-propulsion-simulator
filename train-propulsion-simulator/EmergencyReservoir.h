//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef EMERGENCY_RESERVOIR_DEF
#define EMERGENCY_RESERVOIR_DEF

class Car;

class EmergencyReservoir {

public:

	EmergencyReservoir(Car* car);

	virtual ~EmergencyReservoir();

	// Car
	Car* car;

	// Volume (meters^3)
	static const double VOLUME;

	// Pressure (Newtons / meters^2)
	double pressure;

	// Calculates air pressure
	void calc_pressure();

};

#endif
