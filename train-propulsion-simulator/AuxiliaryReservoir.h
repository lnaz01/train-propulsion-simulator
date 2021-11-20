//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef AUXILIARY_RESERVOIR_DEF
#define AUXILIARY_RESERVOIR_DEF

class Car;

class AuxiliaryReservoir {

public:

	AuxiliaryReservoir(Car* car);

	virtual ~AuxiliaryReservoir();

	// Car
	Car* car;

	// Volume (meters^3)
	static const double VOLUME;

	// Pressure (Newtons / meters^2)
	double pressure;

	// Calculates pressure
	void calc_pressure();

};

#endif
