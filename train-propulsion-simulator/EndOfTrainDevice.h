//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef END_OF_TRAIN_DEVICE
#define END_OF_TRAIN_DEVICE

class Car;
class Locomotive;

class EndOfTrainDevice {

public:

	EndOfTrainDevice(Car* car);

	virtual ~EndOfTrainDevice();

	// Two-way EOT activated boolean
	bool twoWayEOTActivatedBool;

	// Current pressure (boundary condition) after two-way EOT activation
	double pressureAfterTwoWayEOTActivation;

	// Calculates pressure (boundary condition) after two-way EOT activation
	void calc_pressureAfterTwoWayEOTActivation();

private:

	// Car
	Car* car;

	// Corresponding locomotive
	Locomotive* correspondingLocomotive;

	// EOT two-way opening period (seconds)
	const double TWO_WAY_EOT_OPENING_PERIOD = 4.0;

	// Time of two-way EOT activation
	double timeOf2WayEOTActivation;

	// Pressure at time of two-way EOT activation
	double pressureAtTimeOfTwoWayEOTActivation;

	// Rate of change of boundary condition pressure after two-way EOT activation
	double rateOfChangeOfPressureAfterTwoWayEOTActivation;

};

#endif
