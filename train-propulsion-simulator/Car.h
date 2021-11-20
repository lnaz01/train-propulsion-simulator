//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CAR_DEF
#define CAR_DEF

#include "RailVehicle.h"

class AuxiliaryReservoir;
class EmergencyReservoir;
class EndOfTrainDevice;
class InputFileReader_Simulation;
class Simulation;

class Car : public RailVehicle {

public:

	Car(InputFileReader_Simulation* inputFileReader_Simulation);

	virtual ~Car();

	// Auxiliary reservoir
	AuxiliaryReservoir* auxiliaryReservoir;

	// Emergency reservoir
	EmergencyReservoir* emergencyReservoir;

	// End-of-train device
	EndOfTrainDevice* endOfTrainDevice;

	void convertToSI() override;

	void initialize_resultsWriter(Simulation* simulation) override;

	void results(Simulation* simulation, bool whb) override;

	void calc_brakingForce() override;

	void calc_throttleForce() override;

	// Initialize end-of-train device
	void initialize_endOfTrainDevice();

protected:

	void loadPhysicalConstantAlternateNames() override;

};

#endif
