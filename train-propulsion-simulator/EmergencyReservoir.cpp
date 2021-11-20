//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "EmergencyReservoir.h"
#include "Car.h"
#include "ControlValve_Car.h"
#include "InputFileReader_Simulation.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


const double EmergencyReservoir::VOLUME = UnitConverter::in3_To_M3(3500.0);


EmergencyReservoir::EmergencyReservoir(Car* car) {
	this->car = car;
	pressure = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
}


EmergencyReservoir::~EmergencyReservoir() {}


void EmergencyReservoir::calc_pressure() {
	pressure = pressure + (((TrainConsist::GAS_CONSTANT_AIR * car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature) / VOLUME) * ((ControlValve_Car*)car->controlValve)->mdot_er * car->inputFileReader_Simulation->userDefinedSimulations[0]->IMPLICIT_SOLVER_FIXED_TIME_STEP);
	if (pressure < TrainConsist::ATMOSPHERIC_PRESSURE) {
		pressure = TrainConsist::ATMOSPHERIC_PRESSURE;
	}
}

