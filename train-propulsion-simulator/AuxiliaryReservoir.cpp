//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "AuxiliaryReservoir.h"
#include "Car.h"
#include "ControlValve_Car.h"
#include "InputFileReader_Simulation.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


const double AuxiliaryReservoir::VOLUME = UnitConverter::in3_To_M3(2500.0);


AuxiliaryReservoir::AuxiliaryReservoir(Car* car) {
	this->car = car;
	pressure = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
}


AuxiliaryReservoir::~AuxiliaryReservoir() {}


void AuxiliaryReservoir::calc_pressure() {
	pressure = pressure + (((TrainConsist::GAS_CONSTANT_AIR * car->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature) / VOLUME) * ((ControlValve_Car*)car->controlValve)->mdot_ar * car->inputFileReader_Simulation->userDefinedSimulations[0]->IMPLICIT_SOLVER_FIXED_TIME_STEP);
	if (pressure < TrainConsist::ATMOSPHERIC_PRESSURE) {
		pressure = TrainConsist::ATMOSPHERIC_PRESSURE;
	}
}

