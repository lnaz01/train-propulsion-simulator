//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "EndOfTrainDevice.h"
#include "Car.h"
#include "InputFileReader_Simulation.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "TrainConsist.h"


EndOfTrainDevice::EndOfTrainDevice(Car* car) {
	this->car = car;
	twoWayEOTActivatedBool = false;
	pressureAfterTwoWayEOTActivation = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
	// Calculate corresponding locomotive
	correspondingLocomotive = NULL;
	if ((car->inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size() > 0) &&
		(car->inputFileReader_Simulation->userDefinedTrainConsists[0]->eotDeviceCapability == 2)) {
		if (car->positionInTrainConsist == 0) {
			correspondingLocomotive = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[0];
		}
		else {
			int nlocs = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives.size();
			correspondingLocomotive = car->inputFileReader_Simulation->userDefinedTrainConsists[0]->locomotives[nlocs - 1];
		}
	}
}


EndOfTrainDevice::~EndOfTrainDevice() {}


void EndOfTrainDevice::calc_pressureAfterTwoWayEOTActivation() {
	if (correspondingLocomotive != NULL) {
		// Determine if two-way EOT device is activated
		if (correspondingLocomotive->currentAutomaticBrakeValveSetting > LocomotiveOperator::EMERGENCY_BRAKING_THRESHOLD) {
			twoWayEOTActivatedBool = false;
		}
		else {
			if (twoWayEOTActivatedBool == false) {
				twoWayEOTActivatedBool = true;
				timeOf2WayEOTActivation = car->inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverTime;
				if (car->positionInTrainConsist == 0) {
					pressureAtTimeOfTwoWayEOTActivation = car->brakePipeAirPressure;
				}
				else {
					pressureAtTimeOfTwoWayEOTActivation = car->brakePipeAirPressure;
				}
				rateOfChangeOfPressureAfterTwoWayEOTActivation =
					(UnitConverter::psi_To_Pa(round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE))) - pressureAtTimeOfTwoWayEOTActivation) / TWO_WAY_EOT_OPENING_PERIOD;
				pressureAfterTwoWayEOTActivation = pressureAtTimeOfTwoWayEOTActivation;
			}
			else {
				double ets2weota;  // elapsed time since two-way EOT activation
				ets2weota = std::abs(car->inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverTime - timeOf2WayEOTActivation);
				if (ets2weota < TWO_WAY_EOT_OPENING_PERIOD) {
					pressureAfterTwoWayEOTActivation = pressureAtTimeOfTwoWayEOTActivation + (ets2weota * rateOfChangeOfPressureAfterTwoWayEOTActivation);
				}
				else {
					pressureAfterTwoWayEOTActivation = UnitConverter::psi_To_Pa(round(UnitConverter::pa_To_Psi(TrainConsist::ATMOSPHERIC_PRESSURE)));
				}
			}
		}
	}
}

