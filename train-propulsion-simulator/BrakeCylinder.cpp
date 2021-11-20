//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "BrakeCylinder.h"
#include "AuxiliaryReservoir.h"
#include "Car.h"
#include "ControlValve_Car.h"
#include "EmergencyReservoir.h"
#include "Function.h"
#include "InputFileReader_Simulation.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "Simulation.h"
#include "TrainConsist.h"


BrakeCylinder::BrakeCylinder(RailVehicle* railVehicle) {
	this->railVehicle = railVehicle;
	if (railVehicle->componentType == 2) {
		railVehicleType = 0;
	}
	else {
		railVehicleType = 1;
	}
	calc_maxPressureForFullServiceBraking();
	calc_maxPressureForEmergencyBraking();
}


BrakeCylinder::~BrakeCylinder() {}


void BrakeCylinder::calc_maxPressureForFullServiceBraking() {
	double numerator0 = (TrainConsist::ATMOSPHERIC_PRESSURE * (INITIAL_VOLUME + (PISTON_AREA * PISTON_DISP_WHEEL_CONTACT))) + (LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE * AuxiliaryReservoir::VOLUME) +
		(TrainConsist::ATMOSPHERIC_PRESSURE * (PIPING_VOLUME_BC_CV + PIPING_VOLUME_AUXRES_CV));
	double denominator0 = (INITIAL_VOLUME + (PISTON_AREA * PISTON_DISP_WHEEL_CONTACT)) + AuxiliaryReservoir::VOLUME + (PIPING_VOLUME_BC_CV + PIPING_VOLUME_AUXRES_CV);
	maxPressureForFullServiceBraking = numerator0 / denominator0;
}


void BrakeCylinder::calc_maxPressureForEmergencyBraking() {
	double numerator0 = (TrainConsist::ATMOSPHERIC_PRESSURE * (INITIAL_VOLUME + (PISTON_AREA * PISTON_DISP_WHEEL_CONTACT))) + (LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE * AuxiliaryReservoir::VOLUME) +
		(LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE * EmergencyReservoir::VOLUME) + (TrainConsist::ATMOSPHERIC_PRESSURE * (PIPING_VOLUME_BC_CV + PIPING_VOLUME_AUXRES_CV + PIPING_VOLUME_EMRES_CV));
	double denominator0 = (INITIAL_VOLUME + (PISTON_AREA * PISTON_DISP_WHEEL_CONTACT)) + AuxiliaryReservoir::VOLUME + EmergencyReservoir::VOLUME + (PIPING_VOLUME_BC_CV + PIPING_VOLUME_AUXRES_CV + PIPING_VOLUME_EMRES_CV);
	maxPressureForEmergencyBraking = numerator0 / denominator0;
}


void BrakeCylinder::calc_retardingBrakeForce() {
	retardingBrakeForce = railVehicle->physicalVariables[1]->interpolate(std::abs(railVehicle->ssv[1]), railVehicle->PVDMIN_SI[1],
		railVehicle->PVDMAX_SI[1]) * normalShoeWheelForce;
	if (railVehicle->ssv[1] > 0.0) {
		retardingBrakeForce = -retardingBrakeForce;
	}
}


void BrakeCylinder::calc_normalShoeWheelForce() {
	if (pistonForce > 0.0) {
		normalShoeWheelForce = railVehicle->brakeRiggingLeverageRatio * railVehicle->physicalVariables[0]->interpolate(pressure, railVehicle->PVDMIN_SI[0], railVehicle->PVDMAX_SI[0]) * pistonForce;
	}
	else {
		normalShoeWheelForce = 0.0;
	}
}


void BrakeCylinder::calc_pistonForce() {
	pistonForce = ((pressure - TrainConsist::ATMOSPHERIC_PRESSURE) * PISTON_AREA) - (SPRING_STIFFNESS * PISTON_DISP_WHEEL_CONTACT) - SPRING_PRE_LOAD - FRICTION_FORCE;
}


void BrakeCylinder::calc_pressure(bool initialPressureCalculationBool) {
	if (railVehicleType == 0) {
		if (initialPressureCalculationBool == true) {
			double bpp = railVehicle->brakePipeAirPressure;
			pressure = ((Car*)railVehicle)->controlValve->brakeCylinderPressureFunction->interpolate(bpp, TrainConsist::ATMOSPHERIC_PRESSURE, LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE);
		}
		else {
			double pipingVolume = PIPING_VOLUME_BC_CV;
			if (((ControlValve_Car*)((Car*)railVehicle)->controlValve)->currentOperatingMode == 1) {
				pipingVolume = pipingVolume + PIPING_VOLUME_AUXRES_CV;
			}
			else if (((ControlValve_Car*)((Car*)railVehicle)->controlValve)->currentOperatingMode == 3) {
				pipingVolume = pipingVolume + PIPING_VOLUME_AUXRES_CV + PIPING_VOLUME_EMRES_CV;
			}
			pressure = pressure + (((TrainConsist::GAS_CONSTANT_AIR * railVehicle->inputFileReader_Simulation->userDefinedTrainConsists[0]->airTemperature) / (LOADED_VOLUME + pipingVolume)) * ((ControlValve_Car*)((Car*)railVehicle)->controlValve)->mdot_bc * railVehicle->inputFileReader_Simulation->userDefinedSimulations[0]->IMPLICIT_SOLVER_FIXED_TIME_STEP);
			if (pressure < TrainConsist::ATMOSPHERIC_PRESSURE) {
				pressure = TrainConsist::ATMOSPHERIC_PRESSURE;
			}
		}
	}
	else {
		double cibvs;  // current independent brake valve setting
		cibvs = ((Locomotive*)railVehicle)->currentIndependentBrakeValveSetting;
		pressure = ((Locomotive*)railVehicle)->controlValve->brakeCylinderPressureFunction->interpolate(cibvs, TrainConsist::ATMOSPHERIC_PRESSURE, LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE);
	}
}

