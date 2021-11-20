//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "ControlValve_Locomotive.h"
#include "BrakeCylinder.h"
#include "Function.h"
#include "Locomotive.h"
#include "LocomotiveOperator.h"
#include "TrainConsist.h"


ControlValve_Locomotive::ControlValve_Locomotive(Locomotive* locomotive) : ControlValve() {
	this->locomotive = locomotive;
	int brakeCylinderPressureFunction_NumberDataPoints = 3;
	brakeCylinderPressureFunction_Domain = new double[brakeCylinderPressureFunction_NumberDataPoints];
	brakeCylinderPressureFunction_Domain[0] = TrainConsist::ATMOSPHERIC_PRESSURE;
	brakeCylinderPressureFunction_Domain[1] = locomotive->brakeCylinder->maxPressureForFullServiceBraking;
	brakeCylinderPressureFunction_Domain[2] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
	brakeCylinderPressureFunction_Range = new double[brakeCylinderPressureFunction_NumberDataPoints];
	brakeCylinderPressureFunction_Range[0] = locomotive->brakeCylinder->maxPressureForEmergencyBraking;
	brakeCylinderPressureFunction_Range[1] = locomotive->brakeCylinder->maxPressureForFullServiceBraking;
	brakeCylinderPressureFunction_Range[2] = TrainConsist::ATMOSPHERIC_PRESSURE;
	calc_brakeCylinderPressureFunction(brakeCylinderPressureFunction_NumberDataPoints);
}


ControlValve_Locomotive::~ControlValve_Locomotive() {
	delete[] brakeCylinderPressureFunction_Domain;
	delete[] brakeCylinderPressureFunction_Range;
	delete brakeCylinderPressureFunction;
}

