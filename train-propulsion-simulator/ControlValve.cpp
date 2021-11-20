//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "ControlValve.h"
#include "Function.h"


ControlValve::ControlValve() {
	brakeCylinderPressureFunction = new Function(false, 2);
}


ControlValve::~ControlValve() {}


void ControlValve::calc_brakeCylinderPressureFunction(int brakeCylinderPressureFunction_NumberDataPoints) {
	brakeCylinderPressureFunction->piecewiseLinear(brakeCylinderPressureFunction_Domain, brakeCylinderPressureFunction_Range, brakeCylinderPressureFunction_NumberDataPoints);
}

