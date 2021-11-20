//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROL_VALVE_LOCOMOTIVE_DEF
#define CONTROL_VALVE_LOCOMOTIVE_DEF

#include "ControlValve.h"

class Locomotive;

class ControlValve_Locomotive : public ControlValve {

public:

	ControlValve_Locomotive(Locomotive* locomotive);

	virtual ~ControlValve_Locomotive();

private:

	// Locomotive
	Locomotive* locomotive;

};

#endif
