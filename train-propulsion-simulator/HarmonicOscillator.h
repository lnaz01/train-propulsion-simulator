//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef HARMONIC_OSCILLATOR_DEF
#define HARMONIC_OSCILLATOR_DEF

#include "DynamicSystem.h"

// Definition of a harmonic oscillator class capable of simulating multiple single-degree-of-freedom harmonic oscillators in series.
// It is assumed that the spring and damper of the first harmonic oscillator is fixed to a rigid surface.
class HarmonicOscillator : public DynamicSystem {

public:

	// numMasses	-->	Number of masses
	// m			-->	Array of masses (kg)
	// c			--> Array of damping constants
	// k			-->	Array of spring constants
	HarmonicOscillator(int numMasses, double* m, double* c, double* k);

	virtual ~HarmonicOscillator();

protected:

	void calc_ssvdot() override;

	void calc_ff() override;

private:

	// Mass
	double* m;

	// Spring constant
	double* k;

	// Damper
	double* c;

};

#endif
