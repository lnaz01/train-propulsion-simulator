//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIC_SYSTEM_DEF
#define DYNAMIC_SYSTEM_DEF

#include <string>
#include <vector>

class ResultsWriter;

class DynamicSystem {

public:

	// ssvLen1	--> First dimension of state-space variables (initial conditions).  For example, this would be '2' for a mass-spring-damper system since 
	//				there are two state-space variables, namely position and velocity.
	// ssvLen2	-->	Second dimension of initial conditions.  For example, this would be the number of masses in a train (series) of harmonic oscillators.
	DynamicSystem(int ssvLen1, int ssvLen2);

	virtual ~DynamicSystem();

	// Length of first dimension of state-space variables
	int ssvLen1;

	// Length of second dimension of state-space variables
	int ssvLen2;

	// Simulates dynamic system using fourth-order Runge-Kutta algorithm
	// resWriter	-->	Results writer
	// t0			-->	Initial time
	// tf			-->	Final time
	// ic			-->	Initial conditions
	// samprat		-->	Sampling rate (Hertz)
	// ssvi			-->	Vector of first dimension of state space variable ('ssv') indices to be written to output file
	// fts			-->	Fixted time step
	void simulateRK4(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double fts);

	// Simulates dynamic system using fourth-order (with fifth-order correction) Runge-Kutta-Fehlberg algorithm
	// resWriter	-->	Results writer
	// t0			-->	Initial time
	// tf			-->	Final time
	// ic			-->	Initial conditions
	// samprat		-->	Sampling rate (Hertz)
	// ssvi			-->	Vector of first dimension of state space variable ('ssv') indices to be written to output file
	// mints		-->	Minimum time step
	// maxts		-->	Maximum time step
	// maxerrthrsh	-->	Maximum error threshold
	void simulateRKF45(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double mints, double maxts, double maxerrthrsh);

	// Simulates dynamic system using fourth-order (with fifth-order correction) Runge-Kutta-Cash-Karp algorithm
	// resWriter	-->	Results writer
	// t0			-->	Initial time
	// tf			-->	Final time
	// ic			-->	Initial conditions
	// samprat		-->	Sampling rate (Hertz)
	// ssvi			-->	Vector of first dimension of state space variable ('ssv') indices to be written to output file
	// mints		-->	Minimum time step
	// maxts		-->	Maximum time step
	// maxerrthrsh	-->	Maximum error threshold
	// deserrthrsh	-->	Desired error threshold
	void simulateRKCK45(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double mints, double maxts, double maxerrthrsh, double deserrthrsh);

protected:

	// Time step
	double tStep;

	// Time
	double t;

	// Time for algorithm
	double tAlg;

	// State-space variables
	double** ssv;

	// State space variables approximation
	double** ssvApp;

	// Derivative of state-space variables (a.k.a. state-space equations)
	double** ssvdot;

	// Forcing function
	double** ff;

	// Algorithm 'k' values
	double*** kVals;

	// Fourth-order result
	double** res4;

	// Fifth-order result
	double** res5;

	// Estimated error
	double** ee;

	// Calculate time for algorithm
	void calc_tAlg(std::string algName, int algStep);

	// Calculate 'ssvApp' for RK4 or RKCK45 algorithm
	void calc_ssvApp(std::string algName, int algStep);

	// Calculate algorithm 'k' values
	void calc_kVals(int algStep);

	// Calculate fourth-order result for RK4 or RKCK45 algorithm
	void calc_res4(std::string algName);

	// Calculate fifth-order result for RKCK45 algorithm
	void calc_res5(std::string algName);

	// Calculate estimated error
	double calc_ee(double maxerrthrsh);

	// Write time step results
	// resWriter	-->	Results writer
	// ssvi			-->	Vector of first dimension of state space variable ('ssv') indices to be written to output file
	// t			-->	Time
	void writeTimeStepResults(ResultsWriter* resWriter, std::vector<int> ssvi, double t);

	// Calculate state space equation values ('ssvdot')
	virtual void calc_ssvdot() = 0;

	// Calculate forcing functions
	virtual void calc_ff() = 0;

};

#endif
