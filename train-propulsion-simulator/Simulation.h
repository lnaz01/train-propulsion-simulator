//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SIMULATION_DEF
#define SIMULATION_DEF

#include <string>
#include "UserDefinedRRComponent.h"
#include "UnitConverter.h"

class InputFileReader_ForcedSpeed;
class InputFileReader_Simulation;
class ResultsWriter;

class Simulation : public UserDefinedRRComponent {

public:

	Simulation(InputFileReader_Simulation* inputFileReader_Simulation);

	virtual ~Simulation();

	// Simulation input file reader
	InputFileReader_Simulation* inputFileReader_Simulation;

	// Maximum time (seconds)
	static constexpr double MAX_NUMBER_OF_SIMULATED_SECONDS = 10800.0;  // 10,800 seconds = 180 minutes

	// Fixed time step (seconds) for implicit solver
	const double IMPLICIT_SOLVER_FIXED_TIME_STEP = 0.02;

	// Current simulation time for implicit solver (seconds)
	double implicitSolverTime;

	// Current simulation time for explicit solver (seconds)
	double explicitSolverTime;

	// Current time step size for explicit solver (seconds)
	double explicitSolverTimeStep;

	// Explicit solver integration type ('0' for fixed time step; '1' for variable time step)
	int explicitSolverType;

	// Number of steps for explicit solver
	int explicitSolverNumSteps;

	// Explicit solver step index
	int explicitSolverStepIndex;

	// Sampling rate (hertz)
	int sampleRate;

	void convertToSI() override;

	std::string load() override;

	// Simulates train dynamics
	// inputFileReader_ForcedSpeed	-->	Forced speed input file reader
	void simulate(InputFileReader_ForcedSpeed* inputFileReader_ForcedSpeed);

protected:

	void loadPhysicalConstantAlternateNames() override;

private:

	// Spatial cushion for start and end of track (feet)
	const double TRACK_SPATIAL_CUSHION_US = 528.0;  // 528 feet = 0.1 miles

	// Spatial cushion for start and end of track (meters)
	const double TRACK_SPATIAL_CUSHION = UnitConverter::ft_To_M(TRACK_SPATIAL_CUSHION_US);

	// Maximum allowable train speed (miles / hour)
	const double MAX_ALLOWABLE_TRAIN_SPEED_US = 150.0;

	// Maximum allowable train speed (meters / second)
	const double MAX_ALLOWABLE_TRAIN_SPEED = UnitConverter::miph_To_Mps(MAX_ALLOWABLE_TRAIN_SPEED_US);

	// Approximate zero velocity (miles / hour)
	const double APPROX_ZERO_VELOCITY_US = 1.0;

	// Approximate zero velocity (meters / second)
	const double APPROX_ZERO_VELOCITY = UnitConverter::miph_To_Mps(APPROX_ZERO_VELOCITY_US);

	// Maximum wait time for positive velocity (seconds)
	const double MAX_WAIT_POSITIVE_VELOCITY = 1800.0;  // 1800 seconds = 30 minutes

	// Elapsed simulation time for updating progress (seconds)
	const int ELAPSED_TIME_FOR_UPDATE_PROGRESS = (int)(MAX_NUMBER_OF_SIMULATED_SECONDS / 100.0);

	// Minimum sampling rate (hertz)
	const int MINIMUM_SAMPLING_RATE = 5;

	// Maximum sampling rate (hertz)
	const int MAXIMUM_SAMPLING_RATE = 1000;

	// Minimum time step (seconds) for variable time step explicit solver
	const double EXPLICIT_SOLVER_MIN_TIME_STEP = 1.0 * pow(10.0, -15.0);

	// Maximum time step (seconds) for variable time step explicit solver
	const double EXPLICIT_SOLVER_MAX_TIME_STEP = 0.02;

	// Fixed time step (seconds) for fixed time step explicit solver
	const double EXPLICIT_SOLVER_FIXED_TIME_STEP = 0.004;

	// Maximum error tolerance (used in variable time-step solver)
	const double EXPLICIT_SOLVER_MAX_ERROR_THRESHOLD = 1.0 * pow(10.0, -9.0);

	// Velocity approximately zero or below boolean
	bool velocityApproxZeroBool;

	// Start time of approximately zero or below velocity
	double startTimeVelocityApproxZero;

	// Point-in-train consist of rail vehicles to save
	int* railVehiclesToSave;

	// Size of point-in-train consist of rail vehicles to save
	int railVehiclesToSaveSize;

	// Point-it-train consist of rail vehicles to save created boolean
	bool railVehiclesToSaveCreatedBool;

	// Output results file writer for brake pipe pressure results
	ResultsWriter* resultsWriter_BrakePipes;

	// Output results file writer for auxiliary reservoir pressure results
	ResultsWriter* resultsWriter_AuxiliaryReservoirs;

	// Output results file writer for emergency reservoir pressure results
	ResultsWriter* resultsWriter_EmergencyReservoirs;

	// Output results file writer for coupler force results
	ResultsWriter* resultsWriter_CouplerForces;

	// Output results file writer for coupler displacement results
	ResultsWriter* resultsWriter_CouplerDisplacements;

	// Performs single time step integration using RKF45 algorithm
	// (returns 'true' if:	1. Time step is too small or 2. Coupler tension/compression is too large)
	bool explicitSolverIntegrationStep();

	// Calculate number of steps for explicit solver
	void calc_explicitSolverNumSteps();

	// Places train consist on track
	void placeTrainConsistOnTrack();

	// Checks if train consist is inside track segment boundaries
	bool checkTrainConsistIsOnTrack();

	// Checks if train consist is inside track segment boundaries
	bool checkTrainConsistIsMoving();

	// Checks if train speed is under maximum allowable speed
	bool checkTrainSpeedIsUnderMaximumAllowableSpeed();

	// Writes time step results for brake pipe pressures
	// whb		-->	Write headers boolean
	void writeResults_BrakePipePressures(bool whb);

	// Writes time step results for auxiliary reservoir pressures
	// whb		-->	Write headers boolean
	void writeResults_AuxiliaryReservoirPressures(bool whb);

	// Writes time step results for emergency reservoir pressures
	// whb		-->	Write headers boolean
	void writeResults_EmergencyReservoirPressures(bool whb);

	// Writes time step results for coupler forces
	// whb		-->	Write headers boolean
	void writeResults_CouplerForces(bool whb);

	// Writes time step results for coupler forces
	// whb		-->	Write headers boolean
	void writeResults_CouplerDisplacements(bool whb);

	// Initializes output results file writer
	void initializeResultsWriters();

	// Closes (and flushes) results writers
	// simulationSuccessfulBool		-->	Simulation successful boolean
	void closeResultsWriters(bool simulationSuccessfulBool);

};

#endif
