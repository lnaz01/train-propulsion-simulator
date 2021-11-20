//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RAIL_VEHICLE_DEF
#define RAIL_VEHICLE_DEF

#include <string>
#include <vector>
#include "ExplicitSSComponent.h"

class BrakeCylinder;
class ControlValve;
class Coupler;
class ResultsWriter;
class Simulation;
class VectorRotator;

class RailVehicle : public ExplicitSSComponent {

public:

	// nssv		-->	Number of state space variables
	// npcs		-->	Number of user-inputted physical constants
	// npvs		-->	Number of user-inputted physical variables
	RailVehicle(InputFileReader_Simulation* inputFileReader_Simulation, int nssv, int npcs, int npvs);

	virtual ~RailVehicle();

	// Minimum ratio of truck center spacing to rail vehicle length
	static constexpr double MIN_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH = 0.5;

	// Maximum ratio of truck center spacing to car body length
	static constexpr double MAX_RATIO_TRUCK_CENTER_SPACING_TO_RAIL_VEHICLE_LENGTH = 0.95;

	// Control valve
	ControlValve* controlValve;

	// Brake cylinder
	BrakeCylinder* brakeCylinder;

	// Vector of couplers (Two couplers in list.  Coupler at index '0' is leading coupler, and coupler at index '1' is trailing coupler.)
	std::vector<Coupler*> couplers;

	// Output results file writer
	ResultsWriter* resultsWriter;

	// Print results boolean
	bool printResultsBool;

	// Position (index) in train consist
	int positionInTrainConsist;

	// Mass (kilograms)
	double mass;

	// Length (meters)
	double length;

	// Brake rigging leverage ratio
	double brakeRiggingLeverageRatio;

	// Brake pipe length (meters)
	double brakePipeLength;

	// Brake pipe pressure (pascals)
	double brakePipeAirPressure;

	// Tangential force due to trailing coupler
	double tangentialForceDueToTrailingCoupler;

	// Lateral force due to trailing coupler
	double lateralForceDueToTrailingCoupler;

	// Initializes results writer
	// simulation		-->	Simulation
	virtual void initialize_resultsWriter(Simulation* simulation) = 0;

	// Writes time step results
	// writeHeadersBool		-->	Write headers boolean
	virtual void results(Simulation* simulation, bool writeHeadersBool) = 0;

	// Throttle force
	virtual void calc_throttleForce() = 0;

	// Braking force
	virtual void calc_brakingForce() = 0;

	void calc_ssvDot() override;

	// Calculates brake pipe length
	void calc_brakePipeLength();

	// Calculates brake rigging leverage ratio
	void calc_brakeRiggingLeverageRatio();

	// Calculates rotation on z axis
	void calc_theta();

	// Calculates global gravitational force
	void calc_globalGravitationalForce();

	// Calculates three-dimensional vector for global gravitational force
	void calc_globalVector_globalGravitationalForce();

	// Calculates local three-dimensional vector resulting from global gravitational force
	void calcLocal3DVecFromGlobalGravitationalForce();

	// Calculates global reactive centrifugal force
	void calc_globalReactiveCentrifugalForce();

	// Calculates three-dimensional vector for global reactive centrifugal force
	void calc_globalVector_globalReactiveCentrifugalForce();

	// Calculates local three-dimensional vector resulting from global reactive centrifugal force
	void calcLocal3DVecFromGlobalReactiveCentrifugalForce();

	// Calculates sum of external tangential forces (including due to gravity, reactive centrifugal force, propulsion resistance, and curving resistance)
	void calc_sumExternalTangentialForces();

	// Calculates tangential force due to leading coupler
	void calc_tangentialForceDueToLeadingCoupler();

	// Calculates tangential force due to trailing coupler
	void calc_tangentialForceDueToTrailingCoupler();

	// Calculates lateral force due to leading coupler
	void calc_lateralForceDueToLeadingCoupler();

	// Calculates lateral force due to trailing coupler
	void calc_lateralForceDueToTrailingCoupler();

	// Calculates angle between top of rail and coupler
	void calc_angleBetweenTopOfRailAndCoupler();

	// Calculates angle between top of rail and center of gravity
	void calc_angleBetweenTopOfRailAndCenterOfGravity();

	// Calculates first square root term used in L/V calculation
	void calc_firstSquareRootTermForLOverVCalc();

	// Calculates second square root term used in L/V calculation
	void calc_secondSquareRootTermForLOverVCalc();

	// Calculates force applied on track by leading truck due to leading coupler
	void calc_forceOnTrackDueToLeadingTruck();

	// Caclculates force applied on track by trailing truck due to trailing coupler
	void calc_forceOnTrackDueToTrailingTruck();

	// Calculates vertical force applied by right side of leading truck
	void calc_verticalForceRightSideLeadingTruck();

	// Calculates vertical force applied by left side of leading truck
	void calc_verticalForceLeftSideLeadingTruck();

	// Calculates vertical force applied by right side of trailing truck
	void calc_verticalForceRightSideTrailingTruck();

	// Calculates vertical force applied by left side of trailing truck
	void calc_verticalForceLeftSideTrailingTruck();

	// Calculates L/V ratio for right side of leading truck
	void calc_lOverVForRightSideOfLeadingTruck();

	// Calculates L/V ratio for left side of leading truck
	void calc_lOverVForLeftSideOfLeadingTruck();

	// Calculates L/V ratio for right side of trailing truck
	void calc_lOverVForRightSideOfTrailingTruck();

	// Calculates L/V ratio for left side of trailing truck
	void calc_lOverVForLeftSideOfTrailingTruck();

	// Calculates maximum L/V ratio
	void calc_lOverVMax();

	// Calculates angle with respect to leading rail vehicle (radians)
	void calc_alpha_leadingRailVehicle();

	// Calculates angle with respect to trailing rail vehicle (radians)
	void calc_alpha_trailingRailVehicle();

protected:

	// Number of axles
	int numberOfAxles;

	// Cross sectional area (meters^2)
	double crossSectionalArea;

	// Streamlining coefficient
	double streamliningCoefficient;

	// Maximum net braking ratio
	double maxNetBrakingRatio;

	// Hand brake status
	int handBrakeStatus;

	// Hand brake efficiency ratio
	double handBrakeEfficiencyRatio;

	// Truck center spacing (meters)
	double truckCenterSpacing;

	// Coupler height (meters)
	double couplerHeight;

	// Center-of-gravity height (meters)
	double centerOfGravityHeight;

	// Tangential force due to leading coupler
	double tangentialForceDueToLeadingCoupler;

	// Lateral force due to leading coupler
	double lateralForceDueToLeadingCoupler;

	// Braking force
	double brakingForce;

	// Throttle force
	double throttleForce;

	// Maximum L/V ratio
	double lOverVMax;

	// Angle with respect to leading rail vehicle (radians)
	double alpha_leadingRailVehicle;

	// Angle with respect to trailing rail vehicle (radians)
	double alpha_trailingRailVehicle;

private:

	// L/V wheel unloading value
	const double LV_WHEEL_UNLOADING = 100000000.0;

	// Ratio of brake pipe length to car length
	const double BRAKE_PIPE_LENGTH_OVER_CAR_LENGTH = 1.1;

	// Brake rigging leverage ratio safety factor
	const double BRAKE_RIGGING_SAFETY_FACTOR = 1.0;

	// Vector rotator for global gravitational force
	// 0	-->	Tangential (longitudinal) force (newtons)
	// 1	-->	Lateral force (newtons)
	// 2	-->	Vertical force (newtons)
	VectorRotator* vectorRotator_globalGravitationalForce;

	// Vector rotator for global reactive centrifugal force
	// 0	-->	Tangential (longitudinal) force (newtons)
	// 1	-->	Lateral force (newtons)
	// 2	-->	Vertical force (newtons)
	VectorRotator* vectorRotator_globalReactiveCentrifugalForce;

	// Brake pipe air velocity (meters/second)
	double brakePipeAirVelocity;

	// Rotation on z axis
	double theta;

	// Sum of external tangential forces
	double sumExternalTangentialForces;

	// L/V ratio for right side of leading truck
	double lOverVForRightSideOfLeadingTruck;

	// L/V ratio for left side of leading truck
	double lOverVForLeftSideOfLeadingTruck;

	// L/V ratio for right side of trailing truck
	double lOverVForRightSideOfTrailingTruck;

	// L/V ratio for left side of trailing truck
	double lOverVForLeftSideOfTrailingTruck;

	// Angle between top of rail and coupler
	double angleBetweenTopOfRailAndCoupler;

	// Angle between top of rail and center of gravity
	double angleBetweenTopOfRailAndCenterOfGravity;

	// First square root term used in L/V calculation
	double firstSquareRootTermForLOverVCalc;

	// Second square root term used in L/V calculation
	double secondSquareRootTermForLOverVCalc;

	// Force applied on track by leading truck
	double forceOnTrackDueToLeadingTruck;

	// Force applied on track by trailing truck
	double forceOnTrackDueToTrailingTruck;

	// Vertical force applied by right side of leading truck
	double verticalForceRightSideLeadingTruck;

	// Vertical force applied by left side of leading truck
	double verticalForceLeftSideLeadingTruck;

	// Vertical force applied by right side of trailing truck
	double verticalForceRightSideTrailingTruck;

	// Vertical force applied by left side of trailing truck
	double verticalForceLeftSideTrailingTruck;

	// Global gravitational force (newtons)
	double globalGravitationalForce;

	// Three-dimensional vector for global gravitational force
	double* globalVector_globalGravitationalForce;

	// Global reactive centrifugal force (newtons)
	double globalReactiveCentrifugalForce;

	// Three-dimensional vector for global reactive centrifugal force
	double* globalVector_globalReactiveCentrifugalForce;

};

#endif
