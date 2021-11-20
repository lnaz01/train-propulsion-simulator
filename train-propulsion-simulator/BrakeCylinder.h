//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRAKE_CYLINDER_DEF
#define BRAKE_CYLINDER_DEF

#include <cmath>
#include "UnitConverter.h"

class RailVehicle;

class BrakeCylinder {

public:

	BrakeCylinder(RailVehicle* railVehicle);

	virtual ~BrakeCylinder();

	// Piping volume between brake cylinder and control valve (meters^3)
	const double PIPING_VOLUME_BC_CV = UnitConverter::in3_To_M3(155.0);

	// Piping volume between auxiliary reservoir and control valve (meters^3)
	const double PIPING_VOLUME_AUXRES_CV = UnitConverter::in3_To_M3(155.0);

	// Piping volume between emergency reservoir and control valve (meters^3)
	const double PIPING_VOLUME_EMRES_CV = UnitConverter::in3_To_M3(260.0);

	// Brake piston displacement for initial wheel/shoe contact (meters)
	// (u_con in Afshari et al.)
	const double PISTON_DISP_WHEEL_CONTACT = UnitConverter::in_To_M(7.0);

	// Brake piston (cylinder) diameter (meters)
	const double PISTON_DIAMETER = UnitConverter::in_To_M(10.0);

	// Brake piston (cylinder) cross-sectional area (meters^2)
	const double PISTON_AREA = M_PI * pow((PISTON_DIAMETER / 2.0), 2.0);

	// Brake cylinder initial volume (meters^3)
	// (V_BI and/or V_BCI in Abdol-Hamid)
	const double INITIAL_VOLUME = UnitConverter::in3_To_M3(150.0);

	// Brake cylinder loaded volume (meters^3)
	const double LOADED_VOLUME = INITIAL_VOLUME + (PISTON_AREA * PISTON_DISP_WHEEL_CONTACT);

	// Brake cylinder spring stiffness (newtons / meter)
	// (K_BC in Abdol-Hamid) */
	const double SPRING_STIFFNESS = UnitConverter::ppi_To_Npm(10.0);

	// Brake cylinder spring pre-load (newtons)
	// (L_BC in Abdol-Hamid)
	const double SPRING_PRE_LOAD = UnitConverter::lb_To_N(200.0);

	// Friction force (newtons)
	const double FRICTION_FORCE = 0.0;

	// Rail vehicle
	RailVehicle* railVehicle;

	// Rail vehicle type ('0' for car; '1' for locomotive)
	int railVehicleType;

	// Pressure (pascals)
	double pressure;

	// Retarding brake force (newtons)
	double retardingBrakeForce;

	// Maximum brake cylinder pressure for full service braking (pascals)
	double maxPressureForFullServiceBraking;

	// Maximum brake cylinder pressure for emergency braking (pascals)
	double maxPressureForEmergencyBraking;

	// Calculates retarding force due to contact between brake shoes and wheels
	void calc_retardingBrakeForce();

	// Calculates normal force between brake shoes and wheels
	void calc_normalShoeWheelForce();

	// Calculates piston force
	void calc_pistonForce();

	// Calculates pressure (should be called after car is loaded and conversion to SI is performed)
	// initialPressureCalculationBool	-->	'true' if brake cylinder pressure is being calculated for the first time (at beginning of simulation)
	void calc_pressure(bool initialPressureCalculationBool);

private:

	// Piston force (newtons)
	double pistonForce;

	// Normal force between brake shoes and wheels
	double normalShoeWheelForce;

	// Calculates maximum brake cylinder pressure for full service braking
	void calc_maxPressureForFullServiceBraking();

	// Calculates maximum brake cylinder pressure for emergency braking
	void calc_maxPressureForEmergencyBraking();

};

#endif
