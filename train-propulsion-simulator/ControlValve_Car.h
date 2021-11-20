//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CONTROL_VALVE_CAR_DEF
#define CONTROL_VALVE_CAR_DEF

#include "ControlValve.h"
#include "UnitConverter.h"

class Car;

class ControlValve_Car : public ControlValve {

public:

	ControlValve_Car(Car* car);

	virtual ~ControlValve_Car();

	// Current operating mode
	// 0	-->	Lap mode
	// 1	-->	Service brake application mode
	// 2	-->	Brake release mode
	// 3	-->	Emergency brake application mode
	int currentOperatingMode;

	// Total air flow to auxiliary reservoir (kilograms/second)
	double mdot_ar;

	// Total air flow to emergency reservoir (kilograms/second)
	double mdot_er;

	// Total air flow to brake cylinder (kilograms / second)
	double mdot_bc;

	// Total air flow to brake pipe (kilograms / second)
	double mdot_bp;

	// Calculates current operating mode
	void calc_currentOperatingMode();

	// Calculates all air flows
	void calc_mdot();

private:

	// Car
	Car* car;

	// Maximum restriction area between brake pipe and auxiliary reservoir (meters^2)
	static const double AMAX_ar_bp;

	// Maximum restriction area between brake pipe and emergency reservoir (meters^2)
	static const double AMAX_er_bp;

	// Maximum restriction area between atmosphere and brake pipe (meters^2)
	static const double AMAX_bp_atm;

	// Maximum restriction area between auxiliary reservoir and brake cylinder (meters^2)
	// (A_i,3 in Abdol-Hamid)
	static const double AMAX_bc_ar;

	// Maximum restriction area between emergency reservoir and brake cylinder (meters^2)
	// (A_HPS in Abdol-Hamid)
	static const double AMAX_bc_er;

	// Maximum restriction area between atmosphere and brake cylinder (meters^2)
	// (A_BCV in Abdol-Hamid)
	static const double AMAX_bc_atm;

	// Lap mode initiation threshold for conventional and distributed power brake systems (pascals)
	const double LMIT = UnitConverter::psi_To_Pa(0.3);

	// Service brake application mode initiation threshold for conventional and distributed power brake systems (pascals)
	const double SBAMIT = UnitConverter::psi_To_Pa(0.8);

	// Service brake release mode initiation threshold for conventional and distributed power brake systems (pascals)
	const double SBRMIT = UnitConverter::psi_To_Pa(1.8);

	// Emergency brake application mode initiation threshold for conventional and distributed power brake systems (pascals)
	const double EBAMIT = UnitConverter::psi_To_Pa(2.8);

	// Air flow from brake pipe to auxiliary reservoir (kilograms / second)
	double mdot_ar_bp;

	// Air flow from brake pipe to emergency reservoir (kilograms / second)
	double mdot_er_bp;

	// Air flow from atmosphere to brake pipe (kilograms / second)
	double mdot_bp_atm;

	// Air flow from auxiliary reservoir to brake cylinder (kilograms / second)
	double mdot_bc_ar;

	// Air flow from emergency reservoir to brake cylinder (kilograms / second)
	double mdot_bc_er;

	// Air flow from atmosphere to brake cylinder (kilograms / second)
	double mdot_bc_atm;

	// Orifice open from brake pipe to auxiliary reservoir ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_ar_bp;

	// Orifice open from brake pipe to emergency reservoir ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_er_bp;

	// Orifice open from atmosphere to brake pipe ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_bp_atm;

	// Orifice open from auxiliary reservoir to brake cylinder ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_bc_ar;

	// Orifice open from emergency reservoir to brake cylinder ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_bc_er;

	// Orifice open from atmosphere to brake cylinder ('true' means orifice is open, and 'false' means orifice is closed)
	bool orificeBool_bc_atm;

	// Calculates total air flow to auxiliary reservoir
	void calc_mdot_ar();

	// Calculates total air flow to emergency reservoir
	void calc_mdot_er();

	// Calculates total air flow to brake cylinder
	void calc_mdot_bc();

	// Calculates total air flow to local brake pipe
	void calc_mdot_bp();

	// Calculates air flow from brake pipe to auxiliary reservoir
	void calc_mdot_ar_bp();

	// Calculates air flow from brake pipe to emergency reservoir
	void calc_mdot_er_bp();

	// Calculates air flow from atmosphere to brake pipe
	void calc_mdot_bp_atm();

	// Calculates air flow from auxiliary reservoir to brake cylinder
	void calc_mdot_bc_ar();

	// Calculates air flow from emergency reservoir to brake cylinder
	void calc_mdot_bc_er();

	// Calculates air flow from atmosphere to brake cylinder
	void calc_mdot_bc_atm();

};

#endif
