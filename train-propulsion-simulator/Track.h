//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRACK_DEF
#define TRACK_DEF

#include <vector>
#include "UnitConverter.h"
#include "UserDefinedRRComponent.h"

class InputFileReader_Simulation;

class Track : public UserDefinedRRComponent {

public:

	Track(InputFileReader_Simulation* inputFileReader_Simulation);

	virtual ~Track();

	// Maximum longitudinal position for first point in US units (feet)
	const double MAXD0_US[3] = { 0.0, 0.0, 0.0 };

	// Track gauge (inches)
	static constexpr double TrackGaugeUS = 56.5;

	// Track gage (meters)
	static const double TrackGaugeSI;

	// Track length (meters)
	double trackLength;

	// Calculates track length
	void calc_trackLength();

	// Track angle
	// longPosition		-->	Longitudinal position (meters)
	double angle(double longPosition);

	void convertToSI() override;

protected:

	void loadPhysicalConstantAlternateNames() override;

private:

	// Minimum track length in US units (feet)
	static constexpr double MINTL = 52800.0;  // 52,800 feet = 10 miles

	// Maximum track length in US units (feet)
	static constexpr double MAXTL = 1056000.0;  // 1,056,000 feet = 200 miles

	// Coefficient of second order term for slope equation
	std::vector<double> coef2;

	// Coefficient of first order term for slope equation
	std::vector<double> coef1;

	// Coefficient of zero order term for slope equation
	std::vector<double> coef0;

	// Coefficients of slope equation calculated boolean
	bool slopeEquationCoefficientsCalculatedBool;

	// Calculates coefficients of piecewise-smooth slope equation
	void slopeEquationCoefficients();

};

#endif
