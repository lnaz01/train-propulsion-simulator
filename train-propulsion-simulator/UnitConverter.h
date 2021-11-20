//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UNIT_CONVERTER_DEF
#define UNIT_CONVERTER_DEF

#define M_PI 3.14159265358979323846

class UnitConverter {

public:

	// DISTANCE

	// Converts inches to meters
	static double in_To_M(double in);

	// Converts meters to inches
	static double m_To_In(double m);

	// Converts feet to meters
	static double ft_To_M(double ft);

	// Converts meters to feet
	static double m_To_Ft(double m);

	// AREA

	// Converts feet^2 to meters^2
	static double ft2_To_M2(double ftsq);

	// Converts meters^2 to feet^2
	static double m2_To_Ft2(double msq);

	// VOLUME

	// Converts inches^3 to meters^3
	static double in3_To_M3(double incubed);

	// TEMPERATURE

	// Converts fahrenheit to kelvin
	static double f_To_K(double f);

	// FORCE AND MASS

	// Converts pounds to newtons
	static double lb_To_N(double lb);

	// Converts newtons to pounds
	static double n_To_Lb(double n);

	// Converts kips to newtons
	static double kip_To_N(double kip);

	// Converts pounds to kilograms
	static double lb_To_Kg(double lb);

	// Converts kilograms to pounds
	static double kg_To_Lb(double kg);

	// Converts kips to kilograms
	static double kip_To_Kg(double kips);

	// Converts kilograms to tons
	static double kg_To_Ton(double kg);

	// SPEED

	// Converts miles/hour to meters/second
	static double miph_To_Mps(double mph);

	// Converts meters/second to miles/hour
	static double mps_To_Miph(double mps);

	// PRESSURE

	// Converts psi to pascals
	static double psi_To_Pa(double psi);

	// Converts pascals to psi
	static double pa_To_Psi(double pa);

	// Converts psf (pounds per square foot) to pascals
	static double psf_To_Pa(double psf);

	// SPRING STIFFNESS COEFFICIENT

	// Converts pounds/inch to newtons/meter
	static double ppi_To_Npm(double ppi);

	// ANGLE

	// Converts percent to radians
	static double pct_To_Rad(double percent);

	// Converts radians to percent
	static double rad_To_Pct(double radians);

	// Converts inches of superelevation to radians
	static double inSup_To_Rad(double ios);

	// Converts radians to inches of superelevation
	static double rad_To_InSup(double radians);

	// CURVATURE

	// Converts geometric curvature (m^-1) to track curvature (degrees)
	static double gCurv_To_TrkCurv(double kappag);

	// Converts track curvature (degrees) to geometric curvature (m^-1)
	static double trkCurv_To_GCurv(double kappar);

	// DAMPING

	// Definition of function that converts pounds/second to kilograms/second
	static double pps_To_Kgps(double lbps);

	// Definition of function that converts kilograms/second to pounds/second
	static double kgps_To_Pps(double kgps);

private:

	// Inches per meter
	static constexpr double in_per_m = 39.36996;

	// Meters per inch
	static constexpr double m_per_in = 0.0254;

	// Feet per meter
	static constexpr double ft_per_m = 3.28083;

	// Meters per foot
	static constexpr double m_per_ft = 0.3048;

	// Miles per meter
	static constexpr double mi_per_m = 0.0006213712;

	// Meters per mile
	static constexpr double m_per_mi = 1609.344;

	// Pounds per newton
	static constexpr double lb_per_n = 0.2248089;

	// Newtons per pound
	static constexpr double n_per_lb = 4.448222;

	// Pounds per kilogram
	static constexpr double lb_per_kg = 2.20462;

	// Kilograms per pound
	static constexpr double kg_per_lb = 0.4535924;

	// Tons per kilogram
	static constexpr double ton_per_kg = 0.0011023;

	// Seconds per hour
	static constexpr double sec_per_hr = 3600.0;

	// Hours per second
	static constexpr double hr_per_sec = 0.0002777778;

};

#endif
