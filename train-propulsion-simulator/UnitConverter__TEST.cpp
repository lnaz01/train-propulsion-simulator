//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "UnitConverter.h"


void in_To_M__test(double in);
void m_To_In__test(double m);
void ft_To_M__test(double ft);
void m_To_Ft__test(double m);
void ft2_To_M2__test(double ftsq);
void m2_To_Ft2__test(double msq);
void in3_To_M3__test(double incubed);
void f_To_K__test(double f);
void lb_To_N__test(double lb);
void n_To_Lb__test(double n);
void lb_To_Kg__test(double lb);
void kg_To_Lb__test(double kg);
void kg_To_Ton__test(double kg);
void miph_To_Mps__test(double mph);
void mps_To_Miph__test(double mps);
void psi_To_Pa__test(double psi);
void pa_To_Psi__test(double pa);
void psf_To_Pa__test(double psf);
void ppi_To_Npm__test(double ppi);
void pct_To_Rad__test(double perc);
void rad_To_Pct__test(double rad);
void inSup_To_Rad__test(double ise);
void rad_To_InSup__test(double rad);
void gCurv_To_TrkCurv__test(double gc);
void trkCurv_To_GCurv__test(double tc);


int main() {

	// Inches to meters
	in_To_M__test(9.0);

	// Meters to inches
	m_To_In__test(7.5);

	// Feet to meters
	ft_To_M__test(0.8);

	// Meters to feet
	m_To_Ft__test(1.2);

	// Feet^2 to meters^2
	ft2_To_M2__test(0.9);

	// Meters^2 to feet^2
	m2_To_Ft2__test(1.1);

	// Inches^3 to meters^3
	in3_To_M3__test(750000.0);

	// Fahrenheit to kelvin
	f_To_K__test(80.0);

	// Pounds to newtons
	lb_To_N__test(185.0);

	// Newtons to pounds
	n_To_Lb__test(505.5);

	// Pounds to kilograms
	lb_To_Kg__test(165.0);

	// Kilograms to pounds
	kg_To_Lb__test(20.5);

	// Kilograms to tons
	kg_To_Ton__test(555.0);

	// Miles/hour to meters/second
	miph_To_Mps__test(60.0);

	// Meters/second to miles/hour
	mps_To_Miph__test(15.0);

	// PSI to pascals
	psi_To_Pa__test(77.5);

	// Pascals to PSI
	pa_To_Psi__test(109.0);

	// PSF to pascals
	psf_To_Pa__test(3.86);

	// "Pounds per inch" to "newtons per meter"
	ppi_To_Npm__test(50000.0);

	// Percent to radians
	pct_To_Rad__test(80.9784);

	// Radians to percent
	rad_To_Pct__test(39.0 * (M_PI / 180.0));

	// Inches of superelevation to radians
	inSup_To_Rad__test(1.5);

	// Radians to inches of superelevation
	rad_To_InSup__test(0.314);

	// Geometric curvature to track curvature
	gCurv_To_TrkCurv__test(0.0502654);

	// Track curvature to geometric curvature
	trkCurv_To_GCurv__test(100.0);

}


void in_To_M__test(double in) {
	std::cout << "Inches to Meters" << std::endl;
	std::cout << in << " inches = " << UnitConverter::in_To_M(in) << " meters" << std::endl;
	std::cout << std::endl;
}


void m_To_In__test(double m) {
	std::cout << "Meters to Inches" << std::endl;
	std::cout << m << " meters = " << UnitConverter::m_To_In(m) << " inches" << std::endl;
	std::cout << std::endl;
}


void ft_To_M__test(double ft) {
	std::cout << "Feet to Meters" << std::endl;
	std::cout << ft << " feet = " << UnitConverter::ft_To_M(ft) << " meters" << std::endl;
	std::cout << std::endl;
}


void m_To_Ft__test(double m) {
	std::cout << "Meters to Feet" << std::endl;
	std::cout << m << " meters = " << UnitConverter::m_To_Ft(m) << " feet" << std::endl;
	std::cout << std::endl;
}


void ft2_To_M2__test(double ftsq) {
	std::cout << "Feet Squared to Meters Squared" << std::endl;
	std::cout << ftsq << " feet^2 = " << UnitConverter::ft2_To_M2(ftsq) << " meters^2" << std::endl;
	std::cout << std::endl;
}


void m2_To_Ft2__test(double msq) {
	std::cout << "Meters Squared to Feet Squared" << std::endl;
	std::cout << msq << " meters^2 = " << UnitConverter::m2_To_Ft2(msq) << " feet^2" << std::endl;
	std::cout << std::endl;
}


void in3_To_M3__test(double incubed) {
	std::cout << "Inches Cubed to Meters Cubed" << std::endl;
	std::cout << incubed << " inches^3 = " << UnitConverter::in3_To_M3(incubed) << " meters^3" << std::endl;
	std::cout << std::endl;
}


void f_To_K__test(double f) {
	std::cout << "Fahrenheit to Kelvin" << std::endl;
	std::cout << f << " fahrenheit = " << UnitConverter::f_To_K(f) << " kelvin" << std::endl;
	std::cout << std::endl;
}


void lb_To_N__test(double lb) {
	std::cout << "Pounds to Newtons" << std::endl;
	std::cout << lb << " pounds = " << UnitConverter::lb_To_N(lb) << " newtons" << std::endl;
	std::cout << std::endl;
}


void n_To_Lb__test(double n) {
	std::cout << "Newtons to Pounds" << std::endl;
	std::cout << n << " newtons = " << UnitConverter::n_To_Lb(n) << " pounds" << std::endl;
	std::cout << std::endl;
}


void lb_To_Kg__test(double lb) {
	std::cout << "Pounds to Kilograms" << std::endl;
	std::cout << lb << " pounds = " << UnitConverter::lb_To_Kg(lb) << " kilograms" << std::endl;
	std::cout << std::endl;
}


void kg_To_Lb__test(double kg) {
	std::cout << "Kilograms to Pounds" << std::endl;
	std::cout << kg << " kilograms = " << UnitConverter::kg_To_Lb(kg) << " pounds" << std::endl;
	std::cout << std::endl;
}


void kg_To_Ton__test(double kg) {
	std::cout << "Kilograms to Tons" << std::endl;
	std::cout << kg << " kilograms = " << UnitConverter::kg_To_Ton(kg) << " tons" << std::endl;
	std::cout << std::endl;
}


void miph_To_Mps__test(double mph) {
	std::cout << "'Miles per Hour' to 'Meters per Second'" << std::endl;
	std::cout << mph << " miles per hour = " << UnitConverter::miph_To_Mps(mph) << " meters per second" << std::endl;
	std::cout << std::endl;
}


void mps_To_Miph__test(double mps) {
	std::cout << "'Meters per Second' to 'Miles per Hour'" << std::endl;
	std::cout << mps << " meters per second = " << UnitConverter::mps_To_Miph(mps) << " miles per hour" << std::endl;
	std::cout << std::endl;
}


void psi_To_Pa__test(double psi) {
	std::cout << "PSI to Pascals" << std::endl;
	std::cout << psi << " psi = " << UnitConverter::psi_To_Pa(psi) << " pascals" << std::endl;
	std::cout << std::endl;
}


void pa_To_Psi__test(double pa) {
	std::cout << "Pascals to PSI" << std::endl;
	std::cout << pa << " pascals = " << UnitConverter::pa_To_Psi(pa) << " psi" << std::endl;
	std::cout << std::endl;
}


void psf_To_Pa__test(double psf) {
	std::cout << "PSF to Pascals" << std::endl;
	std::cout << psf << " psf = " << UnitConverter::psf_To_Pa(psf) << " pascals" << std::endl;
	std::cout << std::endl;
}


void ppi_To_Npm__test(double ppi) {
	std::cout << "'Pounds per Inch' to 'Newtons per Meter'" << std::endl;
	std::cout << ppi << " pounds per inch = " << UnitConverter::ppi_To_Npm(ppi) << " newtons per meter" << std::endl;
	std::cout << std::endl;
}


void pct_To_Rad__test(double perc) {
	std::cout << "Percent to Radians" << std::endl;
	std::cout << perc << " percent = " << UnitConverter::pct_To_Rad(perc) << " radians" << std::endl;
	std::cout << std::endl;
}


void rad_To_Pct__test(double rad) {
	std::cout << "Radians to Percent" << std::endl;
	std::cout << rad << " radians = " << UnitConverter::rad_To_Pct(rad) << " percent" << std::endl;
	std::cout << std::endl;
}


void inSup_To_Rad__test(double ise) {
	std::cout << "'Inches of Superelevation' to Radians" << std::endl;
	std::cout << ise << " inches of superelevation = " << UnitConverter::inSup_To_Rad(ise) << " radians" << std::endl;
	std::cout << std::endl;
}


void rad_To_InSup__test(double rad) {
	std::cout << "Radians to 'Inches of Superelevation'" << std::endl;
	std::cout << rad << " radians = " << UnitConverter::rad_To_InSup(rad) << " inches of superelevation" << std::endl;
	std::cout << std::endl;
}


void gCurv_To_TrkCurv__test(double gc) {
	std::cout << "Geometric Curvature to Track Curvature" << std::endl;
	std::cout << gc << " geometric curvature = " << UnitConverter::gCurv_To_TrkCurv(gc) << " track curvature" << std::endl;
	std::cout << std::endl;
}


void trkCurv_To_GCurv__test(double tc) {
	std::cout << "Track Curvature to Geometric Curvature" << std::endl;
	std::cout << tc << " track curvature = " << UnitConverter::trkCurv_To_GCurv(tc) << " geometric curvature" << std::endl;
	std::cout << std::endl;
}

