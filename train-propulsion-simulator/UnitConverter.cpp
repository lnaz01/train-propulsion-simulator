//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <cmath>
#include "UnitConverter.h"
#include "Track.h"


double UnitConverter::in_To_M(double in) {
	double m = in * UnitConverter::m_per_in;
	return m;
}


double UnitConverter::m_To_In(double m) {
	double in = m * UnitConverter::in_per_m;
	return in;
}


double UnitConverter::ft_To_M(double ft) {
	double m = ft * UnitConverter::m_per_ft;
	return m;
}


double UnitConverter::m_To_Ft(double m) {
	double ft = m * UnitConverter::ft_per_m;
	return ft;
}


double UnitConverter::ft2_To_M2(double ftsq) {
	double msq = ftsq * pow(UnitConverter::m_per_ft, 2.0);
	return msq;
}


double UnitConverter::m2_To_Ft2(double msq) {
	double ftsq = msq * pow(UnitConverter::ft_per_m, 2.0);
	return ftsq;
}


double UnitConverter::in3_To_M3(double incubed) {
	double mcubed = incubed * pow(UnitConverter::m_per_in, 3.0);
	return mcubed;
}


double UnitConverter::f_To_K(double f) {
	double k = ((f - 32.0) * (5.0 / 9.0)) + 273.15;
	return k;
}


double UnitConverter::lb_To_N(double lb) {
	double n = lb * UnitConverter::n_per_lb;
	return n;
}


double UnitConverter::n_To_Lb(double n) {
	double lb = n * UnitConverter::lb_per_n;
	return lb;
}


double UnitConverter::kip_To_N(double kip) {
	double n = (kip * 1000.0) * UnitConverter::n_per_lb;
	return n;
}


double UnitConverter::lb_To_Kg(double lb) {
	double kg = lb * UnitConverter::kg_per_lb;
	return kg;
}


double UnitConverter::kg_To_Lb(double kg) {
	double lb = kg * UnitConverter::lb_per_kg;
	return lb;
}


double UnitConverter::kip_To_Kg(double kips) {
	double kg = kips * (UnitConverter::kg_per_lb * 1000.0);
	return kg;
}


double UnitConverter::kg_To_Ton(double kg) {
	double tons = kg * UnitConverter::ton_per_kg;
	return tons;
}


double UnitConverter::miph_To_Mps(double mph) {
	double mps = mph * UnitConverter::m_per_mi * UnitConverter::hr_per_sec;
	return mps;
}


double UnitConverter::mps_To_Miph(double mps) {
	double mph = mps * UnitConverter::mi_per_m * UnitConverter::sec_per_hr;
	return mph;
}


double UnitConverter::psi_To_Pa(double psi) {
	double pa = psi * pow(UnitConverter::in_per_m, 2.0) *
		UnitConverter::n_per_lb;
	return pa;
}


double UnitConverter::pa_To_Psi(double pa) {
	double psi = pa * pow(UnitConverter::m_per_in, 2.0) *
		UnitConverter::lb_per_n;
	return psi;
}


double UnitConverter::psf_To_Pa(double psf) {
	double pa = psf * pow(UnitConverter::ft_per_m, 2.0) *
		UnitConverter::n_per_lb;
	return pa;
}


double UnitConverter::ppi_To_Npm(double ppi) {
	double npm = ppi * UnitConverter::n_per_lb * UnitConverter::in_per_m;
	return npm;
}


double UnitConverter::pct_To_Rad(double percent) {
	double radians = atan(percent / 100.0);
	return radians;
}


double UnitConverter::rad_To_Pct(double radians) {
	double percent = 100.0 * tan(radians);
	return percent;
}


double UnitConverter::inSup_To_Rad(double ios) {
	double radians = asin(ios / Track::TrackGaugeUS);
	return radians;
}


double UnitConverter::rad_To_InSup(double radians) {
	double ios = Track::TrackGaugeUS * sin(radians);
	return ios;
}


double UnitConverter::gCurv_To_TrkCurv(double kappag) {
	double CL = 50.0;  // chord length (feet)
	double kappar = 2.0 * (180.0 / M_PI) * asin(UnitConverter::ft_To_M(CL) * kappag);
	return kappar;
}


double UnitConverter::trkCurv_To_GCurv(double kappar) {
	double CL = 50.0;  // chord length (feet)
	double kappag = sin((kappar / 2.0) * (M_PI / 180.0)) / (ft_To_M(CL));
	return kappag;
}


double UnitConverter::pps_To_Kgps(double lbps) {
	double kgps = lbps * UnitConverter::kg_per_lb;
	return kgps;
}


double UnitConverter::kgps_To_Pps(double kgps) {
	double lbps = kgps * UnitConverter::lb_per_kg;
	return lbps;
}

