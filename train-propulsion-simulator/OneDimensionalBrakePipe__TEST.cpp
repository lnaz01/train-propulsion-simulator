//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <vector>
#include "OneDimensionalBrakePipe.h"
#include "Function.h"
#include "LocomotiveOperator.h"
#include "ResultsWriter.h"
#include "TrainConsist.h"
#include "UnitConverter.h"


class MyOneDimensionalBrakePipe : public OneDimensionalBrakePipe {
public:
	MyOneDimensionalBrakePipe(int numFE, double* h, double* d1, double* d2);
	double pressureBoundCondCalc(double t, double tStep);
	void leakArrCalc(double airTemp, double* pUP, bool pUPAct);
};


MyOneDimensionalBrakePipe::MyOneDimensionalBrakePipe(int numFE, double* h, double* d1, double* d2) : OneDimensionalBrakePipe(numFE, h, d1, d2) {}


double MyOneDimensionalBrakePipe::pressureBoundCondCalc(double t, double tStep) {
	if (endReservoirPressureDefinedBool == false) {
		endReservoirPressureDefinedBool = true;
		double tp;  // transition period
		tp = 12.5;
		/*
		const int xySize = 2;
		double x[xySize];
		double y[xySize];
		x[0] = 0.0;
		y[0] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[1] = 600.0;
		y[1] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		*/

		const int xySize = 4;
		double x[xySize];
		double y[xySize];
		x[0] = 0.0;
		y[0] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[1] = 5.0;
		y[1] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[2] = x[1] + tp;
		y[2] = UnitConverter::psi_To_Pa(80.0);
		x[3] = 600.0;
		y[3] = UnitConverter::psi_To_Pa(80.0);

		/*
		const int xySize = 24;
		double x[xySize];
		double y[xySize];
		x[0] = 0.0;
		y[0] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[1] = 300.0;
		y[1] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[2] = x[1] + tp;
		y[2] = UnitConverter::psi_To_Pa(80.0);
		x[3] = 600.0;
		y[3] = UnitConverter::psi_To_Pa(80.0);
		x[4] = x[3] + tp;
		y[4] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[5] = 900.0;
		y[5] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[6] = x[5] + tp;
		y[6] = UnitConverter::psi_To_Pa(80.0);
		x[7] = 1200.0;
		y[7] = UnitConverter::psi_To_Pa(80.0);
		x[8] = x[7] + tp;
		y[8] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[9] = 1500.0;
		y[9] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[10] = x[9] + tp;
		y[10] = UnitConverter::psi_To_Pa(80.0);
		x[11] = 1800.0;
		y[11] = UnitConverter::psi_To_Pa(80.0);
		x[12] = x[11] + tp;
		y[12] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[13] = 2100.0;
		y[13] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[14] = x[13] + tp;
		y[14] = UnitConverter::psi_To_Pa(80.0);
		x[15] = 2400.0;
		y[15] = UnitConverter::psi_To_Pa(80.0);
		x[16] = x[15] + tp;
		y[16] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[17] = 2700.0;
		y[17] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[18] = x[17] + tp;
		y[18] = UnitConverter::psi_To_Pa(80.0);
		x[19] = 3000.0;
		y[19] = UnitConverter::psi_To_Pa(80.0);
		x[20] = x[19] + tp;
		y[20] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[21] = 3300.0;
		y[21] = LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE;
		x[22] = x[21] + tp;
		y[22] = UnitConverter::psi_To_Pa(80.0);
		x[23] = 3600.0;
		y[23] = UnitConverter::psi_To_Pa(80.0);
		*/
		endReservoirPressure = new Function(false, 0);
		endReservoirPressure->piecewiseLinear(x, y, xySize);
	}
	return endReservoirPressure->interpolate(t + tStep, TrainConsist::ATMOSPHERIC_PRESSURE, LocomotiveOperator::BRAKE_PIPE_OPERATING_PRESSURE);
}


void MyOneDimensionalBrakePipe::leakArrCalc(double airTemp, double* pUP, bool pUPAct) {
	double OrificeArea = M_PI * pow(UnitConverter::in_To_M(1.0 / 4.0) / 2.0, 2.0);
	if (pUPAct == true) {
		for (int i = 0; i < numFE; i++) {
			leakArr[i] = 0.0;
		}
		for (int i = 25; i <= 25; i++) {
			double rEXP = pressure[i] / pUP[i];
			//double mdot = 0.6 * OrificeArea * pUP * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5) * (abs(1 - rEXP) / (1 - rEXP));
			double mdot = 0.6 * OrificeArea * pUP[i] * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5);
			if (pressure[i] > pUP[i]) {
				mdot = -mdot;
			}
			leakArr[i] = mdot;
		}
		for (int i = 149; i <= 149; i++) {
			double rEXP = pressure[i] / pUP[i];
			//double mdot = 0.6 * OrificeArea * pUP * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5) * (abs(1 - rEXP) / (1 - rEXP));
			double mdot = 0.6 * OrificeArea * pUP[i] * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5);
			if (pressure[i] > pUP[i]) {
				mdot = -mdot;
			}
			leakArr[i] = mdot;
		}
		/*
		for (int i = 0; i < locNodArrSize; i++) {
			double rEXP = pressure[i] / pUP[i];
			//double mdot = 0.6 * OrificeArea * pUP * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5) * (abs(1 - rEXP) / (1 - rEXP));
			double mdot = 0.6 * OrificeArea * pUP[i] * pow(abs(pow(rEXP, 2.0) - 1.0) / (TrainConsist::GAS_CONSTANT_AIR * airTemp), 0.5);
			if (pressure[i] > pUP[i]) {
				mdot = -mdot;
			}
			leakArr[i] = mdot;
		}
		*/
	}
	else {
		for (int i = 0; i < numFE; i++) {
			leakArr[i] = 0.0;
		}
	}
}


int main() {

	// Car length array
	const int NFE = 30;
	double carLength1 = UnitConverter::ft_To_M(66.0);
	double carLength2 = UnitConverter::ft_To_M(66.0);
	double* h = new double[NFE];
	h[0] = 33.0;
	for (int i = 1; i < NFE; i++) {
		if ((i % 2) == 0) {
			h[i] = carLength1;
		}
		else {
			h[i] = carLength2;
		}
	}

	// Brake pipe diameter
	double* d1 = new double[NFE];
	double* d2 = new double[NFE];
	double BPD = UnitConverter::in_To_M(1.25);
	for (int i = 0; i < NFE; i++) {
		d1[i] = BPD;
		d2[i] = BPD;
	}

	// Brake pipe operating pressure (Pascals)
	double BPOP = UnitConverter::psi_To_Pa(105.0);

	// MyOneDimensionalAirPipe
	MyOneDimensionalBrakePipe* my1DBP2 = new MyOneDimensionalBrakePipe(NFE, h, d1, d2);

	// Air temperature
	double AT = UnitConverter::f_To_K(100.0);

	// Initial conditions
	double* pressureInit = new double[NFE + 1];
	double* velocityInit = new double[NFE + 1];
	for (int i = 0; i < (NFE + 1); i++) {
		pressureInit[i] = BPOP;
		velocityInit[i] = 0.0;
	}

	// Upstream pressure array
	double* pUP = new double[NFE];
	for (int i = 0; i < NFE; i++) {
		pUP[i] = UnitConverter::psi_To_Pa(15.0);
	}

	// Simulate using finite elements and gaussian elimination algorithm
	std::string resultsFilePath_pressure = "C:/Users/Leith/Desktop/OneDimensionalAirPipe_pressure_TEST.csv";
	std::string resultsFilePath_velocity = "C:/Users/Leith/Desktop/OneDimensionalAirPipe_velocity_TEST.csv";
	ResultsWriter* resultsWriter_pressure = new ResultsWriter(resultsFilePath_pressure, false);
	ResultsWriter* resultsWriter_velocity = new ResultsWriter(resultsFilePath_velocity, false);
	double simTime = 180.0;  // seconds
	double timeStep = 0.02;  // seconds
	std::cout << "Running simulation..." << std::endl;
	my1DBP2->simulate(resultsWriter_pressure, resultsWriter_velocity, simTime, timeStep, AT, pressureInit, velocityInit, "right", pUP, false);

	// Flush and close output file stream in results writer for RK4 algorithm
	resultsWriter_pressure->ofs->flush();
	resultsWriter_pressure->ofs->close();
	resultsWriter_velocity->ofs->flush();
	resultsWriter_velocity->ofs->close();

	// Delete
	delete[] h;
	delete[] d1;
	delete[] d2;
	delete[] pressureInit;
	delete[] velocityInit;
	delete[] pUP;
	delete resultsWriter_pressure;
	delete resultsWriter_velocity;

	// End
	std::cout << "END" << std::endl;

}

