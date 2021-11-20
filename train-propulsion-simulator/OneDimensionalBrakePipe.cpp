//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "OneDimensionalBrakePipe.h"
#include "LinearSystem.h"
#include "ResultsWriter.h"
#include "UnitConverter.h"


OneDimensionalBrakePipe::OneDimensionalBrakePipe(int numFE, double* h, double* d1, double* d2) {
	this->numFE = numFE;
	this->h = h;
	this->d1 = d1;
	this->d2 = d2;
	locNodArrSize = numFE + 1;
	// Left-end reservoir pressure function defined boolean
	endReservoirPressureDefinedBool = false;
	// Location of nodes array
	locNodArr = new double[locNodArrSize];
	double cumDistance = 0.0;
	locNodArr[0] = cumDistance;
	for (int i = 0; i < numFE; i++) {
		cumDistance = cumDistance + h[i];
		locNodArr[i + 1] = cumDistance;
	}
	// Reynolds number array
	reynoldsNumArr = new double[locNodArrSize];
	// Wall friction factor array
	wallFricFactorArr = new double[locNodArrSize];
	// Leakage array
	leakArr = new double[numFE];
	// Pressure array
	pressure = new double[locNodArrSize];
	// Velocity array
	velocity = new double[locNodArrSize];
	// Linear system
	linearSystem = new LinearSystem();
}


OneDimensionalBrakePipe::~OneDimensionalBrakePipe() {
	// Location of nodes array
	delete[] locNodArr;
	// Reynolds number array
	delete[] reynoldsNumArr;
	// Wall friction factor array
	delete[] wallFricFactorArr;
	// Leakage array
	delete[] leakArr;
	// Pressure array
	delete[] pressure;
	// Velocity array
	delete[] velocity;
	// Linear system
	delete linearSystem;
	// Left-end reservoir pressure function
	if (endReservoirPressureDefinedBool == true) {
		delete endReservoirPressure;
	}
}


void OneDimensionalBrakePipe::simulate(ResultsWriter* resWriter_pressure, ResultsWriter* resWriter_velocity, double timeSim, double timeStep, double airTemp, double* pressureInit, double* velocityInit, std::string leftRightPressureBoundCond, double* pUP, bool pUPAct) {
	double t = 0;  // current simulation time
	// Density array
	double* density = new double[locNodArrSize];
	// 'm' array
	double* m = new double[locNodArrSize];
	for (int i = 0; i < locNodArrSize; i++) {
		pressure[i] = pressureInit[i];
		velocity[i] = velocityInit[i];
		density[i] = pressure[i] / (Rg * airTemp);
		m[i] = density[i] * velocity[i];
	}
	// Brake pipe diameter harmonic mean array
	double* dh = new double[numFE];
	for (int i = 0; i < numFE; i++) {
		dh[i] = (2.0 * d1[i] * d2[i]) / (d1[i] + d2[i]);
	}
	// Brake pipe area arrays
	double* a1 = new double[numFE];
	double* a2 = new double[numFE];
	double* ah = new double[numFE];
	for (int i = 0; i < numFE; i++) {
		a1[i] = M_PI * pow(d1[i] / 2.0, 2.0);
		a2[i] = M_PI * pow(d2[i] / 2.0, 2.0);
		ah[i] = (2.0 * a1[i] * a2[i]) / (a1[i] + a2[i]);
	}
	// Calculate air dynamic viscosity
	double airDynVisc = airDynViscCalc(airTemp);
	// Allocate space for global system matrix
	int systemMatrixDim = 2 * (numFE + 1);
	double** systemMatrix = new double* [systemMatrixDim];
	for (int i = 0; i < systemMatrixDim; i++) {
		systemMatrix[i] = new double[systemMatrixDim];
	}
	// Allocate space for global forcing vector
	double* forcingVector = new double[systemMatrixDim];
	// Allocate space for u1 and u2 arrays
	double* u1 = new double[numFE];
	double* u2 = new double[numFE];
	// Allocate space for c1 and c2 arrays
	double* c1 = new double[numFE];
	double* c2 = new double[numFE];
	double* ceff = new double[numFE];
	// Do-while loop for progressing through time steps
	do {
		// Calculate wall friction factor
		wallFricFactorArrCalc(airTemp, airDynVisc);
		// Calculate leakage
		leakArrCalc(airTemp, pUP, pUPAct);
		// u1 and u2 arrays
		for (int i = 0; i < numFE; i++) {
			u1[i] = velocity[i];
			u2[i] = velocity[i + 1];
		}
		// c1 and c2 arrays
		for (int i = 0; i < numFE; i++) {
			if (u1[i] == 0.0) {
				c1[i] = 0.0;
			}
			else {
				c1[i] = wallFricFactorArr[i] * ((density[i] * pow(u1[i], 2.0)) / 8.0) * (u1[i] / std::abs(u1[i])) * M_PI * dh[i];
			}
			if (u2[i] == 0.0) {
				c2[i] = 0.0;
			}
			else {
				c2[i] = wallFricFactorArr[i + 1] * ((density[i + 1] * pow(u2[i], 2.0)) / 8.0) * (u2[i] / std::abs(u2[i])) * M_PI * dh[i];
			}
			ceff[i] = (c1[i] + c2[i]) / 2.0;
		}
		// Set up system matrix
		for (int i = 0; i < systemMatrixDim; i++) {
			for (int j = 0; j < systemMatrixDim; j++) {
				systemMatrix[i][j] = 0.0;
			}
		}
		for (int i = 0; i < systemMatrixDim; i++) {
			if (i == 0) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					systemMatrix[i][0] = 1.0;
				}
				else {
					systemMatrix[i][0] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[0] * h[0]) / 3.0);
					systemMatrix[i][1] = -(a1[0] / 2.0);
					systemMatrix[i][2] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[0] * h[0]) / 6.0);
					systemMatrix[i][3] = a2[0] / 2.0;
				}
			}
			else if (i == 1) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					systemMatrix[i][0] = -(ah[0] / 2.0);
					systemMatrix[i][1] = ((1.0 / timeStep) * ((ah[0] * h[0]) / 3.0)) - ((u1[0] * a1[0]) / 2.0);
					systemMatrix[i][2] = ah[0] / 2.0;
					systemMatrix[i][3] = ((1.0 / timeStep) * ((ah[0] * h[0]) / 6.0)) + ((u2[0] * a2[0]) / 2.0);
				}
				else {
					systemMatrix[i][1] = 1.0;
				}
			}
			else if (i == systemMatrixDim - 2) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					systemMatrix[i][systemMatrixDim - 4] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[numFE - 1] * h[numFE - 1]) / 6.0);
					systemMatrix[i][systemMatrixDim - 3] = -(a1[numFE - 1] / 2.0);
					systemMatrix[i][systemMatrixDim - 2] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[numFE - 1] * h[numFE - 1]) / 3.0);
					systemMatrix[i][systemMatrixDim - 1] = a2[numFE - 1] / 2.0;
				}
				else {
					systemMatrix[i][systemMatrixDim - 2] = 1.0;
				}
			}
			else if (i == systemMatrixDim - 1) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					systemMatrix[i][systemMatrixDim - 1] = 1.0;
				}
				else {
					systemMatrix[i][systemMatrixDim - 4] = -(ah[numFE - 1] / 2.0);
					systemMatrix[i][systemMatrixDim - 3] = ((1.0 / timeStep) * ((ah[numFE - 1] * h[numFE - 1]) / 6.0)) - ((u1[numFE - 1] * a1[numFE - 1]) / 2.0);
					systemMatrix[i][systemMatrixDim - 2] = ah[numFE - 1] / 2.0;
					systemMatrix[i][systemMatrixDim - 1] = ((1.0 / timeStep) * ((ah[numFE - 1] * h[numFE - 1]) / 3.0)) + ((u2[numFE - 1] * a2[numFE - 1]) / 2.0);
				}
			}
			else {
				int currFE1 = (int)floor(i / 2) - 1;
				int currFE2 = (int)floor(i / 2);
				if (i % 2 == 0) {
					systemMatrix[i][i - 2] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE1] * h[currFE1]) / 6.0);
					systemMatrix[i][i - 1] = -(a1[currFE1] / 2.0);
					systemMatrix[i][i] = ((1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE1] * h[currFE1]) / 3.0)) + ((1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE2] * h[currFE2]) / 3.0));
					systemMatrix[i][i + 1] = (a2[currFE1] / 2.0) - (a1[currFE2] / 2.0);
					systemMatrix[i][i + 2] = (1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE2] * h[currFE2]) / 6.0);
					systemMatrix[i][i + 3] = a2[currFE2] / 2.0;
				}
				else {
					systemMatrix[i][i - 3] = -(ah[currFE1] / 2.0);
					systemMatrix[i][i - 2] = ((1.0 / timeStep) * ((ah[currFE1] * h[currFE1]) / 6.0)) - ((u1[currFE1] * a1[currFE1]) / 2.0);
					systemMatrix[i][i - 1] = (ah[currFE1] / 2.0) - (ah[currFE2] / 2.0);
					systemMatrix[i][i] = ((1.0 / timeStep) * ((ah[currFE1] * h[currFE1]) / 3.0)) + ((1.0 / timeStep) * ((ah[currFE2] * h[currFE2]) / 3.0)) + ((u2[currFE1] * a2[currFE1]) / 2.0) - ((u1[currFE2] * a1[currFE2]) / 2.0);
					systemMatrix[i][i + 1] = ah[currFE2] / 2.0;
					systemMatrix[i][i + 2] = ((1.0 / timeStep) * ((ah[currFE2] * h[currFE2]) / 6.0)) + ((u2[currFE2] * a2[currFE2]) / 2.0);
				}
			}
		}
		// Set up forcing vector
		for (int i = 0; i < systemMatrixDim; i++) {
			if (i == 0) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					forcingVector[i] = pressureBoundCondCalc(t, timeStep);
				}
				else {
					double term1 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[0] * h[0]) / 3.0) * pressure[0];
					double term2 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[0] * h[0]) / 6.0) * pressure[1];
					double term3 = leakArr[0] / 2.0;
					forcingVector[i] = term1 + term2 + term3;
				}
			}
			else if (i == 1) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					double term1 = (1.0 / timeStep) * ((ah[0] * h[0]) / 3.0) * m[0];
					double term2 = (1.0 / timeStep) * ((ah[0] * h[0]) / 6.0) * m[1];
					double term3 = (ceff[0] * h[0]) / 2.0;
					forcingVector[i] = term1 + term2 - term3;
				}
				else {
					forcingVector[i] = 0.0;
				}
			}
			else if (i == systemMatrixDim - 2) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					double term1 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[numFE - 1] * h[numFE - 1]) / 6.0) * pressure[locNodArrSize - 2];
					double term2 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[numFE - 1] * h[numFE - 1]) / 3.0) * pressure[locNodArrSize - 1];
					double term3 = leakArr[numFE - 1] / 2.0;
					forcingVector[i] = term1 + term2 + term3;
				}
				else {
					forcingVector[i] = pressureBoundCondCalc(t, timeStep);
				}
			}
			else if (i == systemMatrixDim - 1) {
				if (leftRightPressureBoundCond.compare("left") == 0) {
					forcingVector[i] = 0.0;
				}
				else {
					double term1 = (1.0 / timeStep) * ((ah[numFE - 1] * h[numFE - 1]) / 6.0) * m[locNodArrSize - 2];
					double term2 = (1.0 / timeStep) * ((ah[numFE - 1] * h[numFE - 1]) / 3.0) * m[locNodArrSize - 1];
					double term3 = (ceff[numFE - 1] * h[numFE - 1]) / 2.0;
					forcingVector[i] = term1 + term2 - term3;
				}
			}
			else {
				int currFE1 = (int)floor(i / 2) - 1;
				int currFE2 = (int)floor(i / 2);
				for (int j = 0; j < systemMatrixDim; j++) {
					if (i % 2 == 0) {
						double term1 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE1] * h[currFE1]) / 6.0) * pressure[currFE1];
						double term2 = (((1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE1] * h[currFE1]) / 3.0)) + ((1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE2] * h[currFE2]) / 3.0))) * pressure[currFE2];
						double term3 = (1.0 / (Rg * airTemp * timeStep)) * ((ah[currFE2] * h[currFE2]) / 6.0) * pressure[currFE2 + 1];
						double term4 = leakArr[currFE1] / 2.0;
						double term5 = leakArr[currFE2] / 2.0;
						forcingVector[i] = term1 + term2 + term3 + term4 + term5;
					}
					else {
						double term1 = (1.0 / timeStep) * ((ah[currFE1] * h[currFE1]) / 6.0) * m[currFE1];
						double term2 = (((1.0 / timeStep) * ((ah[currFE1] * h[currFE1]) / 3.0)) + ((1.0 / timeStep) * ((ah[currFE2] * h[currFE2]) / 3.0))) * m[currFE2];
						double term3 = (1.0 / timeStep) * ((ah[currFE2] * h[currFE2]) / 6.0) * m[currFE2 + 1];
						double term4 = (ceff[currFE1] * h[currFE1]) / 2.0;
						double term5 = (ceff[currFE2] * h[currFE2]) / 2.0;
						forcingVector[i] = term1 + term2 + term3 - term4 - term5;
					}
				}
			}
		}
		// Solve using gaussian elimination
		linearSystem->gaussElimination(systemMatrix, forcingVector, systemMatrixDim);
		// Update pressure and velocity values
		for (int i = 0; i < systemMatrixDim; i++) {
			if (i % 2 == 0) {
				pressure[(int)floor(i / 2.0)] = linearSystem->stateSpaceVector[i];
			}
			else {
				m[(int)floor(i / 2.0)] = linearSystem->stateSpaceVector[i];
			}
		}
		// Calculate density
		for (int i = 0; i < locNodArrSize; i++) {
			density[i] = pressure[i] / (Rg * airTemp);
		}
		// Calculate velocity
		for (int i = 0; i < locNodArrSize; i++) {
			velocity[i] = m[i] / density[i];
		}
		// Update time
		t = t + timeStep;
		// Print results
		std::vector<double> resLine_pressure;
		std::vector<double> resLine_velocity;
		resLine_pressure.push_back(t);
		resLine_velocity.push_back(t);
		for (int i = 0; i < locNodArrSize; i++) {
			resLine_pressure.push_back(UnitConverter::pa_To_Psi(pressure[i]));
			resLine_velocity.push_back(UnitConverter::mps_To_Miph(velocity[i]));
		}
		resWriter_pressure->writeLine(resLine_pressure);
		resWriter_velocity->writeLine(resLine_velocity);
	} while (t < timeSim);
	delete[] density;
	delete[] m;
	delete[] dh;
	delete[] a1;
	delete[] a2;
	delete[] ah;
	delete[] u1;
	delete[] u2;
	delete[] c1;
	delete[] c2;
	delete[] ceff;
	for (int i = 0; i < systemMatrixDim; i++) {
		delete[] systemMatrix[i];
	}
	delete[] systemMatrix;
	delete[] forcingVector;
}


void OneDimensionalBrakePipe::wallFricFactorArrCalc(double airTemp, double airDynVisc) {
	reynoldsNumArrCalc(airTemp, airDynVisc);
	double a = 0.0;
	double b = 0.0;
	for (int i = 0; i < locNodArrSize; i++) {
		if ((reynoldsNumArr[i] >= 0.0) && (reynoldsNumArr[i] <= 2000.0)) {
			a = 64.00;
			b = -1.00;
		}
		else if ((reynoldsNumArr[i] > 2000.0) && (reynoldsNumArr[i] <= 4000.0)) {
			a = 0.000137;
			b = 0.717;
		}
		else if ((reynoldsNumArr[i] > 4000.0) && (reynoldsNumArr[i] <= 40000.0)) {
			a = 0.13977;
			b = -0.11781;
		}
		else {
			a = 0.04;
			b = 0.0;
		}
		wallFricFactorArr[i] = a * pow(reynoldsNumArr[i], b);
	}
}


void OneDimensionalBrakePipe::reynoldsNumArrCalc(double airTemp, double airDynVisc) {
	for (int i = 0; i < locNodArrSize; i++) {
		double rho = pressure[i] / (Rg * airTemp);
		double deff;
		if (i < locNodArrSize - 1) {
			deff = d1[i];
		}
		else {
			deff = d2[i - 1];
		}
		reynoldsNumArr[i] = (rho * std::abs(velocity[i]) * deff) / airDynVisc;
		if (reynoldsNumArr[i] < 0.1) {
			reynoldsNumArr[i] = 0.1;
		}
	}
}


double OneDimensionalBrakePipe::airDynViscCalc(double airTemp) {
	double airDynVisc = 0.0;
	// AirTemp < (-40 F)
	if (airTemp < UnitConverter::f_To_K(-40.0)) {
		airDynVisc = UnitConverter::psf_To_Pa(3.29 * pow(10.0, -7.0));
	}
	// (-40 F) <= AirTemp < (-20 F)
	else if ((airTemp >= UnitConverter::f_To_K(-40.0)) && (airTemp < UnitConverter::f_To_K(-20.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.29 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.34 * pow(10.0, -7.0))) / 2.0;
	}
	// (-20 F) <= AirTemp < (0 F)
	else if ((airTemp >= UnitConverter::f_To_K(-20.0)) && (airTemp < UnitConverter::f_To_K(0.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.34 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.38 * pow(10.0, -7.0))) / 2.0;
	}
	// (0 F) <= AirTemp < (10 F)
	else if ((airTemp >= UnitConverter::f_To_K(0.0)) && (airTemp < UnitConverter::f_To_K(10.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.38 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.44 * pow(10.0, -7.0))) / 2.0;
	}
	// (10 F) <= AirTemp < (20 F)
	else if ((airTemp >= UnitConverter::f_To_K(10.0)) && (airTemp < UnitConverter::f_To_K(20.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.44 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.50 * pow(10.0, -7.0))) / 2.0;
	}
	// (20 F) <= AirTemp < (30 F)
	else if ((airTemp >= UnitConverter::f_To_K(20.0)) && (airTemp < UnitConverter::f_To_K(30.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.50 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.58 * pow(10.0, -7.0))) / 2.0;
	}
	// (30 F) <= AirTemp < (40 F)
	else if ((airTemp >= UnitConverter::f_To_K(30.0)) && (airTemp < UnitConverter::f_To_K(40.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.58 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.60 * pow(10.0, -7.0))) / 2.0;
	}
	// (40 F) <= AirTemp < (50 F)
	else if ((airTemp >= UnitConverter::f_To_K(40.0)) && (airTemp < UnitConverter::f_To_K(50.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.60 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.68 * pow(10.0, -7.0))) / 2.0;
	}
	// (50 F) <= AirTemp < (60 F)
	else if ((airTemp >= UnitConverter::f_To_K(50.0)) && (airTemp < UnitConverter::f_To_K(60.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.68 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.75 * pow(10.0, -7.0))) / 2.0;
	}
	// (60 F) <= AirTemp < (70 F)
	else if ((airTemp >= UnitConverter::f_To_K(60.0)) && (airTemp < UnitConverter::f_To_K(70.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.75 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.82 * pow(10.0, -7.0))) / 2.0;
	}
	// (70 F) <= AirTemp < (80 F)
	else if ((airTemp >= UnitConverter::f_To_K(70.0)) && (airTemp < UnitConverter::f_To_K(80.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.82 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.86 * pow(10.0, -7.0))) / 2.0;
	}
	// (80 F) <= AirTemp < (90 F)
	else if ((airTemp >= UnitConverter::f_To_K(80.0)) && (airTemp < UnitConverter::f_To_K(90.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.86 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.90 * pow(10.0, -7.0))) / 2.0;
	}
	// (90 F) <= AirTemp < (100 F)
	else if ((airTemp >= UnitConverter::f_To_K(90.0)) && (airTemp < UnitConverter::f_To_K(100.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.90 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(3.94 * pow(10.0, -7.0))) / 2.0;
	}
	// (100 F) <= AirTemp < (120 F)
	else if ((airTemp >= UnitConverter::f_To_K(100.0)) && (airTemp < UnitConverter::f_To_K(120.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(3.94 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(4.02 * pow(10.0, -7.0))) / 2.0;
	}
	// (120 F) <= AirTemp < (140 F)
	else if ((airTemp >= UnitConverter::f_To_K(120.0)) && (airTemp < UnitConverter::f_To_K(140.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(4.02 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(4.13 * pow(10.0, -7.0))) / 2.0;
	}
	// (140 F) <= AirTemp < (160 F)
	else if ((airTemp >= UnitConverter::f_To_K(140.0)) && (airTemp < UnitConverter::f_To_K(160.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(4.13 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(4.22 * pow(10.0, -7.0))) / 2.0;
	}
	// (160 F) <= AirTemp < (180 F)
	else if ((airTemp >= UnitConverter::f_To_K(160.0)) && (airTemp < UnitConverter::f_To_K(180.0))) {
		airDynVisc = (UnitConverter::psf_To_Pa(4.22 * pow(10.0, -7.0)) + UnitConverter::psf_To_Pa(4.34 * pow(10.0, -7.0))) / 2.0;
	}
	// (180 F) <= AirTemp
	else {
		airDynVisc = UnitConverter::psf_To_Pa(4.34 * pow(10.0, -7.0));
	}
	return airDynVisc;
}

