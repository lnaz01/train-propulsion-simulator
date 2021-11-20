//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "ExplicitSSComponent.h"
#include "InputFileReader_Simulation.h"
#include "Simulation.h"


ExplicitSSComponent::ExplicitSSComponent(InputFileReader_Simulation* inputFileReader_Simulation, int physicalConstantsSize, int physicalVariablesSize,
	int ssvSize) : UserDefinedRRComponent(inputFileReader_Simulation, physicalConstantsSize, physicalVariablesSize) {
	this->ssvSize = ssvSize;
	ssv = new double[ssvSize];
	ssvApp = new double[ssvSize];
	ssvDot = new double[ssvSize];
	kValsBool = false;
	res4 = new double[ssvSize];
	res5 = new double[ssvSize];
	ee = new double[ssvSize];
}


ExplicitSSComponent::~ExplicitSSComponent() {
	delete[] ssv;
	delete[] ssvApp;
	delete[] ssvDot;
	if (kValsBool == true) {
		for (int i = 0; i < inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverNumSteps; i++) {
			delete[] kVals[i];
		}
		delete[] kVals;
	}
	delete[] res4;
	delete[] res5;
	delete[] ee;
}


void ExplicitSSComponent::update_ssv() {
	for (int i = 0; i < ssvSize; i++) {
		ssv[i] = res4[i];
	}
}


void ExplicitSSComponent::calc_ssvApp(InputFileReader_Simulation* inputFileReader_Simulation) {
	if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverType == 0) {
		if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 0) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i];
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 1) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((1.0 / 2.0) * kVals[0][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 2) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((1.0 / 2.0) * kVals[1][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 3) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ kVals[2][i];
			}
		}
	}
	else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverType == 1) {
		if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 0) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i];
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 1) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((1.0 / 4.0) * kVals[0][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 2) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((3.0 / 32.0) * kVals[0][i])
					+ ((9.0 / 32.0) * kVals[1][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 3) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((1932.0 / 2197.0) * kVals[0][i])
					- ((7200.0 / 2197.0) * kVals[1][i])
					+ ((7296.0 / 2197.0) * kVals[2][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 4) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					+ ((439.0 / 216.0) * kVals[0][i])
					- ((8.0 / 1.0) * kVals[1][i])
					+ ((3680.0 / 513.0) * kVals[2][i])
					- ((845.0 / 4104.0) * kVals[3][i]);
			}
		}
		else if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex == 5) {
			for (int i = 0; i < ssvSize; i++) {
				ssvApp[i] = ssv[i]
					- ((8.0 / 27.0) * kVals[0][i])
					+ ((2.0 / 1.0) * kVals[1][i])
					- ((3544.0 / 2565.0) * kVals[2][i])
					+ ((1859.0 / 4104.0) * kVals[3][i])
					- ((11.0 / 40.0) * kVals[4][i]);
			}
		}
	}
}


void ExplicitSSComponent::calc_kVals() {
	if (kValsBool == false) {
		kVals = new double* [inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverNumSteps];
		for (int i = 0; i < inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverNumSteps; i++) {
			kVals[i] = new double[ssvSize];
		}
		kValsBool = true;
	}
	for (int i = 0; i < ssvSize; i++) {
		kVals[inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverStepIndex][i] =
			inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverTimeStep * ssvDot[i];
	}
}


void ExplicitSSComponent::calc_res4() {
	if (inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverType == 0) {
		for (int i = 0; i < ssvSize; i++) {
			res4[i] = ssv[i] + ((1.0 / 6.0) *
				(kVals[0][i]
					+ (2.0 * kVals[1][i])
					+ (2.0 * kVals[2][i])
					+ kVals[3][i]));
		}
	}
	else {  // inputFileReader_Simulation->userDefinedSimulations[0]->explicitSolverType == 1
		for (int i = 0; i < ssvSize; i++) {
			res4[i] = ssv[i]
				+ ((25.0 / 216.0) * kVals[0][i])
				+ ((1408.0 / 2565.0) * kVals[2][i])
				+ ((2197.0 / 4104.0) * kVals[3][i])
				- ((1.0 / 5.0) * kVals[4][i]);
		}
	}
}


void ExplicitSSComponent::calc_res5() {
	for (int i = 0; i < ssvSize; i++) {
		res5[i] = ssv[i]
			+ ((16.0 / 135.0) * kVals[0][i])
			+ ((6656.0 / 12825.0) * kVals[2][i])
			+ ((28561.0 / 56430.0) * kVals[3][i])
			- ((9.0 / 50.0) * kVals[4][i])
			+ ((2.0 / 55.0) * kVals[5][i]);
	}
}


double ExplicitSSComponent::calc_ee() {
	double max_est_error = 0.0;
	for (int i = 0; i < ssvSize; i++) {
		ee[i] = std::abs(res5[i] - res4[i]);
		if (ee[i] > max_est_error) {
			max_est_error = ee[i];
		}
	}
	return max_est_error;
}

