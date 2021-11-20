//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <vector>
#include "DynamicSystem.h"
#include "ResultsWriter.h"


DynamicSystem::DynamicSystem(int ssvLen1, int ssvLen2) {
	this->ssvLen1 = ssvLen1;
	this->ssvLen2 = ssvLen2;
	t = 0;
	tAlg = 0;
	tStep = 0;
	// State space variables
	ssv = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		ssv[i] = new double[ssvLen2];
	}
	// Approximated state space variables
	ssvApp = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		ssvApp[i] = new double[ssvLen2];
	}
	// Derivative of state space variables
	ssvdot = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		ssvdot[i] = new double[ssvLen2];
	}
	// Forcing function
	ff = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		ff[i] = new double[ssvLen2];
	}
	// 'k' values
	int max_runge_kutta_steps = 6;
	kVals = new double** [max_runge_kutta_steps];
	for (int i = 0; i < max_runge_kutta_steps; i++) {
		kVals[i] = new double* [ssvLen1];
		for (int j = 0; j < ssvLen1; j++) {
			kVals[i][j] = new double[ssvLen2];
		}
	}
	// Fourth order result
	res4 = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		res4[i] = new double[ssvLen2];
	}
	// Fifth order result
	res5 = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		res5[i] = new double[ssvLen2];
	}
	// Esimated error
	ee = new double* [ssvLen1];
	for (int i = 0; i < ssvLen1; i++) {
		ee[i] = new double[ssvLen2];
	}
}


DynamicSystem::~DynamicSystem() {
	// State space variables
	for (int i = 0; i < ssvLen1; i++) {
		delete[] ssv[i];
	}
	delete[] ssv;
	// Approximated state space variables
	for (int i = 0; i < ssvLen1; i++) {
		delete[] ssvApp[i];
	}
	delete[] ssvApp;
	// Derivative of state space variables
	for (int i = 0; i < ssvLen1; i++) {
		delete[] ssvdot[i];
	}
	delete[] ssvdot;
	// Forcing function
	for (int i = 0; i < ssvLen1; i++) {
		delete[] ff[i];
	}
	delete[] ff;
	// 'k' values
	int max_runge_kutta_steps = 6;
	for (int i = 0; i < max_runge_kutta_steps; i++) {
		for (int j = 0; j < ssvLen1; j++) {
			delete[] kVals[i][j];
		}
		delete[] kVals[i];
	}
	delete[] kVals;
	// Fourth order result
	for (int i = 0; i < ssvLen1; i++) {
		delete[] res4[i];
	}
	delete[] res4;
	// Fifth order result
	for (int i = 0; i < ssvLen1; i++) {
		delete[] res5[i];
	}
	delete[] res5;
	// Esimated error
	for (int i = 0; i < ssvLen1; i++) {
		delete[] ee[i];
	}
	delete[] ee;
}


void DynamicSystem::simulateRK4(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double fts) {
	tStep = fts;
	t = t0;
	double tw = t0;
	for (int i = 0; i < ssvLen1; i++) {
		for (int j = 0; j < ssvLen2; j++) {
			ssv[i][j] = ic[i][j];
		}
	}
	int rk4_nsteps = 4;
	writeTimeStepResults(resWriter, ssvi, t);
	do {
		for (int k = 0; k < rk4_nsteps; k++) {
			calc_tAlg("rk4", k);
			calc_ssvApp("rk4", k);
			calc_ff();
			calc_ssvdot();
			calc_kVals(k);
		}
		calc_res4("rk4");
		// Update time value
		t = t + tStep;
		// Update state-space variables
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				ssv[j][i] = res4[j][i];
			}
		}
		// Print results based on sample rate condition
		if ((t - tw) >= (1.0 / samprat)) {
			tw = t;
			writeTimeStepResults(resWriter, ssvi, t);
		}
	} while (t < tf);
}


void DynamicSystem::simulateRKF45(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double mints, double maxts, double maxerrthrsh) {
	tStep = maxts;
	t = t0;
	double tw = t0;
	for (int i = 0; i < ssvLen1; i++) {
		for (int j = 0; j < ssvLen2; j++) {
			ssv[i][j] = ic[i][j];
		}
	}
	int rkf45_nsteps = 6;
	writeTimeStepResults(resWriter, ssvi, t);
	do {
		if (tStep < mints) {
			std::cout << "Simulation terminated due to potential numerical ";
			std::cout << "instability." << std::endl;
			remove(resWriter->fp.c_str());
			exit(EXIT_SUCCESS);
		}
		for (int k = 0; k < rkf45_nsteps; k++) {
			calc_tAlg("rkf45", k);
			calc_ssvApp("rkf45", k);
			calc_ff();
			calc_ssvdot();
			calc_kVals(k);
		}
		calc_res4("rkf45");
		calc_res5("rkf45");
		double maxee = calc_ee(maxerrthrsh);
		if (maxee > maxerrthrsh) {
			tStep = tStep * 0.75;
		}
		else {
			// Update time value
			t = t + tStep;
			// Update state-space variables
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssv[j][i] = res4[j][i];
				}
			}
			// Print results based on sample rate condition
			if ((t - tw) >= (1.0 / samprat)) {
				tw = t;
				writeTimeStepResults(resWriter, ssvi, t);
			}
			// Calculate next time step
			tStep = tStep * 1.25;
			if (tStep > maxts) {
				tStep = maxts;
			}

		}
	} while (t < tf);
}


void DynamicSystem::simulateRKCK45(ResultsWriter* resWriter, double t0, double tf, double** ic, double samprat, std::vector<int> ssvi, double mints, double maxts, double maxerrthrsh, double deserrthrsh) {
	tStep = maxts;
	t = t0;
	double tw = t0;
	for (int i = 0; i < ssvLen1; i++) {
		for (int j = 0; j < ssvLen2; j++) {
			ssv[i][j] = ic[i][j];
		}
	}
	int rkck45_nsteps = 6;
	writeTimeStepResults(resWriter, ssvi, t);
	do {
		if (tStep < mints) {
			std::cout << "Simulation terminated due to potential numerical ";
			std::cout << "instability." << std::endl;
			remove(resWriter->fp.c_str());
			exit(EXIT_SUCCESS);
		}
		for (int k = 0; k < rkck45_nsteps; k++) {
			calc_tAlg("rkck45", k);
			calc_ssvApp("rkck45", k);
			calc_ff();
			calc_ssvdot();
			calc_kVals(k);
		}
		calc_res4("rkck45");
		calc_res5("rkck45");
		double maxee = calc_ee(maxerrthrsh);
		if (maxee < maxerrthrsh) {
			// Update time value
			t = t + tStep;
			// Update state-space variables
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssv[j][i] = res4[j][i];
				}
			}
			// Print results based on sample rate condition
			if ((t - tw) >= (1.0 / samprat)) {
				tw = t;
				writeTimeStepResults(resWriter, ssvi, t);
			}
		}
		// Calculate revised/next time step
		tStep = tStep * pow(std::abs(deserrthrsh / maxee), 0.2);
		if (tStep > maxts) {
			tStep = maxts;
		}
	} while (t < tf);
}


void DynamicSystem::calc_tAlg(std::string algName, int algStep) {
	if (algName.compare("rk4") == 0) {
		if (algStep == 0) {
			tAlg = t;
		}
		else if (algStep == 1) {
			tAlg = t + ((1.0 / 2.0) * tStep);
		}
		else if (algStep == 2) {
			tAlg = t + ((1.0 / 2.0) * tStep);
		}
		else if (algStep == 3) {
			tAlg = t + tStep;
		}
	}
	else if (algName.compare("rkf45") == 0) {
		if (algStep == 0) {
			tAlg = t;
		}
		else if (algStep == 1) {
			tAlg = t + ((1.0 / 4.0) * tStep);
		}
		else if (algStep == 2) {
			tAlg = t + ((3.0 / 8.0) * tStep);
		}
		else if (algStep == 3) {
			tAlg = t + ((12.0 / 13.0) * tStep);
		}
		else if (algStep == 4) {
			tAlg = t + tStep;
		}
		else if (algStep == 5) {
			tAlg = t + ((1.0 / 2.0) * tStep);
		}
	}
	else if (algName.compare("rkck45") == 0) {
		if (algStep == 0) {
			tAlg = t;
		}
		else if (algStep == 1) {
			tAlg = t + ((1.0 / 5.0) * tStep);
		}
		else if (algStep == 2) {
			tAlg = t + ((3.0 / 10.0) * tStep);
		}
		else if (algStep == 3) {
			tAlg = t + ((3.0 / 5.0) * tStep);
		}
		else if (algStep == 4) {
			tAlg = t + tStep;
		}
		else if (algStep == 5) {
			tAlg = t + ((7.0 / 8.0) * tStep);
		}
	}
}


void DynamicSystem::calc_ssvApp(std::string algName, int algStep) {
	if (algName.compare("rk4") == 0) {
		if (algStep == 0) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i];
				}
			}
		}
		else if (algStep == 1) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i] +
						((1.0 / 2.0) * kVals[0][j][i]);
				}
			}
		}
		else if (algStep == 2) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i] +
						((1.0 / 2.0) * kVals[1][j][i]);
				}
			}
		}
		else if (algStep == 3) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i] +
						kVals[2][j][i];
				}
			}
		}
	}
	else if (algName.compare("rkf45") == 0) {
		if (algStep == 0) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i];
				}
			}
		}
		else if (algStep == 1) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((1.0 / 4.0) * kVals[0][j][i]);
				}
			}
		}
		else if (algStep == 2) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((3.0 / 32.0) * kVals[0][j][i])
						+ ((9.0 / 32.0) * kVals[1][j][i]);
				}
			}
		}
		else if (algStep == 3) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((1932.0 / 2197.0) * kVals[0][j][i])
						- ((7200.0 / 2197.0) * kVals[1][j][i])
						+ ((7296.0 / 2197.0) * kVals[2][j][i]);
				}
			}
		}
		else if (algStep == 4) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((439.0 / 216.0) * kVals[0][j][i])
						- ((8.0 / 1.0) * kVals[1][j][i])
						+ ((3680.0 / 513.0) * kVals[2][j][i])
						- ((845.0 / 4104.0) * kVals[3][j][i]);
				}
			}
		}
		else if (algStep == 5) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						- ((8.0 / 27.0) * kVals[0][j][i])
						+ ((2.0 / 1.0) * kVals[1][j][i])
						- ((3544.0 / 2565.0) * kVals[2][j][i])
						+ ((1859.0 / 4104.0) * kVals[3][j][i])
						- ((11.0 / 40.0) * kVals[4][j][i]);
				}
			}
		}
	}
	else if (algName.compare("rkck45") == 0) {
		if (algStep == 0) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i];
				}
			}
		}
		else if (algStep == 1) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((1.0 / 5.0) * kVals[0][j][i]);
				}
			}
		}
		else if (algStep == 2) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((3.0 / 40.0) * kVals[0][j][i])
						+ ((9.0 / 40.0) * kVals[1][j][i]);
				}
			}
		}
		else if (algStep == 3) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((3.0 / 10.0) * kVals[0][j][i])
						- ((9.0 / 10.0) * kVals[1][j][i])
						+ ((6.0 / 5.0) * kVals[2][j][i]);
				}
			}
		}
		else if (algStep == 4) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						- ((11.0 / 45.0) * kVals[0][j][i])
						+ ((5.0 / 2.0) * kVals[1][j][i])
						- ((70.0 / 27.0) * kVals[2][j][i])
						+ ((35.0 / 27.0) * kVals[3][j][i]);
				}
			}
		}
		else if (algStep == 5) {
			for (int j = 0; j < ssvLen1; j++) {
				for (int i = 0; i < ssvLen2; i++) {
					ssvApp[j][i] = ssv[j][i]
						+ ((1631.0 / 55296.0) * kVals[0][j][i])
						+ ((175.0 / 512.0) * kVals[1][j][i])
						+ ((575.0 / 13824.0) * kVals[2][j][i])
						+ ((44275.0 / 110592.0) * kVals[3][j][i])
						+ ((253.0 / 4096.0) * kVals[4][j][i]);
				}
			}
		}
	}
}


void DynamicSystem::calc_kVals(int algStep) {
	for (int j = 0; j < ssvLen1; j++) {
		for (int i = 0; i < ssvLen2; i++) {
			kVals[algStep][j][i] = tStep * ssvdot[j][i];
		}
	}
}


void DynamicSystem::calc_res4(std::string algName) {
	if (algName.compare("rk4") == 0) {
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				res4[j][i] = ssv[j][i] + ((1.0 / 6.0) *
					(kVals[0][j][i]
						+ (2.0 * kVals[1][j][i])
						+ (2.0 * kVals[2][j][i])
						+ kVals[3][j][i]));
			}
		}
	}
	else if (algName.compare("rkf45") == 0) {
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				res4[j][i] = ssv[j][i]
					+ ((25.0 / 216.0) * kVals[0][j][i])
					+ ((1408.0 / 2565.0) * kVals[2][j][i])
					+ ((2197.0 / 4104.0) * kVals[3][j][i])
					- ((1.0 / 5.0) * kVals[4][j][i]);
			}
		}
	}
	else if (algName.compare("rkck45") == 0) {
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				res4[j][i] = ssv[j][i]
					+ ((37.0 / 378.0) * kVals[0][j][i])
					+ ((250.0 / 621.0) * kVals[2][j][i])
					+ ((125.0 / 594.0) * kVals[3][j][i])
					+ ((512.0 / 1771.0) * kVals[5][j][i]);
			}
		}
	}
}


void DynamicSystem::calc_res5(std::string algName) {
	if (algName.compare("rkf45") == 0) {
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				res5[j][i] = ssv[j][i]
					+ ((16.0 / 135.0) * kVals[0][j][i])
					+ ((6656.0 / 12825.0) * kVals[2][j][i])
					+ ((28561.0 / 56430.0) * kVals[3][j][i])
					- ((9.0 / 50.0) * kVals[4][j][i])
					+ ((2.0 / 55.0) * kVals[5][j][i]);
			}
		}
	}
	else if (algName.compare("rkck45") == 0) {
		for (int j = 0; j < ssvLen1; j++) {
			for (int i = 0; i < ssvLen2; i++) {
				res5[j][i] = ssv[j][i]
					+ ((2825.0 / 27648.0) * kVals[0][j][i])
					+ ((18575.0 / 48384.0) * kVals[2][j][i])
					+ ((13525.0 / 55296.0) * kVals[3][j][i])
					+ ((277.0 / 14336.0) * kVals[4][j][i])
					+ ((1.0 / 4.0) * kVals[5][j][i]);
			}
		}
	}
}


double DynamicSystem::calc_ee(double maxerrthrsh) {
	double maxee = 0.0;
	for (int j = 0; j < ssvLen1; j++) {
		for (int i = 0; i < ssvLen2; i++) {
			if (std::abs(res5[j][i] - res4[j][i]) > maxee) {
				maxee = std::abs(res5[j][i] - res4[j][i]);
			}
		}
	}
	if (maxee == 0.0) {
		maxee = maxerrthrsh / 1000.0;
	}
	return maxee;
}


void DynamicSystem::writeTimeStepResults(ResultsWriter* resWriter, std::vector<int> ssvi, double t) {
	int resLen = (ssvLen2 * ssvLen1) + 1;
	std::vector<double> res;
	res.clear();
	res.push_back(t);
	for (int i = 0; i < ssvLen2; i++) {
		for (int j = 0; j < ssvi.size(); j++) {
			res.push_back(ssv[ssvi[j]][i]);
		}
	}
	resWriter->writeLine(res);
}

