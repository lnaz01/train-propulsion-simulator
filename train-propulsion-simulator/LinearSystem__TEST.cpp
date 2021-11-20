//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <chrono>
#include "LinearSystem.h"
#include "ResultsWriter.h"


void BeamOnWinklerFoundation(double* e, double* i, double* k, double* p, double* x, int npts, std::string fnm);


int main() {

	// Start clock
	std::chrono::steady_clock::time_point begin_time = std::chrono::steady_clock::now();
	// Testing 'GaussianElimination' with beam on Winkler foundation example
	int npts = 1801;
	double* ym = new double[npts];
	double* mi = new double[npts];
	double* mod = new double[npts];
	double* p = new double[npts];
	double* x = new double[npts];
	for (int i = 0; i < npts; i++) {
		ym[i] = 30000000;
		mi[i] = 64.97;
		mod[i] = 3000.0;
		if (i == (int)(npts / 2.0)) {
			p[i] = -35000.0;
		}
		else {
			p[i] = 0.0;
		}
		x[i] = (int)(-npts / 8.0) + (i / 4.0);
	}
	std::string fnm = "C:/Users/Leith/Desktop/LinearSystem__TEST.csv";
	BeamOnWinklerFoundation(ym, mi, mod, p, x, npts, fnm);
	std::cout << "END" << std::endl;
	// Stop clock
	std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
	double elapsedTimeNANO = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - begin_time).count();
	double elapsedTime = elapsedTimeNANO / 1000000000;
	std::cout << "elapsedTime: " << elapsedTime << std::endl;
	delete[] ym;
	delete[] mi;
	delete[] mod;
	delete[] p;
	delete[] x;

}


void BeamOnWinklerFoundation(double* e, double* i, double* k, double* p, double* x, int npts, std::string fnm) {
	// Modify forcing vector 'p' values
	for (int j = 0; j < npts; j++) {
		p[j] = (p[j] / (abs(x[1] - x[0]))) * (1.0 / (e[j] * i[j])) *
			pow(abs(x[1] - x[0]), 4.0);
	}
	// System matrix
	double** sys = new double* [npts];
	for (int i = 0; i < npts; i++) {
		sys[i] = new double[npts];
	}
	for (int j = 0; j < npts; j++) {
		for (int j2 = 0; j2 < npts; j2++) {
			sys[j][j2] = 0.0;
		}
	}
	for (int j = 0; j < npts; j++) {
		sys[j][j] = 6.0 + (k[j] * (1.0 / (e[j] * i[j])) *
			pow(abs(x[1] - x[0]), 4.0));
	}
	for (int j = 0; j < (npts - 1); j++) {
		sys[j][j + 1] = -4.0;
	}
	for (int j = 1; j < npts; j++) {
		sys[j][j - 1] = -4.0;
	}
	for (int j = 0; j < (npts - 2); j++) {
		sys[j][j + 2] = 1.0;
	}
	for (int j = 2; j < npts; j++) {
		sys[j][j - 2] = 1.0;
	}
	// State space vector
	LinearSystem* linearSystem = new LinearSystem;
	linearSystem->gaussElimination(sys, p, npts);
	// Results writer
	ResultsWriter* resultsWriter = new ResultsWriter(fnm, false);
	for (int j = 0; j < npts; j++) {
		std::vector<double> d(2);
		d[0] = x[j];
		d[1] = linearSystem->stateSpaceVector[j];
		resultsWriter->writeLine(d);
	}
	resultsWriter->ofs->flush();
	resultsWriter->ofs->close();
	// Delete system matrix 'sys'
	for (int i = 0; i < npts; i++) {
		delete[] sys[i];
	}
	delete[] sys;
	// Delete linear system
	delete linearSystem;
	// Delete results writer
	delete resultsWriter;
}

