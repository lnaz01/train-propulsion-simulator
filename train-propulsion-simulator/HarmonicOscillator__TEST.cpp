//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <string>
#include <vector>
#include "HarmonicOscillator.h"
#include "ResultsWriter.h"


int main() {

	// Array of masses, damping constants, and spring constants
	int numMasses = 1;
	double* m = new double[1];
	m[0] = 1.0;
	double* c = new double[1];
	c[0] = 0.0;
	double* k = new double[1];
	k[0] = 1000.0;

	// Initial conditions
	int icLen1 = 2;  // 0 --> Position,  1 --> Velocity
	double** ic = new double* [icLen1];
	for (int i = 0; i < icLen1; i++) {
		ic[i] = new double[numMasses];
	}
	for (int i = 0; i < numMasses; i++) {
		ic[0][i] = 1.0;
		ic[1][i] = 0.0;
	}

	// Harmonic oscillator
	HarmonicOscillator* myHarmonicOscillator = new HarmonicOscillator(numMasses, m, c, k);

	// Output file path and results writer for RK4 algorithm
	std::string outputFilePathRK4 = "C:/Users/Leith/Desktop/HarmonicOscillator__RK4.csv";
	ResultsWriter* myResultsWriterRK4 = new ResultsWriter(outputFilePathRK4, false);

	// Simulate using RK4 algorithm
	double t0 = 0.0;
	double tf = 60.0;
	double samprat = 50;
	std::vector<int> ssvi;
	ssvi.push_back(0);
	double fts = 0.01;
	std::cout << "Running RK4 algorithm..." << std::endl;
	myHarmonicOscillator->simulateRK4(myResultsWriterRK4, t0, tf, ic, samprat, ssvi, fts);

	// Flush and close output file stream in results writer for RK4 algorithm
	myResultsWriterRK4->ofs->flush();
	myResultsWriterRK4->ofs->close();

	// Output file path and results writer for RKF45 algorithm
	std::string outputFilePathRKF45 = "C:/Users/Leith/Desktop/HarmonicOscillator__RKF45.csv";
	ResultsWriter* myResultsWriterRKF45 = new ResultsWriter(outputFilePathRKF45, false);

	// Simulate using RKF45 algorithm
	double mints = 0.00001;
	double maxts = 0.05;
	double maxerrthrsh = pow(10.0, -5.0);
	std::cout << "Running RKF45 algorithm..." << std::endl;
	myHarmonicOscillator->simulateRKF45(myResultsWriterRKF45, t0, tf, ic, samprat, ssvi, mints, maxts, maxerrthrsh);

	// Flush and close output file stream in results writer for RKF45 algorithm
	myResultsWriterRKF45->ofs->flush();
	myResultsWriterRKF45->ofs->close();

	// Output file path and results writer for RKCK45 algorithm
	std::string outputFilePathRKCK45 = "C:/Users/Leith/Desktop/HarmonicOscillator__RKCK45.csv";
	ResultsWriter* myResultsWriterRKCK45 = new ResultsWriter(outputFilePathRKCK45, false);

	// Simulate using RKCK45 algorithm
	double deserrthrsh = pow(10.0, -6.0);
	std::cout << "Running RKCK45 algorithm..." << std::endl;
	myHarmonicOscillator->simulateRKCK45(myResultsWriterRKCK45, t0, tf, ic, samprat, ssvi, mints, maxts, maxerrthrsh, deserrthrsh);

	// Flush and close output file stream in results writer for RKCK45 algorithm
	myResultsWriterRKCK45->ofs->flush();
	myResultsWriterRKCK45->ofs->close();

	// Delete harmonic oscillator and results writers
	delete[] m;
	delete[] c;
	delete[] k;
	delete myHarmonicOscillator;
	delete myResultsWriterRK4;
	delete myResultsWriterRKF45;
	delete myResultsWriterRKCK45;

}

