//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "HarmonicOscillator.h"


HarmonicOscillator::HarmonicOscillator(int mSize, double* m, double* c, double* k) : DynamicSystem(2, mSize) {
	this->m = new double[mSize];
	this->c = new double[mSize];
	this->k = new double[mSize];
	for (int i = 0; i < mSize; i++) {
		this->m[i] = m[i];
		this->c[i] = c[i];
		this->k[i] = k[i];
	}
}


HarmonicOscillator::~HarmonicOscillator() {
	delete[] m;
	delete[] c;
	delete[] k;
}


void HarmonicOscillator::calc_ssvdot() {
	for (int i = 0; i < ssvLen2; i++) {
		ssvdot[0][i] = ssvApp[1][i];
		if (ssvLen2 == 1) {
			ssvdot[1][i] = (
				-(k[i] * (ssvApp[0][i] - 0))
				- (c[i] * (ssvApp[1][i] - 0))
				+ ff[1][i]
				) / m[i];
		}
		else {
			if (i == (ssvLen2 - 1)) {
				ssvdot[1][i] = (
					-(k[i] * (ssvApp[0][i] - ssvApp[0][i - 1]))
					- (c[i] * (ssvApp[1][i] - ssvApp[1][i - 1]))
					+ ff[1][i]
					) / m[i];
			}
			else if (i == 0) {
				ssvdot[1][i] = (
					-(k[i] * ssvApp[0][i])
					- (c[i] * ssvApp[1][i])
					- (k[i + 1] * (ssvApp[0][i] - ssvApp[0][i + 1]))
					- (c[i + 1] * (ssvApp[1][i] - ssvApp[1][i + 1]))
					+ ff[1][i]
					) / m[i];
			}
			else {
				ssvdot[1][i] = (
					-(k[i] * (ssvApp[0][i] - ssvApp[0][i - 1]))
					- (c[i] * (ssvApp[1][i] - ssvApp[1][i - 1]))
					- (k[i + 1] * (ssvApp[0][i] - ssvApp[0][i + 1]))
					- (c[i + 1] * (ssvApp[1][i] - ssvApp[1][i + 1]))
					+ ff[1][i]
					) / m[i];
			}
		}
	}
}


void  HarmonicOscillator::calc_ff() {
	for (int i = 0; i < ssvLen1; i++) {
		for (int j = 0; j < ssvLen2; j++) {
			ff[i][j] = 0.0;
		}
	}
}

