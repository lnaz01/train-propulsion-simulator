//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include "ResultsWriter.h"
#include "Point.h"


ResultsWriter::ResultsWriter(std::string fp, bool appnd) {
	this->fp = fp;
	ofs = new std::ofstream;
	if (appnd == false) {
		// Remove file with file path 'fp' if it already exists
		FILE* file1;
		if (file1 = fopen(fp.c_str(), "r")) {
			fclose(file1);
			remove(fp.c_str());
		}
		/*
		FILE* file1;
		errno_t err1;
		if (err1 = fopen_s(&file1, fp.c_str(), "r") == 0) {
			fclose(file1);
			remove(fp.c_str());
		}
		*/
		this->ofs->open(fp, std::ios::trunc);
	}
	else {
		this->ofs->open(fp, std::ios::app);
	}
}


ResultsWriter::~ResultsWriter() {
	delete ofs;
}


void ResultsWriter::writePoints(Point* points, int pointsSize) {
	for (int i = 0; i < pointsSize; i++) {
		*ofs << points[i].x << "," << points[i].y << std::endl;
	}
}

