//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <string>
#include <iostream>
#include "ResultsWriter.h"
#include "Point.h"


void writeLine__TEST1(ResultsWriter* resultsWriter);
void writeLine__TEST2(ResultsWriter* resultsWriter);
void writePoints__TEST(ResultsWriter* resultsWriter);


int main() {

	// File path
	std::string myFilePath = "C:/Users/Leith/Desktop/myTestFile.csv";
	// Initialize results writer
	ResultsWriter* myWriter = new ResultsWriter(myFilePath, false);
	// Test writing array of strings
	writeLine__TEST1(myWriter);
	// Test writing array of doubles
	writeLine__TEST2(myWriter);
	// Test writing array of points (one point per line)
	writePoints__TEST(myWriter);
	// Flush and close ouput file stream
	myWriter->ofs->flush();
	myWriter->ofs->close();
	// Delete results writer
	delete myWriter;

}


void writeLine__TEST1(ResultsWriter* resultsWriter) {
	//std::string myStringArray[3] = { "Locomotive 1", "Car 1", "Car 2" };
	//resultsWriter->writeLine(myStringArray, 3);
	std::vector<std::string> myStringVector{ "Locomotive 1", "Car 1", "Car 2" };
	resultsWriter->writeLine(myStringVector);
}


void writeLine__TEST2(ResultsWriter* resultsWriter) {
	//double myDoubleArray[3] = { 400.0, 280.0, 250.0 };
	//resultsWriter->writeLine(myDoubleArray, 3);
	std::vector<double> myDoubleVector{ 400.0, 280.0, 250.0 };
	resultsWriter->writeLine(myDoubleVector);
}


void writePoints__TEST(ResultsWriter* resultsWriter) {
	// Array of points
	Point* points = new Point[3];
	points[0].x = 2.1;
	points[0].y = 8.9;
	points[1].x = 4.1;
	points[1].y = 3.5;
	points[2].x = 5.8;
	points[2].y = 7.1;
	// Write points
	resultsWriter->writePoints(points, 3);
	// Delete array of points
	delete[] points;
}

