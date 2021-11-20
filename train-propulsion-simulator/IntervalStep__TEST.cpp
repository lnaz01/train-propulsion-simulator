//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "IntervalStep.h"
#include "InputFileReader_Simulation.h"
#include "Point.h"
#include "ResultsWriter.h"
#include "Track.h"


void console__TEST(IntervalStep* intervalStep);
void interpolate_TEST1(IntervalStep* intervalStep);
void interpolate_TEST2(IntervalStep* intervalStep);
void add_point__TEST(IntervalStep* intervalStep);
void copy__TEST(IntervalStep* intervalStep);
void load__TEST();


int main() {
	int ili = 0;
	IntervalStep* intervalStep = new IntervalStep(ili);
	// Add points
	Point* point1 = new Point(5.5, 20.0);
	Point* point2 = new Point(8.0, 20.0);
	intervalStep->add_point(point1);
	intervalStep->add_point(point2);
	// Test
	console__TEST(intervalStep);
	interpolate_TEST1(intervalStep);
	interpolate_TEST2(intervalStep);
	add_point__TEST(intervalStep);
	copy__TEST(intervalStep);
	load__TEST();
}


// Tests printing smooth interval to console
void console__TEST(IntervalStep* intervalStep1) {
	std::cout << "console__TEST" << std::endl;
	std::cout << "intervalStep1: " << std::endl;
	intervalStep1->console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests single point interpolation of smooth interval
void interpolate_TEST1(IntervalStep* intervalStep1) {
	double interpolated_value = intervalStep1->interpolate(7.2, 0.0, 30.0);
	std::cout << "interpolate__TEST" << std::endl;
	std::cout << "interpolated_value: " << interpolated_value << std::endl;
	std::cout << std::endl << std::endl << std::endl;
}


// Tests multiple point interpolation of smooth interval
void interpolate_TEST2(IntervalStep* intervalStep1) {
	intervalStep1->interpolateMult(0.0, 40.0);
	Point* points = new Point[intervalStep1->numInterpPointsPlot + 2];
	std::cout << "interpolateMult__TEST" << std::endl;
	for (int i = 0; i < (intervalStep1->numInterpPointsPlot + 2); i++) {
		points[i].x = intervalStep1->interpPoints[i][0];
		points[i].y = intervalStep1->interpPoints[i][1];
		std::cout << "points[" << i << "]: " << std::endl;
		points[i].console();
		std::cout << std::endl;
	}
	std::string fp = "C:/Users/Leith/Desktop/interpolateMult__TEST.csv";
	ResultsWriter resultsWriter(fp, false);
	resultsWriter.writePoints(points, intervalStep1->numInterpPointsPlot + 2);
	resultsWriter.ofs->flush();
	resultsWriter.ofs->close();
	std::cout << std::endl << std::endl << std::endl;
	delete[] points;
}


// Tests adding point to smooth interval
void add_point__TEST(IntervalStep* intervalStep1) {
	Point* point = new Point(9.5, 4.5);
	intervalStep1->add_point(point);
	std::cout << "add_point__TEST" << std::endl;
	std::cout << "intervalStep1: " << std::endl;
	intervalStep1->console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests copying smooth interval
void copy__TEST(IntervalStep* intervalStep1) {
	std::cout << "copy__TEST" << std::endl;
	std::cout << "intervalStep1: " << std::endl;
	intervalStep1->console();
	IntervalStep intervalStep_copy(5);
	intervalStep_copy.copy(intervalStep1);
	std::cout << std::endl;
	std::cout << "intervalStep1_copy: " << std::endl;
	intervalStep_copy.console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests loading smooth interval from file
void load__TEST() {
	Track* track = new Track(new InputFileReader_Simulation());
	//std::string stril = "0.0, 0.1; 105600.4, 1.0";
	std::string stril = "0.0, 0.1; 5.0, 1.1; 60000.0, 2.2";
	IntervalStep* intervalStep = new IntervalStep(0);
	std::string strval = intervalStep->load(stril, track, track->physicalVariables[0]);
	std::cout << "load__TEST" << std::endl;
	std::cout << "strval: " << strval << std::endl;
	std::cout << "intervalSmooth: " << std::endl;
	intervalStep->console();
	std::cout << std::endl << std::endl << std::endl;
}

