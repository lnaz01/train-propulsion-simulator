//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "IntervalSmooth.h"
#include "InputFileReader_Simulation.h"
#include "Point.h"
#include "ResultsWriter.h"
#include "Track.h"


void console__TEST(IntervalSmooth* intervalSmooth);
void interpolate__TEST(IntervalSmooth* intervalSmooth);
void interpolateMult__TEST(IntervalSmooth* intervalSmooth);
void add_pt__TEST(IntervalSmooth* intervalSmooth);
void copy__TEST(IntervalSmooth* intervalSmooth);
void load__TEST();


int main() {

	int intervalIndex = 0;
	IntervalSmooth* intervalSmooth = new IntervalSmooth(intervalIndex, 5);
	// Add points
	Point* point1 = new Point(5.5, 20.0);
	Point* point2 = new Point(7.0, 26.5);
	Point* point3 = new Point(7.5, 16.5);
	Point* point4 = new Point(8.0, 17.0);
	intervalSmooth->add_point(point1);
	intervalSmooth->add_point(point2);
	intervalSmooth->add_point(point3);
	intervalSmooth->add_point(point4);
	console__TEST(intervalSmooth);
	interpolate__TEST(intervalSmooth);
	interpolateMult__TEST(intervalSmooth);
	add_pt__TEST(intervalSmooth);
	copy__TEST(intervalSmooth);
	load__TEST();

}


// Tests printing smooth interval to console
void console__TEST(IntervalSmooth* intervalSmooth) {
	std::cout << "console__TEST" << std::endl;
	std::cout << "intervalSmooth: " << std::endl;
	intervalSmooth->console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests single point interpolation of smooth interval
void interpolate__TEST(IntervalSmooth* intervalSmooth) {
	double interpolated_value = intervalSmooth->interpolate(7.2, 0.0, 30.0);
	std::cout << "interpolate__TEST" << std::endl;
	std::cout << "interpolated_value: " << interpolated_value << std::endl;
	std::cout << std::endl << std::endl << std::endl;
}


// Tests multiple point interpolation of smooth interval
void interpolateMult__TEST(IntervalSmooth* intervalSmooth) {
	intervalSmooth->interpolateMult(0.0, 40.0);
	Point* points = new Point[intervalSmooth->numInterpPointsPlot + 2];
	std::cout << "interpolateMult__TEST" << std::endl;
	for (int i = 0; i < (intervalSmooth->numInterpPointsPlot + 2); i++) {
		points[i].x = intervalSmooth->interpPoints[i][0];
		points[i].y = intervalSmooth->interpPoints[i][1];
		std::cout << "points[" << i << "]: ";
		points[i].console();
		std::cout << std::endl;
	}
	std::string fp = "C:/Users/Leith/Desktop/interpolateMult__TEST.csv";
	ResultsWriter resultsWriter(fp, false);
	resultsWriter.writePoints(points, intervalSmooth->numInterpPointsPlot + 2);
	resultsWriter.ofs->flush();
	resultsWriter.ofs->close();
	std::cout << std::endl << std::endl << std::endl;
	delete[] points;
}


// Tests adding point to smooth interval
void add_pt__TEST(IntervalSmooth* intervalSmooth) {
	Point* point5 = new Point(9.5, 4.5);
	intervalSmooth->add_point(point5);
	std::cout << "add_pt__TEST" << std::endl;
	std::cout << "intervalSmooth: " << std::endl;
	intervalSmooth->console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests copying smooth interval
void copy__TEST(IntervalSmooth* intervalSmooth) {
	std::cout << "copy__TEST" << std::endl;
	std::cout << "intervalSmooth.intervalIndex: " << intervalSmooth->intervalIndex << std::endl;
	std::cout << "intervalSmooth: " << std::endl;
	intervalSmooth->console();
	IntervalSmooth intervalSmooth_copy(9, 5);
	intervalSmooth_copy.copy(intervalSmooth);
	std::cout << std::endl;
	std::cout << "intervalSmooth_copy.intervalIndex: " << intervalSmooth_copy.intervalIndex << std::endl;
	std::cout << "intervalSmooth_copy: " << std::endl;
	intervalSmooth_copy.console();
	std::cout << std::endl << std::endl << std::endl;
}


// Tests loading smooth interval from file
void load__TEST() {
	Track* track = new Track(new InputFileReader_Simulation());
	//std::string stril = "0.0, 0.1; 105600.4, 1.0";
	std::string stril = "0.0, 0.1; 5.0, 1.1; 60000.0, 2.2";
	IntervalSmooth* intervalSmooth = new IntervalSmooth(0, 3);
	std::string strval = intervalSmooth->load(stril, track, track->physicalVariables[0]);
	std::cout << "load__TEST" << std::endl;
	std::cout << "strval: " << strval << std::endl;
	std::cout << "intervalSmooth: " << std::endl;
	intervalSmooth->console();
	std::cout << std::endl << std::endl << std::endl;
}

