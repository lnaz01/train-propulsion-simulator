//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "Function.h"
#include "IntervalSmooth.h"
#include "Point.h"
#include "ResultsWriter.h"


void console__TEST(Function* function1);
void interpolate__TEST(Function* function1);
void interpolateMult__TEST(Function* function1);
void add_interval__TEST(Function* function1);
void copy__TEST(Function* function1);


int main() {

	// Add points
	Function* functionSmooth = new Function(false, 0);
	functionSmooth->add_interval();
	functionSmooth->intervals[0]->add_point(new Point(5.5, 20.0));
	functionSmooth->intervals[0]->add_point(new Point(7.0, 26.5));
	functionSmooth->intervals[0]->add_point(new Point(7.5, 16.5));
	functionSmooth->intervals[0]->add_point(new Point(8.0, 17.0));
	console__TEST(functionSmooth);
	interpolate__TEST(functionSmooth);
	interpolateMult__TEST(functionSmooth);
	add_interval__TEST(functionSmooth);
	copy__TEST(functionSmooth);

}


// Prints function to console
void console__TEST(Function* function1) {
	std::cout << "console__TEST" << std::endl;
	std::cout << "function1: " << std::endl;
	function1->console();
	std::cout << std::endl;
	std::cout << "function1_fni: " << function1->functionIndex << std::endl;
	std::cout << std::endl << std::endl << std::endl;
}


// Tests single point interpolation of function
void interpolate__TEST(Function* function1) {
	double interpolated_value = function1->interpolate(7.4, 0.0, 40.0);
	std::cout << "interpolate__TEST" << std::endl;
	std::cout << "interpolated_value: " << interpolated_value << std::endl;
	std::cout << std::endl << std::endl << std::endl;
}


// Tests multiple point interpolation of function
void interpolateMult__TEST(Function* function1) {
	function1->interpolateMult(0.0, 40.0);
	Point* points = new Point[function1->numInterpPointsPlot];
	for (int i = 0; i < function1->numInterpPointsPlot; i++) {
		points[i].x = function1->interpPoints[i][0];
		points[i].y = function1->interpPoints[i][1];
	}
	std::cout << "interpolateMult__TEST" << std::endl;
	for (int i = 0; i < function1->numInterpPointsPlot; i++) {
		std::cout << "points[" << i << "]: ";
		points[i].console();
		std::cout << std::endl;
	}
	std::string fp = "C:/Users/Leith/Desktop/interpolateMult__TEST.csv";
	ResultsWriter resultsWriter(fp, false);
	resultsWriter.writePoints(points, function1->numInterpPointsPlot);
	resultsWriter.ofs->flush();
	resultsWriter.ofs->close();
	std::cout << std::endl << std::endl << std::endl;
	delete[] points;
}


// Tests adding interval to function
void add_interval__TEST(Function* function1) {
	// Add interval
	function1->add_interval();
	// Add points to newly added interval
	function1->intervals[1]->add_point(new Point(8.0, 17.0));
	function1->intervals[1]->add_point(new Point(9.0, 19.0));
	function1->intervals[1]->add_point(new Point(10.0, 11.0));
	function1->intervals[1]->add_point(new Point(12.0, 13.0));
	std::cout << "add_interval__TEST" << std::endl;
	std::cout << "function1: " << std::endl;
	function1->console();
	std::cout << std::endl;
	std::cout << "function1->intervals[0]->intervalIndex: " << function1->intervals[0]->intervalIndex << std::endl;
	std::cout << "function1->intervals[1]->intervalIndex: " << function1->intervals[1]->intervalIndex << std::endl;
	std::cout << std::endl;
	// Interpolated points for function with newly added interval
	function1->interpolateMult(0.0, 40.0);
	Point* points = new Point[function1->numInterpPointsPlot];
	for (int i = 0; i < function1->numInterpPointsPlot; i++) {
		points[i].x = function1->interpPoints[i][0];
		points[i].y = function1->interpPoints[i][1];
	}
	std::cout << "interpolateMult__TEST2" << std::endl;
	for (int i = 0; i < function1->numInterpPointsPlot; i++) {
		std::cout << "points[" << i << "]: ";
		points[i].console();
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl << std::endl;
	delete[] points;
}


// Tests copying function
void copy__TEST(Function* function1) {
	std::cout << "copy__TEST" << std::endl;
	std::cout << "function1: " << std::endl;
	function1->console();
	std::cout << std::endl;
	//Function function1_copy = new Function(new Trk(new IDFRSim()), 5);
	Function function1_copy(false, 0);
	function1_copy.copy(function1);
	std::cout << "function1_copy: " << std::endl;
	function1_copy.console();
	std::cout << std::endl;
	std::cout << "function1_copy.functionIndex: " << function1_copy.functionIndex << std::endl;
	std::cout << std::endl << std::endl << std::endl;
}

