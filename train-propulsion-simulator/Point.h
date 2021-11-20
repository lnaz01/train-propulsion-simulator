//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POINT_DEF
#define POINT_DEF

#include <string>

class Function;

class Point {

public:

	Point();

	// x		-->	Independent variable
	// y		-->	Dependent variable
	Point(double x, double y);

	virtual ~Point();

	// Independent variable
	double x;

	// Dependent variable
	double y;

	// Copies point
	void copy(Point* point);

	// Checks if string represents a valid point and loads string data into point (if point is valid)
	// str			-->	String of data
	// function		-->	Function which contains point
	std::string load(std::string str, Function* function);

	// Prints to console
	void console();

};

#endif
