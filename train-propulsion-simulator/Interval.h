//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INTERVAL_DEF
#define INTERVAL_DEF

#include <string>
#include <vector>
#include "Point.h"

class Function;
class Point;
class UserDefinedRRComponent;

class Interval {

public:

	Interval(int intervalIndex);

	virtual ~Interval();

	// Minimum number of points
	static constexpr int MIN_NUMBER_OF_POINTS = 2;

	// Adds point
	virtual void add_point(Point* point) = 0;

	// Calculates single spline-interpolated value of a data point
	// xpt		-->	Independent variable value for which the spline-interpolated value is desired
	// miny		-->	Minimum threshold for dependent variable
	// maxy		-->	Maximum threshold for dependent variable
	virtual double interpolate(double xpt, double miny, double maxy) = 0;

	// Checks if string represents a valid interval and loads string data into interval (if point is valid)
	virtual std::string _load(std::string str, UserDefinedRRComponent* udrrc, Function* function) = 0;

	// Clears interval
	virtual void zap() = 0;

	// Points
	std::vector<Point*> points;

	// Interval index
	int intervalIndex;

	// Number of spline points for plotting
	int numInterpPointsPlot;

	// Interpolated points
	double** interpPoints;

	// Calculates multiple interpolated values for independent variable 'x' and dependent variable 'y'
	// miny		-->	Minimum threshold for dependent variable
	// maxy		-->	Maximum threshold for dependent variable
	void interpolateMult(double miny, double maxy);

	// Copies interval
	void copy(Interval* interval);

	// Checks if string represents a valid interval and loads string data into interval (if point is valid)
	std::string load(std::string str, UserDefinedRRComponent* udrrc, Function* function);

	// Prints to console
	void console();

protected:

	// Maximum number of points
	int maxNumberOfPoints;

	// Interpolated points calculated boolean
	bool interpPointsCalcBool;

};

#endif
