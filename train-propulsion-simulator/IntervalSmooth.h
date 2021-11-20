//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INTERVALSMOOTH_DEF
#define INTERVALSMOOTH_DEF

#include <string>
#include "Interval.h"

class Function;
class LinearSystem;
class Point;
class UserDefinedRRComponent;

class IntervalSmooth : public Interval {

public:

	IntervalSmooth(int intervalIndex, int maximumNumberOfPoints);

	virtual ~IntervalSmooth();

	// Maximum number of points per smooth interval
	static const int ABSOLUTE_MAX_NUMBER_OF_POINTS = 30;

	// Intermediate 'z' values calculated boolean
	bool zvcb;

	void add_point(Point* point) override;

	double interpolate(double xpt, double miny, double maxy) override;

	std::string _load(std::string str, UserDefinedRRComponent* udrrc, Function* function) override;

	void zap() override;

private:

	// Linear system
	LinearSystem* linearSystem;

	// Independent variable step size array
	double* h;

	// Intermediate 'z' values used in 'YSplineInterpolation' method
	double* z;

	// Calculates array of doubles 'z' used in the 'Spline' method
	void calc_z();

	// Calculates independent variable 'h' step sizes.  Element at index '0' (first element) is a 'placeholder' and is set equal to zero.
	void calc_h();

};

#endif
