//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FUNCTION_DEF
#define FUNCTION_DEF

#include <string>
#include <vector>

class InputFileReader;
class Interval;
class UserDefinedRRComponent;

class Function {

public:

	// userDefinedRRComponent		-->	User-defined railroad component
	// functionIndex				-->	Function index
	Function(UserDefinedRRComponent* userDefinedRRComponent, int functionIndex);

	// stepFunctionBool					-->	Step function boolean ('true' for step function)
	// maxNumberOfPointsPerInterval		-->	Maximum number of points per interval (only used if first parameter 'sfb' is false)
	Function(bool stepFunctionBool, int maxNumberOfPointsPerInterval);

	virtual ~Function();

	// Start function string
	static const std::string START_FUNCTION_STRING;

	// End function string
	static const std::string END_FUNCTION_STRING;

	// Maximum number of intervals per function
	static constexpr int MAX_NUM_INTERVALS = 2000;

	// Step function boolean ('true' for step function)
	bool stepFunctionBool;

	// Function index
	int functionIndex;

	// Intervals
	std::vector<Interval*> intervals;

	// Interpolated points
	double** interpPoints;

	// Number of spline interpolated points for plotting
	int numInterpPointsPlot;

	// Adds interval
	void add_interval();

	// Defines piecewise linear function
	// x		-->	Independent variable data
	// y		-->	Dependent variable data
	// xySize	-->	Size of data
	void piecewiseLinear(double* x, double* y, int xySize);

	// Calculates single spline-interpolated value of a data point
	// xpt		-->	Independent variable value for which the spline-interpolated value is desired
	// miny		-->	Minimum threshold for dependent variable
	// maxy		-->	Maximum threshold for dependent variable
	double interpolate(double xpt, double miny, double maxy);

	// Calculates multiple interpolated values for independent variable 'x' and dependent variable 'y'
	// miny		-->	Minimum threshold for dependent variable
	// maxy		-->	Maximum threshold for dependent variable
	void interpolateMult(double miny, double maxy);

	// Copies function
	// function		-->	Function to be copied
	void copy(Function* function);

	// Clears function
	void zap();

	// Checks if inputted text data represents a valid function and loads inputted text data into function (if function is valid)
	std::string load(InputFileReader* inputFileReader, UserDefinedRRComponent* udrrc, int functionIndex);

	// Prints to console
	void console();

private:

	// Maximum number of points per interval
	int maxNumberOfPointsPerInterval;

	// Interpolated points calculated boolean
	bool interpPointsCalcBool;

};

#endif
