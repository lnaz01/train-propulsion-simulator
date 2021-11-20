//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the TRAIN PROPULSION SIMULATOR repository available at github.com/lnaz01/train-propulsion-simulator //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INTERVALSTEP_DEF
#define INTERVALSTEP_DEF

#include "Interval.h"

class Function;
class Point;
class UserDefinedRRComponent;

class IntervalStep : public Interval {

public:

	IntervalStep(int intervalIndex);

	virtual ~IntervalStep();

	void add_point(Point* point) override;

	double interpolate(double xpt, double miny, double maxy) override;

	std::string _load(std::string str, UserDefinedRRComponent* udrrc, Function* function) override;

	void zap() override;

};

#endif
