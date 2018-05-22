#pragma once

#include "PointMatcher.h"

//! Systematic sampling, with variation over time
template<typename T>
struct FixStepSamplingDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	
	inline static const std::string description()
	{
		return "Subsampling. This filter reduces the size of the point cloud by only keeping one point over step ones; with step varying in time from startStep to endStep, each iteration getting multiplied by stepMult. If use as prefilter (i.e. before the iterations), only startStep is used.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
			( "startStep", "initial number of point to skip (initial decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> )
			( "endStep", "maximal or minimal number of points to skip (final decimation factor)", "10", "1", "2147483647", &P::Comp<unsigned> )
			( "stepMult", "multiplication factor to compute the new decimation factor for each iteration", "1", "0.0000001", "inf", &P::Comp<double> )
		;
	}
	
	// number of steps to skip
	const unsigned startStep;
	const unsigned endStep;
	const double stepMult;

protected:
	double step;
	
public:
	FixStepSamplingDataPointsFilter(const Parameters& params = Parameters());
	virtual ~FixStepSamplingDataPointsFilter() {};
	virtual void init();
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};
