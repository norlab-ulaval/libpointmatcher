#include "FixStepSampling.h"

#include "PointMatcherPrivate.h"


// FixStepSamplingDataPointsFilter
// Constructor
template<typename T>
FixStepSamplingDataPointsFilter<T>::FixStepSamplingDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("FixStepSamplingDataPointsFilter", 
		FixStepSamplingDataPointsFilter::availableParameters(), params),
	startStep(Parametrizable::get<unsigned>("startStep")),
	endStep(Parametrizable::get<unsigned>("endStep")),
	stepMult(Parametrizable::get<double>("stepMult")),
	step(startStep)
{
	LOG_INFO_STREAM("Using FixStepSamplingDataPointsFilter with startStep=" << startStep << ", endStep=" << endStep << ", stepMult=" << stepMult);
}


template<typename T>
void FixStepSamplingDataPointsFilter<T>::init()
{
	step = startStep;
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
FixStepSamplingDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void FixStepSamplingDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const int iStep(step);
	const int nbPointsIn = cloud.features.cols();
	const int phase(rand() % iStep);

	int j = 0;
	for (int i = phase; i < nbPointsIn; i += iStep)
	{
		cloud.setColFrom(j, cloud, i);
		++j;
	}

	cloud.conservativeResize(j);

	const double deltaStep(startStep * stepMult - startStep);
	step *= stepMult;
	if (deltaStep < 0 && step < endStep)
		step = endStep;
	if (deltaStep > 0 && step > endStep)
		step = endStep;

}

template struct FixStepSamplingDataPointsFilter<float>;
template struct FixStepSamplingDataPointsFilter<double>;

