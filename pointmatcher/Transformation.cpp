// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "Core.h"
#include <iostream>

// TransformFeatures
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::TransformFeatures::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename DataPoints::Features Features;
	
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());
	
	DataPoints transformedDataPoints(
		Features(input.features.rows(), input.features.cols()),
		input.featureLabels,
		input.descriptors,
		input.descriptorLabels
	);
	Features& transformedFeatures(transformedDataPoints.features);
	
	for (int i = 0; i < transformedFeatures.cols(); ++i)
	{
		transformedFeatures.col(i) = parameters * input.features.col(i);
	}
	
	return transformedDataPoints;
}

template struct MetricSpaceAligner<float>::TransformFeatures;
template struct MetricSpaceAligner<double>::TransformFeatures;


// TransformDescriptors
template<typename T>
typename MetricSpaceAligner<T>::DataPoints MetricSpaceAligner<T>::TransformDescriptors::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	typedef typename DataPoints::Descriptors Descriptors;
	assert(parameters.rows() == parameters.cols());

	DataPoints transformedDataPoints(input);

	const int ptCount(input.descriptors.cols());

	//NOTE: Only need rotation for descriptors (up to now...)
	const Matrix R(parameters.corner(Eigen::TopLeft, 
			parameters.rows()-1, parameters.cols()-1));

	int descRow(0);
	for(int unsigned i = 0; i < transformedDataPoints.descriptorLabels.size(); i++)
	{
		int span(input.descriptorLabels[i].span);
		if(transformedDataPoints.descriptorLabels[i].text.compare("normals") == 0)
		{
			assert(span == parameters.rows()-1);
			transformedDataPoints.descriptors.block(descRow, 0, span, ptCount) = 
				R * input.descriptors.block(descRow, 0, span, ptCount);
		}
		descRow += input.descriptorLabels[i].span;
	}
	
	return transformedDataPoints;
}

template struct MetricSpaceAligner<float>::TransformDescriptors;
template struct MetricSpaceAligner<double>::TransformDescriptors;

