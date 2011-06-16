// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Core.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace std;

template<typename T>
void MetricSpaceAligner<T>::DataPointsFilters::init()
{
	for (DataPointsFiltersIt it = this->begin(); it != this->end(); ++it)
	{
		(*it)->init();
	}
}

template<typename T>
void MetricSpaceAligner<T>::DataPointsFilters::apply(DataPoints& cloud, bool iterate)
{
	DataPoints filteredCloud;
	for (DataPointsFiltersIt it = this->begin(); it != this->end(); ++it)
	{
		const int pointsCount(cloud.features.cols());
		if (pointsCount == 0)
			throw ConvergenceError("no point to filter");
		
		filteredCloud = (*it)->filter(cloud, iterate);
		swapDataPoints<T>(cloud, filteredCloud);
	}
}

template<typename T>
void MetricSpaceAligner<T>::Transformations::apply(DataPoints& cloud, const TransformationParameters& parameters) const
{
	DataPoints transformedCloud;
	for (TransformationsConstIt it = this->begin(); it != this->end(); ++it)
	{
		transformedCloud = (*it)->compute(cloud, parameters);
		swapDataPoints<T>(cloud, transformedCloud);
	}
}

template<typename T>
void MetricSpaceAligner<T>::TransformationCheckers::init(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->init(parameters, iterate);
}

template<typename T>
void MetricSpaceAligner<T>::TransformationCheckers::check(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->check(parameters, iterate);
}

template<typename T> template<typename F>
typename MetricSpaceAligner<T>::OutlierWeights MetricSpaceAligner<T>::OutlierFilters<F>::compute(
	const typename MetricSpaceAligner<T>::DataPoints& filteredReading,
	const typename MetricSpaceAligner<T>::DataPoints& filteredReference,
	const typename MetricSpaceAligner<T>::Matches& input,
	bool& iterate) const
{
	if (this->empty())
	{
		// we do not have any filter, therefore we must put 0 weights for infinite distances
		OutlierWeights w(input.dists.rows(), input.dists.cols());
		for (int x = 0; x < w.cols(); ++x)
		{
			for (int y = 0; y < w.rows(); ++y)
			{
				if (input.dists(y, x) == numeric_limits<T>::infinity())
					w(y, x) = 0;
				else
					w(y, x) = 1;
			}
		}
		return w;
	}
	else
	{
		// apply filters, they should take care of infinite distances
		OutlierWeights w = (*this->begin())->compute(filteredReading, filteredReference, input, iterate);
		if (this->size() > 1)
		{
			for (typename Vector::const_iterator it = (this->begin() + 1); it != this->end(); ++it)
				w = w.array() * (*it)->compute(filteredReading, filteredReference, input, iterate).array();
		}
		return w;
	}
}


template<typename T>
MetricSpaceAligner<T>::ICPChainBase::ICPChainBase():
	matcher(0), 
	errorMinimizer(0),
	inspector(0),
	outlierMixingWeight(0.5)
{}

template<typename T>
MetricSpaceAligner<T>::ICPChainBase::~ICPChainBase()
{
	this->cleanup();
}

template<typename T>
void MetricSpaceAligner<T>::ICPChainBase::cleanup()
{
	for (DataPointsFiltersIt it = readingDataPointsFilters.begin(); it != readingDataPointsFilters.end(); ++it)
		delete *it;
	readingDataPointsFilters.clear();
	
	for (DataPointsFiltersIt it = readingStepDataPointsFilters.begin(); it != readingStepDataPointsFilters.end(); ++it)
		delete *it;
	readingStepDataPointsFilters.clear();
	
	for (DataPointsFiltersIt it = keyframeDataPointsFilters.begin(); it != keyframeDataPointsFilters.end(); ++it)
		delete *it;
	keyframeDataPointsFilters.clear();
	
	for (TransformationsIt it = transformations.begin(); it != transformations.end(); ++it)
		delete *it;
	transformations.clear();
	
	if (matcher)
		delete matcher;
	matcher = 0;
	
	for (FeatureOutlierFiltersIt it = featureOutlierFilters.begin(); it != featureOutlierFilters.end(); ++it)
		delete *it;
	featureOutlierFilters.clear();
	
	for (DescriptorOutlierFiltersIt it = descriptorOutlierFilters.begin(); it != descriptorOutlierFilters.end(); ++it)
		delete *it;
	descriptorOutlierFilters.clear();
	
	if (errorMinimizer)
		delete errorMinimizer;
	errorMinimizer = 0;
	
	for (TransformationCheckersIt it = transformationCheckers.begin(); it != transformationCheckers.end(); ++it)
		delete *it;
	transformationCheckers.clear();
	
	if (inspector)
		delete inspector;
	inspector = 0;
}

template<typename T>
void MetricSpaceAligner<T>::ICPChainBase::setDefault()
{
	this->cleanup();
	
	this->transformations.push_back(new TransformFeatures());
	this->readingDataPointsFilters.push_back(new RandomSamplingDataPointsFilter(0.5));
	this->keyframeDataPointsFilters.push_back(new SamplingSurfaceNormalDataPointsFilter(10, true, true, false, false, false));
	this->matcher = new KDTreeMatcher();
	this->featureOutlierFilters.push_back(new TrimmedDistOutlierFilter(0.85));
	this->errorMinimizer = new PointToPlaneErrorMinimizer();
	this->transformationCheckers.push_back(new CounterTransformationChecker(40));
	this->transformationCheckers.push_back(new ErrorTransformationChecker(0.001, 0.001, 3));
	
	this->inspector = new NullInspector;
	
	this->outlierMixingWeight = 1;
}



template<typename T>
MetricSpaceAligner<T>::ICP::ICP()
{
}

template<typename T>
MetricSpaceAligner<T>::ICP::~ICP()
{
}


template<typename T>
typename MetricSpaceAligner<T>::TransformationParameters MetricSpaceAligner<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn)
{
	const int dim = readingIn.features.rows();
	TransformationParameters identity = TransformationParameters::Identity(dim, dim);
	return this->compute(readingIn, referenceIn, identity);
}

template<typename T>
typename MetricSpaceAligner<T>::TransformationParameters MetricSpaceAligner<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& initialTransformationParameters)
{
	return this->compute(readingIn, referenceIn, initialTransformationParameters);
}

template<typename T>
typename MetricSpaceAligner<T>::TransformationParameters MetricSpaceAligner<T>::ICP::compute(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& initialTransformationParameters)
{
	assert(this->matcher);
	assert(this->errorMinimizer);
	assert(this->inspector);
	
	timer t; // Print how long take the algo
	const int dim = referenceIn.features.rows();
	
	// apply reference filters
	bool iterate(true);
	DataPoints reference(referenceIn);
	this->keyframeDataPointsFilters.init();
	this->keyframeDataPointsFilters.apply(reference, iterate);
	if (!iterate)
		return Matrix::Identity(dim, dim);
	
	// center reference point cloud, retrieve offset
	const int nbPtsReference = referenceIn.features.cols();
	const Vector meanReference = referenceIn.features.rowwise().sum() / nbPtsReference;
	for(int i=0; i < dim-1; i++)
		reference.features.row(i).array() -= meanReference(i);
	
	Matrix Tref(Matrix::Identity(dim, dim));
	Tref.block(0,dim-1, dim-1, 1) = meanReference.head(dim-1);
	this->matcher->init(reference, iterate);

	DataPoints reading(readingIn);
	const int nbPtsReading = reading.features.cols();
	this->readingDataPointsFilters.init();
	this->readingDataPointsFilters.apply(reading, iterate);
	
	this->readingStepDataPointsFilters.init();

	this->inspector->init();
	
	TransformationParameters transformationParameters = Tref.inverse() * initialTransformationParameters;
	
	this->transformationCheckers.init(transformationParameters, iterate);

	size_t iterationCount(0);
	
	cerr << "msa::icp - preprocess took " << t.elapsed() << " [s]" << endl;
	cerr << "msa::icp - nb points in reference: " << nbPtsReference << " -> " << reference.features.cols() << endl;
	cerr << "msa::icp - nb points in reading: " << nbPtsReading << " -> " << reading.features.cols() << endl;
	t.restart();
	
	while (iterate)
	{
		DataPoints stepReading(reading);
		
		//-----------------------------
		// Apply step filter
		this->readingStepDataPointsFilters.apply(stepReading, iterate);
		
		//-----------------------------
		// Transform Readings
		this->transformations.apply(stepReading, transformationParameters);
		
		//-----------------------------
		// Match to closest point in Reference
		const Matches matches(
			this->matcher->findClosests(stepReading, reference, iterate)
		);
		
		//-----------------------------
		// Detect outliers
		const OutlierWeights featureOutlierWeights(
			this->featureOutlierFilters.compute(stepReading, reference, matches, iterate)
		);
		
		const OutlierWeights descriptorOutlierWeights(
			this->descriptorOutlierFilters.compute(stepReading, reference, matches, iterate)
		);
		
		assert(featureOutlierWeights.rows() == matches.ids.rows());
		assert(featureOutlierWeights.cols() == matches.ids.cols());
		assert(descriptorOutlierWeights.rows() == matches.ids.rows());
		assert(descriptorOutlierWeights.cols() == matches.ids.cols());
		
		//cout << "featureOutlierWeights: " << featureOutlierWeights << "\n";
		//cout << "descriptorOutlierWeights: " << descriptorOutlierWeights << "\n";
		
		const OutlierWeights outlierWeights(
			featureOutlierWeights * this->outlierMixingWeight +
			descriptorOutlierWeights * (1 - this->outlierMixingWeight)
		);
		

		//-----------------------------
		// Dump
		this->inspector->dumpIteration(
			iterationCount, transformationParameters, reference, stepReading, matches, featureOutlierWeights, descriptorOutlierWeights, this->transformationCheckers
		);
		
		//-----------------------------
		// Error minimization
		transformationParameters *= this->errorMinimizer->compute(
			stepReading, reference, outlierWeights, matches, iterate
		);
		
		this->transformationCheckers.check(Tref * transformationParameters, iterate);
		
		++iterationCount;
	}
	
	this->inspector->finish(iterationCount);
	
	cerr << "msa::icp - " << iterationCount << " iterations took " << t.elapsed() << " [s]" << endl;
	
	// Move transformation back to original coordinate (without center of mass)
	return Tref * transformationParameters;
}

template<typename T>
MetricSpaceAligner<T>::ICPSequence::ICPSequence(const int dim, const std::string& filePrefix, const bool dumpStdErrOnExit):
	ratioToSwitchKeyframe(0.8),
	keyFrameDuration(16, "key_frame_duration", filePrefix, dumpStdErrOnExit),
	convergenceDuration(16, "convergence_duration", filePrefix, dumpStdErrOnExit),
	iterationsCount(16, "iterations_count", filePrefix, dumpStdErrOnExit),
	pointCountIn(16, "point_count_in", filePrefix, dumpStdErrOnExit),
	pointCountReading(16, "point_count_reading", filePrefix, dumpStdErrOnExit),
	pointCountKeyFrame(16, "point_count_key_frame", filePrefix, dumpStdErrOnExit),
	pointCountTouched(16, "point_count_touched", filePrefix, dumpStdErrOnExit),
	overlapRatio(16, "overlap_ratio", filePrefix, dumpStdErrOnExit),
	keyFrameCreated(false),
	keyFrameTransform(Matrix::Identity(dim+1, dim+1)),
	keyFrameTransformOffset(Matrix::Identity(dim+1, dim+1)),
	curTransform(Matrix::Identity(dim+1, dim+1)),
	lastTransformInv(Matrix::Identity(dim+1, dim+1))
{}

template<typename T>
MetricSpaceAligner<T>::ICPSequence::~ICPSequence()
{
}

template<typename T>
void MetricSpaceAligner<T>::ICPSequence::setDefault()
{
	ICPChainBase::setDefault();
	ratioToSwitchKeyframe = 0.8;
}

template<typename T>
void MetricSpaceAligner<T>::ICPSequence::resetTracking(DataPoints& inputCloud)
{
	const int tDim(keyFrameTransform.rows());
	createKeyFrame(inputCloud);
	keyFrameTransform = Matrix::Identity(tDim, tDim);
	lastTransformInv = Matrix::Identity(tDim, tDim);
}

template<typename T>
void MetricSpaceAligner<T>::ICPSequence::createKeyFrame(DataPoints& inputCloud)
{
	timer t; // Print how long take the algo
	t.restart();
	const int tDim(keyFrameTransform.rows());
	const int ptCount(inputCloud.features.cols());
	
	// apply filters
	bool iterate(true);
	this->keyframeDataPointsFilters.init();
	this->keyframeDataPointsFilters.apply(inputCloud, iterate);
	if (!iterate)
		return;
	
	pointCountKeyFrame.push_back(inputCloud.features.cols());
	
	// center keyframe, retrieve offset
	const int nbPtsKeyframe = inputCloud.features.cols();
	const Vector meanKeyframe = inputCloud.features.rowwise().sum() / nbPtsKeyframe;
	for(int i=0; i < tDim-1; i++)
		inputCloud.features.row(i).array() -= meanKeyframe(i);
		
	// update keyframe
	if (inputCloud.features.cols() > 0)
	{
		keyFrameCloud = inputCloud;
		keyFrameTransformOffset.block(0,tDim-1, tDim-1, 1) = meanKeyframe.head(tDim-1);
		curTransform = Matrix::Identity(tDim, tDim);
		
		this->matcher->init(keyFrameCloud, iterate);
		
		keyFrameCreated = true;
	
		keyFrameDuration.push_back(t.elapsed());
	}
	else
		cerr << "Warning: ignoring attempt to create a keyframe from an empty cloud (" << ptCount << " points before filtering)" << endl;
}

// WARNING: Reading and reference DataPoints will change!
// TODO: Put those constant??
template<typename T>
typename MetricSpaceAligner<T>::TransformationParameters MetricSpaceAligner<T>::ICPSequence::operator ()(
	const DataPoints& inputCloudIn)
{
	assert(this->matcher);
	assert(this->errorMinimizer);
	assert(this->inspector);
	
	lastTransformInv = getTransform().inverse();
	DataPoints inputCloud(inputCloudIn);


	// initial keyframe
	keyFrameCreated = false;
	if (keyFrameCloud.features.cols() == 0)
	{
		this->createKeyFrame(inputCloud);
		return curTransform;
	}
	
	timer t; // Print how long take the algo
	t.restart();

	////

	bool iterate(true);
	
	DataPoints reading(inputCloud);
	pointCountIn.push_back(inputCloud.features.cols());
	
	this->readingDataPointsFilters.init();
	this->readingDataPointsFilters.apply(reading, iterate);
	pointCountReading.push_back(reading.features.cols());
	
	this->readingStepDataPointsFilters.init();
	
	this->inspector->init();
	
	TransformationParameters transformationParameters = keyFrameTransformOffset.inverse() * curTransform;
	
	this->transformationCheckers.init(transformationParameters, iterate);

	size_t iterationCount(0);
	
	while (iterate)
	{
		DataPoints stepReading(reading);
		
		//-----------------------------
		// Apply step filter
		this->readingStepDataPointsFilters.apply(stepReading, iterate);
		
		//-----------------------------
		// Transform Readings
		this->transformations.apply(stepReading, transformationParameters);
		
		//-----------------------------
		// Match to closest point in Reference
		const Matches matches(
			this->matcher->findClosests(stepReading, keyFrameCloud, iterate)
		);
		
		//-----------------------------
		// Detect outliers
		const OutlierWeights featureOutlierWeights(
			this->featureOutlierFilters.compute(stepReading, keyFrameCloud, matches, iterate)
		);
		
		const OutlierWeights descriptorOutlierWeights(
			this->descriptorOutlierFilters.compute(stepReading, keyFrameCloud, matches, iterate)
		);
		
		assert(featureOutlierWeights.rows() == matches.ids.rows());
		assert(featureOutlierWeights.cols() == matches.ids.cols());
		assert(descriptorOutlierWeights.rows() == matches.ids.rows());
		assert(descriptorOutlierWeights.cols() == matches.ids.cols());
		
		//cout << "featureOutlierWeights: " << featureOutlierWeights << "\n";
		//cout << "descriptorOutlierWeights: " << descriptorOutlierWeights << "\n";
		
		const OutlierWeights outlierWeights(
			featureOutlierWeights * this->outlierMixingWeight +
			descriptorOutlierWeights * (1 - this->outlierMixingWeight)
		);

		//-----------------------------
		// Dump
		this->inspector->dumpIteration(
			iterationCount, transformationParameters, keyFrameCloud, stepReading, matches, featureOutlierWeights, descriptorOutlierWeights, this->transformationCheckers
		);
		
		//-----------------------------
		// Error minimization
		transformationParameters *= this->errorMinimizer->compute(
			stepReading, keyFrameCloud, outlierWeights, matches, iterate
		);
		
		this->transformationCheckers.check(keyFrameTransformOffset * transformationParameters, iterate);
		
		++iterationCount;
	}
	iterationsCount.push_back(iterationCount);
	pointCountTouched.push_back(this->matcher->getVisitCount());
	this->matcher->resetVisitCount();
	this->inspector->finish(iterationCount);
	
	// Move transformation back to original coordinate (without center of mass)
	curTransform = keyFrameTransformOffset * transformationParameters;
	
	convergenceDuration.push_back(t.elapsed());
	overlapRatio.push_back(this->errorMinimizer->getWeightedPointUsedRatio());
	
	if (this->errorMinimizer->getWeightedPointUsedRatio() < ratioToSwitchKeyframe)
	{
		// new keyframe
		keyFrameTransform *= curTransform;
		this->createKeyFrame(inputCloud);
	}
	
	/*transDriftX.push_back(curTransform(0, 3));
	transDriftY.push_back(curTransform(1, 3));
	transDriftZ.push_back(curTransform(2, 3));
	
	const double pitch = -asin(curTransform(2,0));
	const double roll = atan2(curTransform(2,1), curTransform(2,2));
	const double yaw = atan2(curTransform(1,0) / cos(pitch), curTransform(0,0) / cos(pitch));
	rotDriftRoll.push_back(roll);
	rotDriftPitch.push_back(pitch);
	rotDriftYaw.push_back(yaw);
	*/
	
	// Return transform in world space
	return keyFrameTransform * curTransform;
}

template struct MetricSpaceAligner<float>;
template struct MetricSpaceAligner<double>;

