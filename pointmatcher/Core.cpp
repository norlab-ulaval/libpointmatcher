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
#include <boost/format.hpp> 

using namespace std;

/*
struct __toto
{
	__toto()
	{
		typedef PointMatcher<float>::Parametrizable P;
		P p(
			"test",
			"this is a test",
			{ { "val0", "test val0", 1}, { "val1", "test val1", "string" } },
			P::Parameters()
		);
		cout << p << endl;
	}
};

static __toto __toto_instance;*/

template<typename T> template<typename S>
PointMatcher<T>::Parametrizable::ParameterDoc::ParameterDoc(const std::string name, const std::string doc, const S defaultValue):
	name(name),
	doc(doc),
	defaultValue(boost::lexical_cast<string>(defaultValue))
{}

template<typename T>
PointMatcher<T>::Parametrizable::Parametrizable(
	const std::string& name,
	const std::string& doc,
	std::initializer_list<ParameterDoc> paramsDoc,
	const Parameters& params):
	name(name),
	doc(doc),
	parametersDoc(paramsDoc)
{
	// fill current parameters from either values passed as argument, or default value
	for (auto it = parametersDoc.cbegin(); it != parametersDoc.cend(); ++it)
	{
		const string& paramName(it->name);
		Parameters::const_iterator paramIt(params.find(paramName));
		if (paramIt != params.end())
			parameters[paramName] = paramIt->second;
		else
			parameters[paramName] = it->defaultValue;
	}
}

template<typename T>
void PointMatcher<T>::Parametrizable::dump(std::ostream& o) const
{
	o << name << " - " << doc << endl;
	for (auto it = parametersDoc.cbegin(); it != parametersDoc.cend(); ++it)
		o << it->name << " (" << it->defaultValue << ") - " << it->doc << endl;
}

template<typename T>
std::string PointMatcher<T>::Parametrizable::getParam(const std::string& name) const
{
	Parameters::const_iterator paramIt(parameters.find(name));
	if (paramIt == parameters.end())
		throw Error((boost::format("Parameter %1 does not exist in object %2") % name % this->name).str());
	// TODO: use string distance to propose close one, copy/paste code from Aseba
	return paramIt->second;
}

// DataPoints

template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Features& features, const Labels& featureLabels):
	features(features),
	featureLabels(featureLabels)
{}

template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Features& features, const Labels& featureLabels, const Descriptors& descriptors, const Labels& descriptorLabels):
	features(features),
	featureLabels(featureLabels),
	descriptors(descriptors),
	descriptorLabels(descriptorLabels)
{}

template<typename T>
typename PointMatcher<T>::DataPoints::Descriptors PointMatcher<T>::DataPoints::getDescriptorByName(const std::string& name) const
{
	int row(0);
	
	for(unsigned int i = 0; i < descriptorLabels.size(); i++)
	{
		const int span(descriptorLabels[i].span);
		if(descriptorLabels[i].text.compare(name) == 0)
		{
			return descriptors.block(row, 0, 
					span, descriptors.cols());
		}

		row += span;
	}

	return Descriptors();
}

// Matches
template<typename T>
PointMatcher<T>::Matches::Matches(const Dists& dists, const Ids ids):
	dists(dists),
	ids(ids)
{}

template<typename T>
T PointMatcher<T>::Matches::getDistsQuantile(const T quantile) const
{
	// TODO: check alignment and use matrix underlying storage when available
	// build array
	vector<T> values;
	values.reserve(dists.rows() * dists.cols());
	for (int x = 0; x < dists.cols(); ++x)
		for (int y = 0; y < dists.rows(); ++y)
			if ((dists(y, x) != numeric_limits<T>::infinity()) && (dists(y, x) > 0))
				values.push_back(dists(y, x));
	if (values.size() == 0)
		throw ConvergenceError("no outlier to filter");
	
	// get quantile
	nth_element(values.begin(), values.begin() + (values.size() * quantile), values.end());
	return values[values.size() * quantile];
}

template<typename T>
void PointMatcher<T>::DataPointsFilters::init()
{
	for (DataPointsFiltersIt it = this->begin(); it != this->end(); ++it)
	{
		(*it)->init();
	}
}

template<typename T>
void PointMatcher<T>::DataPointsFilters::apply(DataPoints& cloud, bool iterate)
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
void PointMatcher<T>::Transformations::apply(DataPoints& cloud, const TransformationParameters& parameters) const
{
	DataPoints transformedCloud;
	for (TransformationsConstIt it = this->begin(); it != this->end(); ++it)
	{
		transformedCloud = (*it)->compute(cloud, parameters);
		swapDataPoints<T>(cloud, transformedCloud);
	}
}

template<typename T>
void PointMatcher<T>::TransformationCheckers::init(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->init(parameters, iterate);
}

template<typename T>
void PointMatcher<T>::TransformationCheckers::check(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->check(parameters, iterate);
}

template<typename T> template<typename F>
typename PointMatcher<T>::OutlierWeights PointMatcher<T>::OutlierFilters<F>::compute(
	const typename PointMatcher<T>::DataPoints& filteredReading,
	const typename PointMatcher<T>::DataPoints& filteredReference,
	const typename PointMatcher<T>::Matches& input,
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
PointMatcher<T>::ICPChainBase::ICPChainBase():
	matcher(0), 
	errorMinimizer(0),
	inspector(0),
	outlierMixingWeight(0.5)
{}

template<typename T>
PointMatcher<T>::ICPChainBase::~ICPChainBase()
{
	this->cleanup();
}

template<typename T>
void PointMatcher<T>::ICPChainBase::cleanup()
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
void PointMatcher<T>::ICPChainBase::setDefault()
{
	this->cleanup();
	
	this->transformations.push_back(new TransformFeatures());
	
	this->readingDataPointsFilters.push_back(new RandomSamplingDataPointsFilter(0.5));
	
	this->keyframeDataPointsFilters.push_back(new SamplingSurfaceNormalDataPointsFilter(10, true, true, false, false, false));
	
	this->featureOutlierFilters.push_back(new TrimmedDistOutlierFilter(0.75));
	
	this->matcher = new KDTreeMatcher();
	
	this->errorMinimizer = new PointToPlaneErrorMinimizer();
	
	this->transformationCheckers.push_back(new CounterTransformationChecker(100));
	this->transformationCheckers.push_back(new ErrorTransformationChecker(0.001, 0.001, 3));
	
	this->inspector = new NullInspector;
	
	this->outlierMixingWeight = 1;
}



template<typename T>
PointMatcher<T>::ICP::ICP()
{
}

template<typename T>
PointMatcher<T>::ICP::~ICP()
{
}


template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn)
{
	const int dim = readingIn.features.rows();
	TransformationParameters identity = TransformationParameters::Identity(dim, dim);
	return this->compute(readingIn, referenceIn, identity);
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& initialTransformationParameters)
{
	return this->compute(readingIn, referenceIn, initialTransformationParameters);
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::compute(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& T_refIn_dataIn)
{
	assert(this->matcher);
	assert(this->errorMinimizer);
	assert(this->inspector);
	
	timer t; // Print how long take the algo
	const int dim = referenceIn.features.rows();
	bool iterate(true);
	
	// Apply reference filters
	// reference is express in frame <refIn>
	DataPoints reference(referenceIn);
	this->keyframeDataPointsFilters.init();
	this->keyframeDataPointsFilters.apply(reference, iterate);
	if (!iterate)
		return Matrix::Identity(dim, dim);
	
	// Create intermediate frame at the center of mass of reference pts cloud
	//  this help to solve for rotations
	const int nbPtsReference = referenceIn.features.cols();
	const Vector meanReference = referenceIn.features.rowwise().sum() / nbPtsReference;
	TransformationParameters T_refIn_refMean(Matrix::Identity(dim, dim));
	T_refIn_refMean.block(0,dim-1, dim-1, 1) = meanReference.head(dim-1);
	
	// Reajust reference position: 
	// from here reference is express in frame <refMean>
	// Shortcut to do T_refIn_refMean.inverse() * reference
	for(int i=0; i < dim-1; i++)
		reference.features.row(i).array() -= meanReference(i);
	
	// Init matcher with reference points center on its mean
	this->matcher->init(reference, iterate);

	// Apply readings filters
	// reading is express in frame <dataIn>
	DataPoints reading(readingIn);
	const int nbPtsReading = reading.features.cols();
	this->readingDataPointsFilters.init();
	this->readingDataPointsFilters.apply(reading, iterate);
	
	// Reajust reading position: 
	// from here reading is express in frame <refMean>
	TransformationParameters 
		T_refMean_dataIn = T_refIn_refMean.inverse() * T_refIn_dataIn;
	this->transformations.apply(reading, T_refMean_dataIn);
	
	// Prepare reading filters used in the loop 
	this->readingStepDataPointsFilters.init();

	this->inspector->init();
	
	// Since reading and reference are express in <refMean>
	// the frame <refMean> is equivalent to the frame <iter(0)>
	TransformationParameters T_iter = Matrix::Identity(dim, dim);
	
	this->transformationCheckers.init(T_iter, iterate);

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
		this->transformations.apply(stepReading, T_iter);
		
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
			iterationCount, T_iter, reference, stepReading, matches, featureOutlierWeights, descriptorOutlierWeights, this->transformationCheckers
		);
		
		//-----------------------------
		// Error minimization
		// equivalent to: 
		//   T_iter(i+1)_iter(0) = T_iter(i+1)_iter(i) * T_iter(i)_iter(0)
		T_iter = this->errorMinimizer->compute(
			stepReading, reference, outlierWeights, matches, iterate
		) * T_iter;
		
		// Old version
		//T_iter = T_iter * this->errorMinimizer->compute(
		//	stepReading, reference, outlierWeights, matches, iterate
		//);
		
		// in test
		
		this->transformationCheckers.check(T_iter, iterate);
		
		++iterationCount;
	}
	
	this->inspector->finish(iterationCount);
	
	cerr << "msa::icp - " << iterationCount << " iterations took " << t.elapsed() << " [s]" << endl;
	
	// Move transformation back to original coordinate (without center of mass)
	// T_iter is equivalent to: T_iter(i+1)_iter(0)
	// the frame <iter(0)> equals <refMean>
	// so we have: 
	//   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_refMean_dataIn
	//   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_iter(0)_dataIn
	// T_refIn_refMean remove the temperary frame added during initialization
	return (T_refIn_refMean * T_iter * T_refMean_dataIn);
}

template<typename T>
PointMatcher<T>::ICPSequence::ICPSequence(const int dim, const std::string& filePrefix, const bool dumpStdErrOnExit):
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
PointMatcher<T>::ICPSequence::~ICPSequence()
{
}

template<typename T>
void PointMatcher<T>::ICPSequence::setDefault()
{
	ICPChainBase::setDefault();
	ratioToSwitchKeyframe = 0.8;
}

template<typename T>
void PointMatcher<T>::ICPSequence::resetTracking(DataPoints& inputCloud)
{
	const int tDim(keyFrameTransform.rows());
	createKeyFrame(inputCloud);
	keyFrameTransform = Matrix::Identity(tDim, tDim);
	lastTransformInv = Matrix::Identity(tDim, tDim);
}

template<typename T>
void PointMatcher<T>::ICPSequence::createKeyFrame(DataPoints& inputCloud)
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
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::operator ()(
	const DataPoints& inputCloudIn)
{
	assert(this->matcher);
	assert(this->errorMinimizer);
	assert(this->inspector);
	
	lastTransformInv = getTransform().inverse();
	DataPoints inputCloud(inputCloudIn);


	// initial keyframe
	keyFrameCreated = false;
	if (!hasKeyFrame())
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
		
		this->transformationCheckers.check(transformationParameters, iterate);
		
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

template struct PointMatcher<float>;
template struct PointMatcher<double>;

