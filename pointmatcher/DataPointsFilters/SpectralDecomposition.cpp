// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
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
#include "SpectralDecomposition.h"

#include <random>

// SpectralDecomposition
template <typename T>
SpectralDecompositionDataPointsFilter<T>::SpectralDecompositionDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("SpectralDecompositionDataPointsFilter", 
		SpectralDecompositionDataPointsFilter::availableParameters(), params),
	k{Parametrizable::get<std::size_t>("k")},
	sigma{Parametrizable::get<T>("sigma")},
	radius{Parametrizable::get<T>("radius")},
	itMax{Parametrizable::get<std::size_t>("itMax")},
	keepNormals{Parametrizable::get<bool>("keepNormals")},
	keepLabels{Parametrizable::get<bool>("keepLabels")},
	keepLambdas{Parametrizable::get<bool>("keepLambdas")},
	keepTensors{Parametrizable::get<bool>("keepTensors")}
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
SpectralDecompositionDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void SpectralDecompositionDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const std::size_t nbPts = cloud.getNbPoints();
	
	TensorVoting<T> tv{sigma, k};
	
	//Refinement step
	tv.refine(cloud);
	
	//Compute real saliencies
	tv.disableBallComponent();
	tv.cfvote(cloud, false);
	tv.decompose();
	tv.toDescriptors();
#if 0	
	const T thresh_pointness = 5. * xi_expectation(3, sigma, radius) / 6.;
	const T thresh_curveness = xi_expectation(1, sigma, radius / 2.) / 2.;
	const T thresh_surfaceness = xi_expectation(2, sigma, radius) / 4.; 
#endif	
//--- 1. Vote to determine prefered orientation + density estimation -----------
	tv.refine(cloud);
	addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, true /*lambdas*/, false /*tensors*/);
	
//--- 2. Filter iteratively on each measure (surfaceness, curveness, pointness)
	std::size_t it = 0;
	const std::size_t itMax_ = itMax;
	std::size_t oldnbPts = nbPts;
	
	auto checkConvergence = [&oldnbPts, &it, &itMax_](const DataPoints& pts, const std::size_t threshold) mutable ->bool{		
		const std::size_t nbPts = pts.getNbPoints();
		bool ret = (oldnbPts - nbPts) < threshold;
		
		//std::cout << "["<<int(ret or (it+1) >= itMax_)<<"] oldnbPts ("<<oldnbPts<<") - nbPts ("<<nbPts<<") -> "<<int(ret) << " -- Iteration "<<it+1<<" / "<<itMax_ << std::endl;
		
		oldnbPts = nbPts;
		
		return ret or ++it >= itMax_;		
	};
	
	do 
	{
		//std::cout << "Iteration : " << it << "("<< cloud.getNbPoints() << ")" << std::endl;	
#if 0
	// 2.1 On pointness
		reduce(cloud, thresh_pointness, "pointness");
	// 2.2 On curveness
		reduce(cloud, thresh_curveness, "curveness");
	// 2.3 On surfaceness
		reduce(cloud, thresh_surfaceness, "surfaceness");
#else
	//std::cout << "Filtering..."<< std::endl;
	// 2.1 On pointness
		filterPointness(cloud, xi_expectation(3, sigma, radius), k);
	// 2.2 On curveness
		filterCurveness(cloud, xi_expectation(1, sigma, radius / 2.), k);
	// 2.3 On surfaceness
		filterSurfaceness(cloud, xi_expectation(2, sigma, radius), k);
#endif
	//Re-compute vote...
		//std::cout << "Recompute vote..."<< std::endl;
		tv.refine(cloud);
		//std::cout << "Add descriptors..."<< std::endl;
		addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, true /*lambdas*/, false /*tensors*/);
	} 
	while(not checkConvergence(cloud, 15 /*delta points*/));
	
//--- 3. Re-encode as Aware tensors + Re-vote ----------------------------------
	//std::cout << "Re-encode as Aware tensors + Re-vote"<< std::endl;
	addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, false /*lambdas*/, true /*tensors*/);
	tv.encode(cloud, TensorVoting<T>::Encoding::AWARE_TENSOR);
	tv.cfvote(cloud);
	tv.decompose();
	tv.toDescriptors();

//--- 4. Add descriptors
	addDescriptor(cloud, tv, keepNormals, keepLabels, keepLambdas, keepTensors);
}


template <typename T>
void SpectralDecompositionDataPointsFilter<T>::addDescriptor(DataPoints& pts, const TensorVoting<T> &tv, bool keepNormals_, bool keepLabels_, bool keepLambdas_, bool keepTensors_) const
{
	const std::size_t nbPts = pts.getNbPoints();

	Matrix labels  = Matrix::Zero(1, nbPts);
	Matrix l1  = PM::Matrix::Zero(1, nbPts);	
	Matrix l2  = PM::Matrix::Zero(1, nbPts);	
	Matrix l3  = PM::Matrix::Zero(1, nbPts);
	
	if(keepLabels_ or keepLambdas_)
	{
	#pragma omp parallel for
		for(std::size_t i = 0; i < nbPts; ++i)
		{
			const T lambda1 = tv.surfaceness(i) + tv.curveness(i) + tv.pointness(i);
			const T lambda2 = tv.curveness(i) + tv.pointness(i);
			const T lambda3 = tv.pointness(i);
	
			int index;
			Vector coeff = (Vector(3) << lambda3, (lambda2 - lambda3), (lambda1 - lambda2)).finished();
			coeff.maxCoeff(&index);  

			labels(i) = index + 1 ;
		
			l1(i) = lambda1 * k;
			l2(i) = lambda2 * k;
			l3(i) = lambda3 * k;
		}
	}
	try
	{	
	    pts.addDescriptor("surfaceness", tv.surfaceness);
	    pts.addDescriptor("curveness", tv.curveness);
	    pts.addDescriptor("pointness", tv.pointness);
	    
	    if(keepLambdas_)
	    {
			pts.addDescriptor("lambda1", l1);
			pts.addDescriptor("lambda2", l2);
			pts.addDescriptor("lambda3", l3);
	    }
	    
		if(keepNormals_)
		{    
			pts.addDescriptor("normals", tv.normals);
			pts.addDescriptor("tangents", tv.tangents);
		}    
	    if(keepLabels_)
	    {
			pts.addDescriptor("labels", labels);
	    }  
	    if(keepTensors_)
	    {
			pts.addDescriptor("sticks", tv.sticks);
			pts.addDescriptor("plates", tv.plates);
			pts.addDescriptor("balls", tv.balls);
	    }
	}
	catch (...) {
	    std::cerr << "SpectralDecomposition<T>::inPlaceFilter::addDescriptor: Cannot add descriptors to pointcloud" << std::endl;
	}

}

//------------------------------------------------------------------------------
// Filter
//------------------------------------------------------------------------------
template <typename T>
void SpectralDecompositionDataPointsFilter<T>::reduce(DataPoints& pts, T threshold, std::string descName) const
{
#if 1
	constexpr std::size_t seed = 1;
	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
	std::uniform_real_distribution<> uni01(0., 1.);
	
	const std::size_t nbPts = pts.getNbPoints();
	
	// Check field exists
	if (!pts.descriptorExists(descName))
	{
		throw InvalidField("SpectralDecomposition<T>::filter: Error, field not found in descriptors.");
	}
	
	const auto& values = pts.getDescriptorViewByName(descName);

	std::size_t j = 0;
	for (std::size_t i = 0; i < nbPts; ++i)
	{
		const T randv = uni01(gen);	
		const T value(values(0,i));
		
		if (value < threshold or randv < 0.2)
		{
			pts.setColFrom(j, pts, i);
			++j;
		}
	}
	pts.conservativeResize(j);	
#else
	PointMatcherSupport::Parametrizable::Parameters params; 
		params["descName"] = descName;
		params["useLargerThan"] = "1";
		params["threshold"] = std::to_string(threshold);
		
	PM::DataPointsFilter* trimDesc = 
		PM::get().DataPointsFilterRegistrar.create("CutAtDescriptorThresholdDataPointsFilter", params);

	trimDesc->inPlaceFilter(pts);
#endif
}

template <typename T>
void SpectralDecompositionDataPointsFilter<T>::filterSurfaceness(DataPoints& pts, T xi, std::size_t k) const
{
	constexpr std::size_t seed = 1;
	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
	std::uniform_real_distribution<> uni01(0., 1.);
	
	const std::size_t nbPts = pts.getNbPoints();
	
	// Check field exists
	if (!pts.descriptorExists("lambda1") or !pts.descriptorExists("lambda2") or !pts.descriptorExists("lambda3"))
	{
		throw InvalidField("SpectralDecomposition<T>::filter: Error, lambdas field not found in descriptors.");
	}
	
	const auto& lambda1 = pts.getDescriptorViewByName("lambda1");
	const auto& lambda2 = pts.getDescriptorViewByName("lambda2");
	const auto& lambda3 = pts.getDescriptorViewByName("lambda3");

	std::size_t j = 0;
	for (std::size_t i = 0; i < nbPts; ++i)
	{
		const T randv = uni01(gen);	
		
		const T nl1 = lambda1(0,i) / k;
		const T nl2 = lambda2(0,i) / k;
		const T nl3 = lambda3(0,i) / k;
		
		if (nl1 < xi or nl2 < 0.75 * xi or nl3 < 0.75 * xi or randv < 0.5)
		{
			pts.setColFrom(j, pts, i);
			++j;
		}
	}
	pts.conservativeResize(j);
}

template <typename T>
void SpectralDecompositionDataPointsFilter<T>::filterCurveness(DataPoints& pts, T xi, std::size_t k) const
{
	constexpr std::size_t seed = 1;
	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
	std::uniform_real_distribution<> uni01(0., 1.);
	
	const std::size_t nbPts = pts.getNbPoints();
	
	// Check field exists
	if (!pts.descriptorExists("lambda1") or !pts.descriptorExists("lambda2") or !pts.descriptorExists("lambda3"))
	{
		throw InvalidField("SpectralDecomposition<T>::filter: Error, lambdas field not found in descriptors.");
	}
	
	const auto& lambda1 = pts.getDescriptorViewByName("lambda1");
	const auto& lambda2 = pts.getDescriptorViewByName("lambda2");
	const auto& lambda3 = pts.getDescriptorViewByName("lambda3");

	std::size_t j = 0;
	for (std::size_t i = 0; i < nbPts; ++i)
	{
		const T randv = uni01(gen);	
		
		const T nl1 = lambda1(0,i) / k;
		const T nl2 = lambda2(0,i) / k;
		const T nl3 = lambda3(0,i) / k;
		
		if (nl1 < xi or nl2 < xi or nl3 < 0.5 * xi or randv < 0.8)
		{
			pts.setColFrom(j, pts, i);
			++j;
		}
	}
	pts.conservativeResize(j);
}

template <typename T>
void SpectralDecompositionDataPointsFilter<T>::filterPointness(DataPoints& pts, T xi, std::size_t k) const
{
	constexpr std::size_t seed = 1;
	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
	std::uniform_real_distribution<> uni01(0., 1.);
	
	const std::size_t nbPts = pts.getNbPoints();
	
	// Check field exists
	if (!pts.descriptorExists("lambda1") or !pts.descriptorExists("lambda2") or !pts.descriptorExists("lambda3"))
	{
		throw InvalidField("SpectralDecomposition<T>::filter: Error, lambdas field not found in descriptors.");
	}
	
	const auto& lambda1 = pts.getDescriptorViewByName("lambda1");
	const auto& lambda2 = pts.getDescriptorViewByName("lambda2");
	const auto& lambda3 = pts.getDescriptorViewByName("lambda3");

	std::size_t j = 0;
	for (std::size_t i = 0; i < nbPts; ++i)
	{
		const T randv = uni01(gen);	
		
		const T nl1 = lambda1(0,i) / k;
		const T nl2 = lambda2(0,i) / k;
		const T nl3 = lambda3(0,i) / k;
		
		if (nl1 < (5./6.) * xi or nl2 < (5./6.) * xi or nl3 < (5./6.) * xi or randv < 0.2)
		{
			pts.setColFrom(j, pts, i);
			++j;
		}
	}
	pts.conservativeResize(j);
}

template struct SpectralDecompositionDataPointsFilter<float>;
template struct SpectralDecompositionDataPointsFilter<double>;
