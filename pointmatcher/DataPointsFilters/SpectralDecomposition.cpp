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
#include <algorithm>

#include "OctreeGrid.h"

#define OCTREE_VERSION 1
#define RANDOM_VERSION 0
#define SURFACE_FIRST_VERSION 0
#define SALIENCIES_VERSION 0

// SpectralDecomposition
template <typename T>
SpectralDecompositionDataPointsFilter<T>::SpectralDecompositionDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("SpectralDecompositionDataPointsFilter", 
		SpectralDecompositionDataPointsFilter::availableParameters(), params),
	nbMaxPts{Parametrizable::get<std::size_t>("nbMaxPts")},
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
	
	if(k > nbPts) return;
	
	TensorVoting<T> tv{sigma, k};
	
//--- 1. Vote to determine prefered orientation + density estimation -----------
	tv.encode(cloud, TensorVoting<T>::Encoding::BALL);
	tv.cfvote(cloud, true);
	tv.decompose();
	tv.toDescriptors();
	addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, true /*lambdas*/, false /*tensors*/);
	
//--- 2. Filter iteratively on each measure (surfaceness, curveness, pointness) to uniformize density
	std::size_t it = 0;
	const std::size_t itMax_ = itMax;
	const std::size_t k_ = k;
	std::size_t oldnbPts = nbPts;
	
	auto checkConvergence = [&oldnbPts, &it, &itMax_, &k_](const DataPoints& pts, const std::size_t threshold) mutable ->bool{		
		const std::size_t nbPts = pts.getNbPoints();
		bool ret = (oldnbPts - nbPts) < threshold;
		
		oldnbPts = nbPts;
		
		return ret or ++it >= itMax_ or k_ >= nbPts;		
	};
	
	do 
	{	
	// 2.1 On pointness
		filterPointness(cloud, xi_expectation(3, sigma, radius), tv.k);
	// 2.2 On curveness
		filterCurveness(cloud, xi_expectation(1, sigma, radius/* / 2. */), tv.k);
	// 2.3 On surfaceness
		filterSurfaceness(cloud, xi_expectation(2, sigma, radius), tv.k);

	//Re-compute vote...
		tv.encode(cloud, TensorVoting<T>::Encoding::BALL);
		tv.cfvote(cloud, true);
		tv.decompose();
		tv.toDescriptors();
		//std::cout << "Add descriptors..."<< std::endl;
		addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, true /*lambdas*/, false /*tensors*/);
	} 
	while(not checkConvergence(cloud, 5 /*delta points*/));
	
//--- 3. Re-encode as Aware tensors + Re-vote ----------------------------------
	addDescriptor(cloud, tv, false /*normals*/, false /*labels*/, false /*lambdas*/, true /*tensors*/);
	tv.encode(cloud, TensorVoting<T>::Encoding::AWARE_TENSOR);
	tv.cfvote(cloud);
	tv.decompose();
	tv.toDescriptors();

//--- 4. Add descriptors
	addDescriptor(cloud, tv, keepNormals, true /*labels*/, keepLambdas, keepTensors); //TODO: add remove not kept descriptors
	
//--- 5. Remove outliers
	removeOutlier(cloud, tv);
	
	//std::cout<< "NbPts: " << cloud.getNbPoints() << std::endl;

	if(nbMaxPts < cloud.getNbPoints())
	{
#if OCTREE_VERSION
//--- 6. Reduce point cloud using octree for spatial distribution (surface first, then curve, then point)
		static constexpr int CURVE = 2;
		static constexpr int SURFACE = 3;
		
		Matrix labels = cloud.getDescriptorViewByName("labels");
	
		const std::size_t nbSurface = (labels.array() == SURFACE).count();
		const std::size_t nbCurve = (labels.array() == CURVE).count();

		//std::cout<< "NbCurve = " << nbCurve << ", NbSurface = " << nbSurface << ", NbPoint = " << nbPoint << std::endl;
	
		bool keepAllSurface = true, keepAllCurve = true, keepAllPoint = true;
	
		int leftToKeep = nbMaxPts - nbSurface;
		if(leftToKeep < 0) keepAllCurve = false;
		leftToKeep -= nbCurve;
		if(leftToKeep < 0) keepAllPoint = false;
	
		if(!keepAllPoint) 
		{
			std::size_t j = 0;
			for (std::size_t i = 0; i < cloud.getNbPoints(); ++i)
			{
				const int label = static_cast<int>(labels(i));

				bool keepPt = ((label == CURVE) and keepAllCurve) 
					or ((label == SURFACE) and keepAllSurface);
		
				if (keepPt)
				{
					cloud.setColFrom(j, cloud, i);
					++j;
				}
			}
			cloud.conservativeResize(j);
		}
	
		const T ratio = T(cloud.getNbPoints()) / T(nbMaxPts); 
	
		Parameters params; 
			params["maxPointByNode"] = std::to_string(static_cast<std::size_t>(ratio));
			params["samplingMethod"] = "3"; //medoid
			params["buildParallel"] = "0";
		
		DataPointsFilter* octreeFilter = 
			PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", params);

		octreeFilter->inPlaceFilter(cloud);
		
		const T prob = T(nbMaxPts) / T(cloud.getNbPoints());
		params.clear();	params["prob"] = std::to_string(prob);
		DataPointsFilter* rand_df= 
			PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
		
		rand_df->inPlaceFilter(cloud);
		
#elif SURFACE_FIRST_VERSION
//--- 6. Reduce point cloud taking surface first, then curves, then points
		static constexpr int POINT = 1;
		static constexpr int CURVE = 2;
		static constexpr int SURFACE = 3;

		constexpr std::size_t seed = 1;
		std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
		std::uniform_real_distribution<> uni01(0., 1.);
		
		T surfaceRatio = 0.; bool keepAllSurface = false;
		T curveRatio = 0.; bool keepAllCurve = false;
		T pointRatio = 0.; 
	
		// 6.1 Keep surfaces
		int leftToKeep = nbMaxPts - nbSurface;
		if(leftToKeep < 0.) //more curve than desried pts 
		{ 
			surfaceRatio = T(nbMaxPts) / T(nbSurface);
		}
		else
		{
			keepAllSurface = true;
			
			// 6.2 Keep curves
			leftToKeep = leftToKeep - nbCurve;
			if(leftToKeep < 0.) //more curve than desried pts 
			{ 
				curveRatio = T(nbMaxPts - nbSurface) / T(nbCurve);
			}
			else
			{
				keepAllCurve = true;
				
				// 6.3 Keep points
				leftToKeep = leftToKeep - nbPoint;
				pointRatio = T(nbMaxPts - nbCurve - nbPoint) / T(nbPoint);	
			}		
		}

		//std::cout<< "("<<keepAllCurve<<") cr= " << curveRatio << std::endl;
		//std::cout<< "("<<keepAllSurface<<") sr= " << surfaceRatio << std::endl;
		//std::cout<< "(0) pr= " << pointRatio << std::endl;
	
		std::size_t j = 0;
		for (std::size_t i = 0; i < cloud.getNbPoints(); ++i)
		{
			const int label = static_cast<int>(labels(i));
			const T randv = uni01(gen);

			bool keepPt = ((label == POINT) and  (randv < pointRatio)) 
				or ((label == CURVE) and  (keepAllCurve or randv < curveRatio)) 
				or ((label == SURFACE) and  (keepAllSurface or randv < surfaceRatio));
		
			if (keepPt)
			{
				cloud.setColFrom(j, cloud, i);
				++j;
			}
		}
		cloud.conservativeResize(j);	
	
#elif RANDOM_VERSION
//--- 6. Remove randomly point till desired number
		const std::size_t reducedNbPts = cloud.getNbPoints();

		const T prob = T(nbMaxPts) / T(reducedNbPts);
		Parameters params;	
			params["prob"] = std::to_string(prob);
		DataPointsFilter* rand_df= 
			PM::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", params);
		
		rand_df->inPlaceFilter(cloud);
#endif
	} //nbMaxPts < cloud.getNbPoints()
	
	cloud.save("spdf-"+std::to_string(getpid())+"-"+std::to_string(cloud.getNbPoints())+".vtk");
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

template <typename T>
void SpectralDecompositionDataPointsFilter<T>::removeOutlier(DataPoints& pts, const TensorVoting<T> &tv) const
{
	static constexpr int POINT = 0;
	static constexpr int CURVE = 1;
	static constexpr int SURFACE = 2;
	
	const std::size_t nbPts = pts.getNbPoints();
	
	if(nbMaxPts > nbPts) return ; //nothing to do

#if SALIENCIES_VERSION	
	const T ratio = T(nbMaxPts) / T(nbPts);
		
	Vector labels(nbPts);
	Matrix saliencies = (Matrix(nbPts,3) << tv.pointness.transpose(), tv.curveness.transpose(), tv.surfaceness.transpose()).finished();
	for(std::size_t i = 0; i < nbPts; ++i)
		saliencies.row(i).maxCoeff(&labels(i));
	
	const std::size_t nbSurface = (labels.array() == SURFACE).count();
	const std::size_t nthSurface = static_cast<std::size_t>(nbSurface * ratio);
	
	const std::size_t nbCurve = (labels.array() == CURVE).count();
	const std::size_t nthCurve = static_cast<std::size_t>(nbCurve * ratio);
	
	const std::size_t nbPoint = (labels.array() == POINT).count();
	const std::size_t nthPoint = static_cast<std::size_t>(nbPoint * ratio);
		
	std::vector<T>  surfaceness_(tv.surfaceness.data(), tv.surfaceness.data()+nbPts);
	std::vector<T>  curveness_(tv.curveness.data(), tv.curveness.data()+nbPts);
	std::vector<T>  pointness_(tv.pointness.data(), tv.pointness.data()+nbPts);

	std::nth_element(surfaceness_.begin(), surfaceness_.begin() + nthSurface, surfaceness_.end(), std::greater<T>());
	std::nth_element(curveness_.begin(), curveness_.begin() + nthCurve, curveness_.end(), std::greater<T>());
	std::nth_element(pointness_.begin(), pointness_.begin() + nthPoint, pointness_.end(), std::greater<T>());	

	const T th_s = surfaceness_[nthSurface];
	const T th_c = curveness_[nthCurve];
	const T th_p = pointness_[nthPoint];
	#if 0		
		std::cout   << nbMaxPts << " / "<< nbPts << " = " << ratio <<std::endl
					<< "S=" <<nbSurface << ", C="<< nbCurve << ", P=" << nbPoint <<std::endl
					<< "--------------------------------------------------------" <<std::endl
					<< "nthS=" <<nthSurface << ", nthC="<< nthCurve << ", nthP=" << nthPoint <<std::endl
					<< "--------------------------------------------------------" <<std::endl
					<< "th_S=" <<th_s << ", th_C="<< th_c << ", th_P=" << th_p <<std::endl << std::endl;
	#endif
#else
	const T th_p = (tv.pointness.maxCoeff() - tv.pointness.minCoeff()) * 0.1 + tv.pointness.minCoeff();
	const T th_c = (tv.curveness.maxCoeff() - tv.curveness.minCoeff()) * 0.1 + tv.curveness.minCoeff();
	const T th_s = (tv.surfaceness.maxCoeff() - tv.surfaceness.minCoeff()) * 0.1 + tv.surfaceness.minCoeff();
#endif
	
	std::size_t j = 0;
	for (std::size_t i = 0; i < nbPts; ++i)
	{
		const T surfaceness = tv.surfaceness(i);
		const T curveness = tv.curveness(i);
		const T pointness = tv.pointness(i);
#if SALIENCIES_VERSION		
		const int label = static_cast<int>(labels(i));
#else
		int label;
		(Vector(3) << pointness, curveness, surfaceness).finished().maxCoeff(&label); 
#endif
		bool keepPt = ((label == POINT) and  (pointness >= th_p)) 
			or ((label == CURVE) and  (curveness >= th_c)) 
			or ((label == SURFACE) and  (surfaceness >= th_s));
		
		if (keepPt)
		{
			pts.setColFrom(j, pts, i);
			++j;
		}
	}
	pts.conservativeResize(j);	
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
		
		if (nl1 < xi or nl2 < xi or nl3 < 0.5 * xi or randv < 0.5)
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
