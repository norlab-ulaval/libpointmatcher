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
#include "NormalSpace.h"

#include <vector>
#include <unordered_map>
#include <random>
#include <ciso646>
#include <cmath>

// NormalSpaceDataPointsFilter
template <typename T>
NormalSpaceDataPointsFilter<T>::NormalSpaceDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("NormalSpaceDataPointsFilter", 
		NormalSpaceDataPointsFilter::availableParameters(), params),
	nbSample{Parametrizable::get<std::size_t>("nbSample")},
	seed{Parametrizable::get<std::size_t>("seed")},
	epsilon{Parametrizable::get<T>("epsilon")},
	nbBucket{std::size_t((2.0 * M_PI / epsilon) * (M_PI / epsilon))}
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
NormalSpaceDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

//TODO: Add support for 2D by building histogram of polar coordinate with uniform sampling

template <typename T>
void NormalSpaceDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	static const int alreadySampled = -1;
	
	//check dimension
	const std::size_t featDim = cloud.features.rows();
	if(featDim < 4) //3D case support only
	{
		std::cerr << "ERROR: NormalSpaceDataPointsFilter does not support 2D point cloud yet (does nothing)" << std::endl;
		return;
	}
		
	//Check number of points
	const int nbPoints = cloud.getNbPoints();		
	if(nbSample >= std::size_t(nbPoints))
		return;

	//Check if there is normals info
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");

	const auto& normals = cloud.getDescriptorViewByName("normals");
	
	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed
	std::uniform_real_distribution<> uni01(0., 1.);
	
	//bucketed normal space
	std::vector<std::vector<int> > idBuckets; //stock int so we can marked selected with -1
	idBuckets.resize(nbBucket);
	
	std::vector<std::size_t> keepIndexes;
	keepIndexes.reserve(nbSample);
	
	///(1) put all points of the data into buckets based on their normal direction
	for (int i = 0; i < nbPoints; ++i)
	{
		assert(normals.col(i).head(3).norm() > 0.99999);
		
		//Theta = polar angle in [0 ; pi]
		const T theta = std::acos(normals(2, i)); 
		//Phi = azimuthal angle in [0 ; 2pi] 
		const T phi = std::fmod(std::atan2(normals(1, i), normals(0, i)) + 2. * M_PI, 2. * M_PI);
		
		//assert(theta >= 0. and theta =< M_PI and phi >= 0. and phi <= 2.*M_PI);
		
		idBuckets[bucketIdx(theta, phi)].push_back(i);
	}
	///(2) uniformly pick points from all the buckets until the desired number of points is selected
	while(keepIndexes.size() < nbSample)
	{
		const T theta = std::acos(1 - 2 * uni01(gen));
		const T phi = 2. * M_PI * uni01(gen);

		std::vector<int>& curBucket = idBuckets[bucketIdx(theta, phi)];

		//Check size
		if(curBucket.empty())
			continue;
	
		const std::size_t bucketSize = curBucket.size();

		//Check if not already all sampled
		bool isEntireBucketSampled = true;
		for(std::size_t id = 0; id < bucketSize and isEntireBucketSampled; ++id)
		{
			isEntireBucketSampled = isEntireBucketSampled and (curBucket[id] == alreadySampled);
		}

		if(isEntireBucketSampled)
			continue;
			
		///(3) A point is randomly picked in a bucket that contains multiple points
		int idToKeep = 0;
		std::size_t idInBucket = 0;
		do
		{
			 idInBucket = static_cast<std::size_t>(bucketSize * uni01(gen));
			 idToKeep = curBucket[idInBucket];

		} while(idToKeep == alreadySampled); 

		keepIndexes.push_back(static_cast<std::size_t>(idToKeep));

		curBucket[idInBucket] = alreadySampled; //set sampled flag
	}
	//TODO: evaluate performances between this solution and sorting the indexes
	// We build map of (old index to new index), in case we sample pts at the begining of the pointcloud
	std::unordered_map<std::size_t, std::size_t> mapidx;
	std::size_t idx = 0;
	
	///(4) Sample the point cloud
	for(std::size_t id : keepIndexes)
	{
		//retrieve index from lookup table if sampling in already switched element
		if(id<idx)
			id = mapidx[id];
		//Switch columns id and idx
		cloud.swapCols(idx, id);	
		//Maintain new index position	
		mapidx[idx] = id;
		//Update index
		++idx;
	}
	cloud.conservativeResize(nbSample);
}

template <typename T>
std::size_t NormalSpaceDataPointsFilter<T>::bucketIdx(T theta, T phi) const
{
	//Theta = polar angle in [0 ; pi] and Phi = azimuthal angle in [0 ; 2pi]
	return static_cast<std::size_t>(theta / epsilon) * static_cast<std::size_t>(2. * M_PI / epsilon) + static_cast<std::size_t>(phi / epsilon);
}

template struct NormalSpaceDataPointsFilter<float>;
template struct NormalSpaceDataPointsFilter<double>;
