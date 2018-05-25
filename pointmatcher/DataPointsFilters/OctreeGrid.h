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
#pragma once

#include "PointMatcher.h"
#include "utils/octree.h"

/*!
 * \class OctreeGridDataPointsFilter
 * \brief Data Filter based on Octree representation
 *
 * \author Mathieu Labussiere (<mathieu dot labu at gmail dot com>)
 * \date 24/05/2018
 * \version 0.1
 *
 * Processings are applyed via Visitors through Depth-first search in the Octree (DFS)
 * i.e. for each node, the Visitor/Callback is call
 */
template<typename T>
struct OctreeGridDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
  	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;

	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
		return "Construct an Octree grid of the point cloud. Constructed either by limiting the number of point in each octant or by limiting the size of the bounding box. Down-sample by taking either the first or a random point, or compute the centroid.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return boost::assign::list_of<ParameterDoc>
		( "buildMethod", "Method to build the Octree: maxPoint (0), maxSize (1)", "0", "0", "1", &P::Comp<int> )
		( "buildParallel", "If 1 (true), use threads to build the octree.", "1", "0", "1", P::Comp<bool> )
		( "maxPointByNode", "Number of point under which the octree stop dividing.", "1", "1", "+inf", &P::Comp<std::size_t> )
		( "maxSizeByNode", "Size of the bounding box under which the octree stop dividing.", "0.01", "0.0001", "+inf", &P::Comp<T> )
		( "samplingMethod", "Method to sample the Octree: First Point (0), Random (1), Centroid (2) (more accurate but costly)", "0", "0", "2", &P::Comp<int> )
		;
	}
//Visitors class to apply processing
	struct FirstPtsSampler;
	struct RandomPtsSampler : FirstPtsSampler;
	struct CentroidSampler : FirstPtsSampler;

//-------	
	enum BuildMethod : int { MAX_POINT=0, MAX_SIZE=1 }; 
	enum SamplingMethod : int { FIRST_PTS=0, RAND_PTS=1, CENTROID=2 };

//Atributes
	bool parallel_build;
	BuildMethod buildMethod;
	
	std::size_t maxPointByNode;
	T           maxSizeByNode;
	
	SamplingMethod samplingMethod;

//Methods	
	//Constructor, uses parameter interface
	OctreeGridDataPointsFilter(const Parameters& params = Parameters());

	OctreeGridDataPointsFilter();
	// Destr
	virtual ~OctreeGridDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};
