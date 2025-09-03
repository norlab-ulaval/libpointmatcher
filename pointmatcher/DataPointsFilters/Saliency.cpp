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
#include "Saliency.h"
#include "utils/sparsetv.h"


// NormalSpaceDataPointsFilter
template <typename T>
SaliencyDataPointsFilter<T>::SaliencyDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("SaliencyDataPointsFilter", 
		SaliencyDataPointsFilter::availableParameters(), params),
	k{Parametrizable::get<std::size_t>("k")},
	sigma{Parametrizable::get<T>("sigma")},
	keepNormals{Parametrizable::get<bool>("keepNormals")},
	keepLabels{Parametrizable::get<bool>("keepLabels")},
	keepTensors{Parametrizable::get<bool>("keepTensors")}
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
SaliencyDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void SaliencyDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
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
	
	Matrix labels  = Matrix::Zero(1, nbPts);
	for(std::size_t i = 0; i < nbPts; ++i)
	{
		const T lambda1 = tv.surfaceness(i);
		const T lambda2 = tv.curveness(i);
		const T lambda3 = tv.pointness(i);
		
		int index;
		Vector coeff = (Vector(3) << lambda3, (lambda2 - lambda3), (lambda1 - lambda2)).finished();
		coeff.maxCoeff(&index);  

		labels(i) = index + 1 ;
	}
	
	try
	{	
	    cloud.addDescriptor("surfaceness", tv.surfaceness);
	    cloud.addDescriptor("curveness", tv.curveness);
	    cloud.addDescriptor("pointness", tv.pointness);
	    
		if(keepNormals)
		{    
			cloud.addDescriptor("normals", tv.normals);
			cloud.addDescriptor("tangents", tv.tangents);
		}    
	    if(keepLabels)
	    {
			cloud.addDescriptor("labels", labels);
	    }  
	    if(keepTensors)
	    {
			cloud.addDescriptor("sticks", tv.sticks);
			cloud.addDescriptor("plates", tv.plates);
			cloud.addDescriptor("balls", tv.balls);
	    }
	}
	catch (...) {
	    std::cerr << "SaliencyDataPointsFilter<T>::inPlaceFilter: Cannot add descriptors to pointcloud" << std::endl;
	}
}

template struct SaliencyDataPointsFilter<float>;
template struct SaliencyDataPointsFilter<double>;
