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
#include "Geometry.h"

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"
#include "IO.h"
#include "MatchersImpl.h"

#include <boost/format.hpp>


#include "utils.h"

// SurfaceNormalDataPointsFilter
// Constructor
template<typename T>
GeometryDataPointsFilter<T>::GeometryDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("GeometryDataPointsFilter",
	        GeometryDataPointsFilter::availableParameters(), params),
    keepUnstructureness(Parametrizable::get<int>("keepUnstructureness")),
    keepStructureness(Parametrizable::get<T>("keepStructureness"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
GeometryDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void GeometryDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;

	const int pointsCount(cloud.features.cols());
	const int descDim(cloud.descriptors.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	// Check that the required eigenValue descriptor exists in the pointcloud
    if (!cloud.descriptorExists("eigValues"))
    {
        throw InvalidField("GeometryDataPointsFilter: Error, no eigValues found in descriptors.");
    }

    // Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < labelDim ; ++i)
		insertDim += cloud.descriptorLabels[i].span;
	if (insertDim != descDim)
		throw InvalidField("SurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");

	// Reserve memory for new descriptors
	const int dimSphericality(1);
	const int dimUnstructureness(1);
	const int dimStructureness(1);

	boost::optional<View> sphericality;
	boost::optional<View> unstructureness;
	boost::optional<View> structureness;

	Labels cloudLabels;
	cloudLabels.push_back(Label("sphericality", dimSphericality));
	if (keepUnstructureness)
		cloudLabels.push_back(Label("unstructureness", dimUnstructureness));
	if (keepStructureness)
		cloudLabels.push_back(Label("structureness", dimStructureness));

    // Reserve memory
	cloud.allocateDescriptors(cloudLabels);

	// Get the views
    const View eigValues = cloud.getDescriptorViewByName("eigValues");
    if (eigValues.rows() != 3)  // And check the dimensions
    {
        throw InvalidField("GeometryDataPointsFilter: Error, the number of eigValues is not 3.");
    }

	sphericality = cloud.getDescriptorViewByName("sphericality");
	if (keepUnstructureness)
		unstructureness = cloud.getDescriptorViewByName("unstructureness");
	if (keepStructureness)
		structureness = cloud.getDescriptorViewByName("structureness");

    // Iterate through the point cloud and evaluate the geometry
	for (int i = 0; i < pointsCount; ++i)
	{
	    // look at the three eigen values
        Vector eigVec = eigValues.col(i);
        // might be already sorted but sort anyway
        std::sort(eigVec.data(),eigVec.data()+eigVec.size());

        // finally, evaluate the geometry
        T sphericality_val;
        T unstructureness_val;
        T structureness_val;

        if (abs(eigVec(2)) < std::numeric_limits<T>::min() or
            abs(eigVec(1)) < std::numeric_limits<T>::min())
        {
            sphericality_val = std::numeric_limits<T>::quiet_NaN();
            unstructureness_val = std::numeric_limits<T>::quiet_NaN();
            structureness_val = std::numeric_limits<T>::quiet_NaN();
        } else {
            unstructureness_val = eigVec(0) / eigVec(2);
            structureness_val =  (eigVec(1) / eigVec(2)) *
                    ((eigVec(1) - eigVec(0)) / sqrt(eigVec(0)*eigVec(0) + eigVec(1)*eigVec(1)));
            sphericality_val = unstructureness_val - structureness_val;
        }

        // store in the pointcloud
        (*sphericality)(0,i) = sphericality_val;
        if (keepUnstructureness)
            (*unstructureness)(0,i) = unstructureness_val;
        if (keepStructureness)
            (*structureness)(0,i) = structureness_val;

	}
	
}

template struct GeometryDataPointsFilter<float>;
template struct GeometryDataPointsFilter<double>;

