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
#include "ComputeNormals.h"

// Eigenvalues
#include "Eigen/Eigenvalues"

#include <boost/format.hpp>

#include "DataPointsFilters/utils/utils.h"

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ComputeNormalsDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void ComputeNormalsDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{

    bool has_deviation = cloud.descriptorExists("deviation");
    bool has_eigenVectors = (cloud.descriptorExists("eigValues") && cloud.descriptorExists("eigVectors"));
    if (!has_deviation && !has_eigenVectors)
    {
        throw MissingDescriptor("ComputeNormals error. The input point cloud must either deviation, or eigValues and eigVectors descriptors.");
    }

	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;

	const int pointsCount(cloud.getNbPoints());
	const int dim(cloud.features.rows()-1);
	const int descDim(cloud.descriptors.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	// Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < labelDim ; ++i)
		insertDim += cloud.descriptorLabels[i].span;
	if (insertDim != descDim)
		throw InvalidField("ComputeNormals: Error, descriptor labels do not match descriptor data");

	// Reserve memory for new descriptors
	const int dimNormals(dim);

	boost::optional<View> normals;

    if (! cloud.descriptorLabels.contains("normals"))
    {
        Labels cloudLabels;
        cloudLabels.push_back(Label("normals", dimNormals));
        // Reserve memory
        cloud.allocateDescriptors(cloudLabels);
    }
    normals = cloud.getDescriptorViewByName("normals");

	for (int i = 0; i < pointsCount; ++i)
	{
		Vector eigenVa = Vector::Zero(dim, 1);
		Matrix eigenVe = Matrix::Zero(dim, dim);
        if (has_deviation)
        {
            Matrix C = cloud.getDescriptorViewByName("deviation").col(i).reshaped(dim, dim);
            // compute eigenvalues and eigenvectors out of deviations
            if(C.fullPivHouseholderQr().rank() >= dim)
            {
                const Eigen::SelfAdjointEigenSolver<Matrix> solver(C);
                eigenVa = solver.eigenvalues().real();
                eigenVe = solver.eigenvectors().real();
            }
        }
        if (has_eigenVectors)
        {
            eigenVa = cloud.getDescriptorViewByName("eigValues").col(i);
            eigenVe = cloud.getDescriptorViewByName("eigVectors").col(i).reshaped(dim, dim);
        }

        normals->col(i) = PointMatcherSupport::computeNormal<T>(eigenVa, eigenVe);
        // clamp normals to [-1,1] to handle approximation errors
        normals->col(i) = normals->col(i).cwiseMax(-1.0).cwiseMin(1.0);
    }
}

template struct ComputeNormalsDataPointsFilter<float>;
template struct ComputeNormalsDataPointsFilter<double>;

