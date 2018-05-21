#pragma once

#include "PointMatcher.h"

#include <vector>
#include <algorithm>

namespace utils {

template<typename T>
struct IdxCompare
{
	typedef typename PointMatcher<T>::Vector Vector;
	const Vector& target;

	IdxCompare(const typename PointMatcher<T>::Vector& target): target(target) {}

	bool operator()(size_t a, size_t b) const { return target(a,0) < target(b,0); }
};


template<typename T>
std::vector<size_t> 
sortIndexes(const typename PointMatcher<T>::Vector& v)
{
	// initialize original index locations
	const size_t idxSize = v.size();
	std::vector<size_t> idx(idxSize);
	for(size_t i=0; i < idxSize; ++i) idx[i]=i;

	// sort indexes based on comparing values in v
	std::sort(idx.begin(), idx.end(), IdxCompare<T>(v));

	return idx;
}

template<typename T>
typename PointMatcher<T>::Vector
sortEigenValues(const typename PointMatcher<T>::Vector& eigenVa)
{
	// sort the eigenvalues in ascending order
	typename PointMatcher<T>::Vector eigenVaSort = eigenVa;
	std::sort(eigenVaSort.data(), eigenVaSort.data() + eigenVaSort.size());
	return eigenVaSort;
}

template<typename T>
typename PointMatcher<T>::Vector 
serializeEigVec(const typename PointMatcher<T>::Matrix& eigenVe)
{
	// serialize row major
	const int eigenVeDim = eigenVe.cols();
	typename PointMatcher<T>::Vector output(eigenVeDim*eigenVeDim);
	for(int k=0; k < eigenVeDim; ++k)
	{
		output.segment(k*eigenVeDim, eigenVeDim) = 
			eigenVe.row(k).transpose();
	}

	return output;
}

template<typename T>
T computeDensity(const typename PointMatcher<T>::Matrix& NN)
{
	//volume in meter
	const T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff(), 3);

	//volume in decimeter
	//T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff()*10.0, 3);
	//const T minVolume = 4.18e-9; // minimum of volume of one millimeter radius
	//const T minVolume = 0.42; // minimum of volume of one centimeter radius (in dm^3)

	//if(volume < minVolume)
	//	volume = minVolume;

	return T(NN.cols())/(volume);
}
template<typename T>
typename PointMatcher<T>::Vector
computeNormal(const typename PointMatcher<T>::Vector& eigenVa, const typename PointMatcher<T>::Matrix& eigenVe)
{
	// Keep the smallest eigenvector as surface normal
	const int nbEigenCol = eigenVe.cols();
	int smallestId(0);
	T smallestValue(std::numeric_limits<T>::max());
	for(int j = 0; j < nbEigenCol ; ++j)
	{
		if (eigenVa(j) < smallestValue)
		{
			smallestId = j;
			smallestValue = eigenVa(j);
		}
	}

  return eigenVe.col(smallestId);
}

template<typename T>
size_t argMax(const typename PointMatcher<T>::Vector& v)
{
	//FIXME: Change that to use the new API. the new Eigen API (3.2.8) allows this with the call maxCoeff. See the section Visitors in https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
	const int size(v.size());
	T maxVal(0);
	size_t maxIdx(0);
	for (int i = 0; i < size; ++i)
	{
		if (v[i] > maxVal)
		{
			maxVal = v[i];
			maxIdx = i;
		}
	}
	return maxIdx;
}

};
