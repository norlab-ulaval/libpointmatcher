// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"
#include <iostream>

using namespace std;

//! Return whether there is a label named text
template<typename T>
bool PointMatcher<T>::DataPoints::Labels::contains(const std::string& text) const
{
	for (const_iterator it(this->begin()); it != this->end(); ++it)
	{
		if (it->text == text)
			return true;
	}
	return false;
}

//! Return the sum of the spans of each label
template<typename T>
size_t PointMatcher<T>::DataPoints::Labels::totalDim() const
{
	size_t dim(0);
	for (const_iterator it(this->begin()); it != this->end(); ++it)
		dim += it->span;
	return dim;
}

//! Construct a label from a given name and number of data dimensions it spans
template<typename T>
PointMatcher<T>::DataPoints::Label::Label(const std::string& text, const size_t span):
	text(text),
	span(span)
{}

//! Construct an empty point cloud
template<typename T>
PointMatcher<T>::DataPoints::DataPoints()
{}

//! Construct a point cloud from existing descriptions
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Labels& featureLabels, const Labels& descriptorLabels, const size_t pointCount):
	featureLabels(featureLabels),
	descriptorLabels(descriptorLabels)
{
	features.resize(featureLabels.totalDim(), pointCount);
	descriptors.resize(descriptorLabels.totalDim(), pointCount);
}

//! Construct a point cloud from existing features without any descriptor
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Matrix& features, const Labels& featureLabels):
	features(features),
	featureLabels(featureLabels)
{}

//! Construct a point cloud from existing features and descriptors
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels):
	features(features),
	featureLabels(featureLabels),
	descriptors(descriptors),
	descriptorLabels(descriptorLabels)
{}

//! Add an other point cloud after the current one
template<typename T>
void PointMatcher<T>::DataPoints::concatenate(const DataPoints& dp)
{
	const int nbPoints1 = this->features.cols();
	const int nbPoints2 = dp.features.cols();
	const int nbPointsTotal = nbPoints1 + nbPoints2;

	const int dimFeat = this->features.rows();
	if(dimFeat != dp.features.rows())
	{
		stringstream errorMsg;
		errorMsg << "Cannot concatenate DataPoints because the dimension of the features are not the same. Actual dimension: " << dimFeat << " New dimension: " << dp.features.rows(); 
		throw InvalidField(errorMsg.str());
	}
	
	Matrix combinedFeat(dimFeat, nbPointsTotal);
	combinedFeat.leftCols(nbPoints1) = this->features;
	combinedFeat.rightCols(nbPoints2) = dp.features;
	DataPoints dpOut(combinedFeat, this->featureLabels);
	
	for(unsigned i = 0; i < this->descriptorLabels.size(); i++)
	{
		const string name = this->descriptorLabels[i].text;
		const int dimDesc = this->descriptorLabels[i].span;
		if(dp.descriptorExists(name, dimDesc) == true)
		{
			Matrix mergedDesc(dimDesc, nbPointsTotal);
			mergedDesc.leftCols(nbPoints1) = this->getDescriptorViewByName(name);
			mergedDesc.rightCols(nbPoints2) = dp.getDescriptorViewByName(name);
			dpOut.addDescriptor(name, mergedDesc);
		}
	}

	this->features.swap(dpOut.features);
	this->featureLabels = dpOut.featureLabels;
	this->descriptors.swap(dpOut.descriptors);
	this->descriptorLabels = dpOut.descriptorLabels;
}



//! Makes sure a feature of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateFeature(const std::string& name, const unsigned dim)
{
	allocateField(name, dim, featureLabels, features);
}

//! Make sure a vector of features of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateFeatures(const Labels& newLabels)
{
	allocateFields(newLabels, featureLabels, features);
}

//! Add a feature by name, remove first if already exists
template<typename T>
void PointMatcher<T>::DataPoints::addFeature(const std::string& name, const Matrix& newFeature)
{
	addField(name, newFeature, featureLabels, features);
}

//! Get feature by name, return a matrix containing a copy of the requested feature
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::DataPoints::getFeatureCopyByName(const std::string& name) const
{
	return Matrix(getFeatureViewByName(name));
}

//! Get a const view on a feature by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getFeatureViewByName(const std::string& name) const
{
	return getConstViewByName(name, featureLabels, features);
}

//! Look if a feature with a given name exist
template<typename T>
bool PointMatcher<T>::DataPoints::featureExists(const std::string& name) const
{
	return fieldExists(name, 0, featureLabels);
}

//! Look if a feature with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::featureExists(const std::string& name, const unsigned dim) const
{
	return fieldExists(name, dim, featureLabels);
}

//! Return the dimension of a feature with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFeatureDimension(const std::string& name) const
{
	return getFieldDimension(name, featureLabels);
}

//! Return the starting row of a feature with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFeatureStartingRow(const std::string& name) const
{
	return getFieldStartingRow(name, featureLabels);
}


//! Get a view on a feature by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getFeatureViewByName(const std::string& name)
{
	return getViewByName(name, featureLabels, features);
}

//! Makes sure a descriptor of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateDescriptor(const std::string& name, const unsigned dim)
{
	allocateField(name, dim, descriptorLabels, descriptors);
}

//! Make sure a vector of descriptors of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateDescriptors(const Labels& newLabels)
{
	allocateFields(newLabels, descriptorLabels, descriptors);
}

//! Add a descriptor by name, remove first if already exists
template<typename T>
void PointMatcher<T>::DataPoints::addDescriptor(const std::string& name, const Matrix& newDescriptor)
{
	addField(name, newDescriptor, descriptorLabels, descriptors);
}

//! Get descriptor by name, return a matrix containing a copy of the requested descriptor
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::DataPoints::getDescriptorCopyByName(const std::string& name) const
{
	return Matrix(getDescriptorViewByName(name));
}

//! Get a const view on a descriptor by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getDescriptorViewByName(const std::string& name) const
{
	return getConstViewByName(name, descriptorLabels, descriptors);
}


//! Get a view on a descriptor by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getDescriptorViewByName(const std::string& name)
{
	return getViewByName(name, descriptorLabels, descriptors);
}

//! Look if a descriptor with a given name exist
template<typename T>
bool PointMatcher<T>::DataPoints::descriptorExists(const std::string& name) const
{
	return fieldExists(name, 0, descriptorLabels);
}

//! Look if a descriptor with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::descriptorExists(const std::string& name, const unsigned dim) const
{
	return fieldExists(name, dim, descriptorLabels);
}

//! Return the dimension of a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getDescriptorDimension(const std::string& name) const
{
	return getFieldDimension(name, descriptorLabels);
}

//! Return the starting row of a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getDescriptorStartingRow(const std::string& name) const
{
	return getFieldStartingRow(name, descriptorLabels);
}


//! Make sure a field of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateField(const std::string& name, const unsigned dim, Labels& labels, Matrix& data) const
{
	if (fieldExists(name, 0, labels))
	{
		const unsigned descDim(getFieldDimension(name, labels));
		if (descDim != dim)
		{
			throw InvalidField(
				(boost::format("The existing field %1% has dimension %2%, different than requested dimension %3%") % name % descDim % dim).str()
			);
		}
	}
	else
	{
		const int oldDim(data.rows());
		const int totalDim(oldDim + dim);
		const int pointCount(features.cols());
		data.conservativeResize(totalDim, pointCount);
		labels.push_back(Label(name, dim));
	}
}

//! Make sure a vector of fields of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateFields(const Labels& newLabels, Labels& labels, Matrix& data) const
{
	typedef vector<bool> BoolVector;
	BoolVector present(newLabels.size(), false);
	
	// for fields that exist, take note and check dimension
	size_t additionalDim(0);
	for (size_t i = 0; i < newLabels.size(); ++i)
	{
		const string& newName(newLabels[i].text);
		const size_t newSpan(newLabels[i].span);
		for(auto it(labels.begin()); it != labels.end(); ++it)
		{
			if (it->text == newName)
			{
				if (it->span != newSpan)
					throw InvalidField(
						(boost::format("The existing field %1% has dimension %2%, different than requested dimension %3%") % newName % it->span % newSpan).str()
					);
				present[i] = true;
				break;
			}
		}
		if (!present[i])
			additionalDim += newSpan;
	}
	
	// for new fields allocate
	const int oldDim(data.rows());
	const int totalDim(oldDim + additionalDim);
	const int pointCount(features.cols());
	data.conservativeResize(totalDim, pointCount);
	for (size_t i = 0; i < newLabels.size(); ++i)
	{
		if (!present[i])
			labels.push_back(newLabels[i]);
	}
}

//! Add a descriptor or feature by name, remove first if already exists
template<typename T>
void PointMatcher<T>::DataPoints::addField(const std::string& name, const Matrix& newField, Labels& labels, Matrix& data) const
{
	const int newFieldDim = newField.rows();
	const int newPointCount = newField.cols();
	const int pointCount = features.cols();

	if (newField.rows() == 0)
		return;

	// Replace if the field exists
	if (fieldExists(name, 0, labels))
	{
		const int fieldDim = getFieldDimension(name, labels);
		
		if(fieldDim == newFieldDim)
		{
			// Ensure that the number of points in the point cloud and in the field are the same
			if(pointCount == newPointCount)
			{
				const int row = getFieldStartingRow(name, labels);
				data.block(row, 0, fieldDim, pointCount) = newField;
			}
			else
			{
				stringstream errorMsg;
				errorMsg << "The field " << name << " cannot be added because the number of points is not the same. Old point count: " << pointCount << "new: " << newPointCount;
				throw InvalidField(errorMsg.str());
			}
		}
		else
		{
			stringstream errorMsg;
			errorMsg << "The field " << name << " already exists but could not be added because the dimension is not the same. Old dim: " << fieldDim << " new: " << newFieldDim;
			throw InvalidField(errorMsg.str());
		}
	}
	else // Add at the end if it is a new field
	{
		if(pointCount == newPointCount)
		{
			const int oldFieldDim(data.rows());
			const int totalDim = oldFieldDim + newFieldDim;
			data.conservativeResize(totalDim, pointCount);
			data.bottomRows(newFieldDim) = newField;
			labels.push_back(Label(name, newFieldDim));
		}
		else
		{
			stringstream errorMsg;
			errorMsg << "The field " << name << " cannot be added because the number of points is not the same. Old point count: " << pointCount << " new: " << newPointCount;
			throw InvalidField(errorMsg.str());
		}
	}
}

//! Get a const view on a matrix by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getConstViewByName(const std::string& name, const Labels& labels, const Matrix& data) const
{
	unsigned row(0);
	for(auto it(labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return data.block(row, 0, it->span, data.cols());
		row += it->span;
	}
	throw InvalidField("Field " + name + " not found");
}

//! Get a view on a matrix by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getViewByName(const std::string& name, const Labels& labels, Matrix& data) const
{
	unsigned row(0);
	for(auto it(labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return data.block(row, 0, it->span, data.cols());
		row += it->span;
	}
	throw InvalidField("Field " + name + " not found");
}

//! Look if a descriptor or a feature with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::fieldExists(const std::string& name, const unsigned dim, const Labels& labels) const
{
	for(auto it(labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
		{
			if (dim == 0 || it->span == dim)
				return true;
			else
				return false;
		}
	}
	return false;
}


//! Return the dimension of a feature or a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFieldDimension(const std::string& name, const Labels& labels) const
{
	for(auto it(labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return it->span;
	}
	return 0;
}


//! Return the starting row of a feature or a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFieldStartingRow(const std::string& name, const Labels& labels) const
{
	unsigned row(0);
	for(auto it(labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return row;
		row += it->span;
	}
	return 0;
}

//! Exchange in place point clouds a and b, with no data copy
template<typename T>
void PointMatcher<T>::swapDataPoints(DataPoints& a, DataPoints& b)
{
	a.features.swap(b.features);
	swap(a.featureLabels, b.featureLabels);
	a.descriptors.swap(b.descriptors);
	swap(a.descriptorLabels, b.descriptorLabels);
}

template struct PointMatcher<float>;
template struct PointMatcher<double>;
