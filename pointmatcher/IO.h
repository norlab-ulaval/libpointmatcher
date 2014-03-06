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

#ifndef __POINTMATCHER_IO_H
#define __POINTMATCHER_IO_H

#include "PointMatcher.h"

//! IO Functions and classes that are dependant on scalar type are defined in this templatized class
template<typename T>
struct PointMatcherIO
{
	typedef typename PointMatcher<T>::Vector Vector; //!< alias
	typedef typename PointMatcher<T>::Matrix Matrix; //!< alias
	typedef typename PointMatcher<T>::DataPoints DataPoints; //!< alias
	typedef typename PointMatcher<T>::TransformationParameters TransformationParameters; //!< alias
	typedef typename PointMatcher<T>::Matrix Parameters; //!< alias

	// CSV

	static DataPoints loadCSV(const std::string& fileName);
	static DataPoints loadCSV(std::istream& is);

	static void saveCSV(const DataPoints& data, const std::string& fileName);
	static void saveCSV(const DataPoints& data, std::ostream& os);

	// VTK

	static DataPoints loadVTK(const std::string& fileName);
	static DataPoints loadVTK(std::istream& is);

	static void saveVTK(const DataPoints& data, const std::string& fileName);

	// PLY

	static DataPoints loadPLY(const std::string& fileName);
	static DataPoints loadPLY(std::istream& is);

	static void savePLY(const DataPoints& data, const std::string& fileName); //! save datapoints to PLY point cloud format

	//! Information to exploit a reading from a file using this library. Fields might be left blank if unused.
	struct FileInfo
	{
		typedef Eigen::Matrix<T, 3, 1> Vector3; //!< alias

		std::string readingFileName; //!< file name of the reading point cloud
		std::string referenceFileName; //!< file name of the reference point cloud
		std::string configFileName; //!< file name of the yaml configuration
		TransformationParameters initialTransformation; //!< matrix of initial estimate transform
		TransformationParameters groundTruthTransformation; //!< matrix of the ground-truth transform
		Vector3 gravity; //!< gravity vector

		FileInfo(const std::string& readingPath="", const std::string& referencePath="", const std::string& configFileName="", const TransformationParameters& initialTransformation=TransformationParameters(), const TransformationParameters& groundTruthTransformation=TransformationParameters(),  const Vector& grativity=Vector3::Zero());
	};

	//! A vector of file info, to be used in batch processing
	struct FileInfoVector: public std::vector<FileInfo>
	{
		FileInfoVector();
		FileInfoVector(const std::string& fileName, std::string dataPath = "", std::string configPath = "");

	protected:
		std::string localToGlobalFileName(const std::string& path, const std::string& fileName);
		bool findTransform(const PointMatcherSupport::CsvElements& data, const std::string& prefix, unsigned dim);
		TransformationParameters getTransform(const PointMatcherSupport::CsvElements& data, const std::string& prefix, unsigned dim, unsigned line);
	};

	//! Check that property defined by type is a valid PLY type note: type must be lowercase
	static bool plyPropTypeValid (const std::string& type);

	//! Interface for PLY property
	struct PLYProperty
	{

		std::string name; //!< name of PLY property
		std::string type; //!< type of PLY property
		std::string idx_type; //!< for list properties, type of number of elements
		unsigned pos; //!< index of the property in element
		bool is_list; //!< member is true of property is a list
		bool is_feature; //!<member is true if is a PM feature, if not, it is a descriptor

		// regular property
		PLYProperty(const std::string& type, const std::string& name, const unsigned pos, const bool is_feature = false);

		// list property
		PLYProperty(const std::string& idx_type, const std::string& type, const std::string& name, const unsigned pos, const bool is_feature = false); //! list prop ctor

		bool operator==(const PLYProperty& other) const; //! compare with other property
	};

	/*! Interface for all PLY elements
	* Must overload supportsProperty to define all properties
	* which are allowed in this element type */
	class PLYElement
	{

	public:
		std::string name; //!< name identifying the PLY element
		unsigned num; //!< number of occurences of the element
		unsigned total_props; //!< total number of properties in PLY element
		unsigned offset; //!< line at which data starts

		PLYElement(const std::string& name, const unsigned num, const unsigned offset) :
			name(name), num(num), total_props(0), offset(offset) {} //! default ctor

		bool supportsProperty(const PLYProperty& prop) const; //! Returns true if property pro is supported by element

		void addProperty(PLYProperty& prop); //! add a property to vector of properties

		const std::vector<PLYProperty>& getFeatureProps() const; //! return vector of feature properties

		const std::vector<PLYProperty>& getDescriptorProps() const; //! return vector of descriptor properties

		size_t getNumSupportedProperties() const; //! return number of properties

		int getNumFeatures() const; //! get number of PM supported feature properties

		int getNumDescriptors() const; //! get number of PM supported descriptor properties

		bool operator==(const PLYElement& other) const;

	protected:
		enum PMPropTypes
		{
			FEATURE,
			DESCRIPTOR,
			UNSUPPORTED
		};

		std::vector<PLYProperty> features; //!< Vector which holds element properties corresponding to PM features
		std::vector<PLYProperty> descriptors; //!< Vector which holds element properties corresponding to PM descriptors

		virtual PMPropTypes getPMType(const PLYProperty& prop) const = 0; //! return the relation to pointmatcher
	};


	//! Implementation of PLY vertex element
	class PLYVertex : public PLYElement
	{
	public:

		PLYVertex(const std::string& name, const unsigned num, const unsigned offset) : PLYElement(name, num, offset) {} //! ctor

		typename PLYElement::PMPropTypes getPMType(const PLYProperty& prop) const; //! implements element interface.
	};

	//! Factory for PLY elements
	class PLYElementF
	{
		enum ElementTypes
		{
			VERTEX,
			UNSUPPORTED
		};

		static ElementTypes getElementType(const std::string& elem_name);
	public:
		bool elementSupported(const std::string& elem_name);
		static PLYElement* createElement(const std::string& elem_name, const int elem_num, const unsigned offset); //! factory function, build element defined by name with elem_num elements
	};

};

#endif // __POINTMATCHER_IO_H
