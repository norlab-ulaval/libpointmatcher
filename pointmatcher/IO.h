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
	typedef typename PointMatcher<T>::DataPoints::Label Label; //!< alias
	typedef typename PointMatcher<T>::DataPoints::Labels Labels; //!< alias

	//! Pair feature column, feature name
	typedef std::pair<int, std::string >LabelAssociationPair;

	//! Map to associate common descriptor sublabels to PM descriptor matrix row and labels
	//! ex: nx, ny, nz are associated with (0,normals) (1,normals) (2,normals) respectively
	typedef std::map<std::string, LabelAssociationPair > SublabelAssociationMap;

	// General
	static SublabelAssociationMap getFeatAssocationMap(); //!< map to store association between common 1d feature labels and their PM label and span dimension
	static SublabelAssociationMap getDescAssocationMap(); //!< map to store association between common 1d descriptor labels and their PM label and span dimension
	static bool featSublabelRegistered(const std::string& externalName); //!< returns true if a particular feature dim label is registered (ie x, y...)
	static bool descSublabelRegistered(const std::string& externalName); //!< returns true if a particular descriptor dim label is registered (ie nx, red...)
	static LabelAssociationPair getFeatAssociationPair(const std::string& externalName); //!< get PM feature label associated with sublabel
	static LabelAssociationPair getDescAssociationPair(const std::string& externalName); //!< get PM descriptor label associated with sublabel

	static std::string getColLabel(const Label& label, const int row); //!< convert a descriptor label to an appropriate sub-label
	
	//! Type of information in a DataPoints. Each type is stored in its own dense matrix.
	enum PMPropTypes
		{
			FEATURE,
			DESCRIPTOR,
			UNSUPPORTED
		};//TODO: add time here too

	//! Structure containing all information required to map external information to PointMatcher internal representation
	struct SupportedLabel
	{
		const std::string internalName; //!< name used in PointMatcher
		const std::string externalName; //!< name used in external format
		const PMPropTypes type; //!< type of information in PointMatcher

		//! Constructor
		SupportedLabel(const std::string& internalName, const std::string& externalName, const PMPropTypes& type);
	};

	//! Vector of supported labels in PointMatcher and their external names
	typedef std::vector<SupportedLabel> SupportedLabels;

	//! Vector containing the mapping of all external names to PointMatcher representation.
	//! The order is important (i.e., nx before ny). This can also be used to remap 
	//! 1D descriptor name to a better one.
	static SupportedLabels getSupportedExternalLabels()
	{
		//Can be read:
		// (internalName, externalName, type)
		const SupportedLabels labels = boost::assign::list_of<SupportedLabel>
			("x", "x", FEATURE)
			("y", "y", FEATURE)
			("z", "z", FEATURE)
			("pad", "pad", FEATURE)
			("normals", "nx", DESCRIPTOR)
			("normals", "ny", DESCRIPTOR)
			("normals", "nz", DESCRIPTOR)
			("normals", "normal_x", DESCRIPTOR)
			("normals", "normal_y", DESCRIPTOR)
			("normals", "normal_z", DESCRIPTOR)
			("color", "red", DESCRIPTOR)
			("color", "green", DESCRIPTOR)
			("color", "blue", DESCRIPTOR)
			("color", "alpha", DESCRIPTOR)
			("eigValues", "eigValues0", DESCRIPTOR)
			("eigValues", "eigValues1", DESCRIPTOR)
			("eigValues", "eigValues2", DESCRIPTOR)
			("eigVectors", "eigVectors0X", DESCRIPTOR)
			("eigVectors", "eigVectors0Y", DESCRIPTOR)
			("eigVectors", "eigVectors0Z", DESCRIPTOR)
			("eigVectors", "eigVectors1X", DESCRIPTOR)
			("eigVectors", "eigVectors1Y", DESCRIPTOR)
			("eigVectors", "eigVectors1Z", DESCRIPTOR)
			("eigVectors", "eigVectors2X", DESCRIPTOR)
			("eigVectors", "eigVectors2Y", DESCRIPTOR)
			("eigVectors", "eigVectors2Z", DESCRIPTOR)
			//("", "", DESCRIPTOR)
			;

			return labels;
	}

	//! Generate a vector of Labels by checking for collision is the same name is reused.
	class LabelGenerator
	{
		Labels labels; //!< vector of labels used to cumulat information

	public:
		//! add a name to the vector of labels. If already there, will increament the dimension.
		void add(std::string internalName);

		//! Return the vector of labels used to build a DataPoints
		Labels getLabels() const;
	};


	//! Associate an external name to a DataPoints type of information
	static PMPropTypes getPMType(const std::string& externalName); //! Return the type of information specific to a DataPoints based on a sulabel name

	// CSV
	static DataPoints loadCSV(const std::string& fileName);
	static DataPoints loadCSV(std::istream& is);

	static void saveCSV(const DataPoints& data, const std::string& fileName);
	static void saveCSV(const DataPoints& data, std::ostream& os);

	// VTK
	//! Enumeration of legacy VTK data types that can be parsed
	enum SupportedVTKDataTypes
	{
		POLYDATA,
		UNSTRUCTURED_GRID
	};

	static DataPoints loadVTK(const std::string& fileName);
	static DataPoints loadVTK(std::istream& is);

	static void saveVTK(const DataPoints& data, const std::string& fileName);

	// PLY
	static DataPoints loadPLY(const std::string& fileName);
	static DataPoints loadPLY(std::istream& is);

	static void savePLY(const DataPoints& data, const std::string& fileName); //!< save datapoints to PLY point cloud format

	// PCD
	static DataPoints loadPCD(const std::string& fileName);
	static DataPoints loadPCD(std::istream& is);

	static void savePCD(const DataPoints& data, const std::string& fileName); //!< save datapoints to PCD point cloud format

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

	//! A structure to hold information about descriptors contained in a CSV file
	struct CsvDescriptor {
		std::string name; //!< name of descriptor
		unsigned 	start_col; //!< column number at which descriptor starts
		unsigned 	span; //!< number of columns spanned by descriptor
	};

	//! Check that property defined by type is a valid PLY type note: type must be lowercase
	static bool plyPropTypeValid (const std::string& type);

	//! Interface for PLY property
	struct PLYProperty
	{
		//PLY information:
		std::string name; //!< name of PLY property
		std::string type; //!< type of PLY property
		std::string idx_type; //!< for list properties, type of number of elements
		unsigned pos; //!< index of the property in element
		bool is_list; //!< member is true of property is a list
		
		//PointMatcher information:
		PMPropTypes pmType; //!< type of information in PointMatcher
		int pmRowID; //!< row id used in a DataPoints 

		//TODO: remove that (now pmType):
		bool is_feature; //!<member is true if is a PM feature, if not, it is a descriptor

		PLYProperty() { } //!< Default constructor. If used member values must be filled later.

		// regular property
		PLYProperty(const std::string& type, const std::string& name, const unsigned pos, const bool is_feature = false);

		// list property
		PLYProperty(const std::string& idx_type, const std::string& type, const std::string& name, const unsigned pos, const bool is_feature = false); //! list prop ctor

		bool operator==(const PLYProperty& other) const; //! compare with other property
	};

	//! Map from a descriptor name to a list PLY property
	//! ex: "normals" -> nx, ny ,nz
	typedef std::map<std::string, std::vector<PLYProperty> > PLYDescPropMap;
	
	//! Vector of properties specific to PLY files
	typedef std::vector<PLYProperty> PLYProperties;

	//! Iterator for a vector of PLY properties
	typedef typename PLYProperties::iterator it_PLYProp;

	//! Interface for all PLY elements. 
	class PLYElement
	{
	public:
		std::string name; //!< name identifying the PLY element
		unsigned num; //!< number of occurences of the element
		unsigned total_props; //!< total number of properties in PLY element
		unsigned offset; //!< line at which data starts
		PLYProperties properties; //!< all properties found in the header
		unsigned nbFeatures; //!< number of valid features found in the header
		unsigned nbDescriptors; //!< number of valid descriptors found in the header


		//! PLY Element constructor
		/**
			@param name name of the ply element (case-sensitive)
			@param num number of times the element appears in the file
			@param offset if there are several elements, the line offset at which this element begins.  Note that, as of writing, only one (vertex) element is supported.

			This object holds information about a PLY element contained in the file.
			It is filled out when reading the header and used when parsing the data.
		 */
		PLYElement(const std::string& name, const unsigned num, const unsigned offset) :
			name(name), num(num), total_props(0), offset(offset), nbFeatures(0), nbDescriptors(0) {}

		bool supportsProperty(const PLYProperty& prop) const; //!< Returns true if property pro is supported by element

		void addProperty(PLYProperty& prop); //!< add a property to vector of properties

		bool operator==(const PLYElement& other) const; //!< comparison operator for elements

	};


	//! Implementation of PLY vertex element
	class PLYVertex : public PLYElement
	{
	public:
		//! Constructor
		/**
					@param num number of times the element appears in the file
					@param offset if there are several elements, the line offset at which this element begins.  Note that, as of writing, only one (vertex) element is supported.

					Implementation of PLY element interface for the vertex element
		 */
		PLYVertex(const unsigned num, const unsigned offset) : PLYElement("vertex", num, offset) {}

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
		bool elementSupported(const std::string& elem_name); //!< returns true if element named elem_name is supported by this parser
		static PLYElement* createElement(const std::string& elem_name, const int elem_num, const unsigned offset); //!< factory function, build element defined by name with elem_num elements
	};
   
   //! Replaces getline for handling windows style CR/LF line endings
   static std::istream & safeGetLine( std::istream& os, std::string & line);
   
};


#endif // __POINTMATCHER_IO_H
