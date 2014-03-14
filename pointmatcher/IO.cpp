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

#include "IO.h"
#include "InspectorsImpl.h"

// For logging
#include "PointMatcherPrivate.h"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <ctype.h>
#include "boost/algorithm/string.hpp"
#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/lexical_cast.hpp"

#ifdef WIN32
#define strtok_r strtok_s
#endif // WIN32

using namespace std;
using namespace PointMatcherSupport;


// Tokenize a string, excepted if it begins with a '%' (a comment in CSV)
static std::vector<string> csvLineToVector(const char* line)
{
	std::vector<string> parsedLine;
	char delimiters[] = " \t,;";
	char *token;
	char tmpLine[1024];
	char *brkt = 0;
	strcpy(tmpLine, line);
	token = strtok_r(tmpLine, delimiters, &brkt);
	if(line[0] != '%') // Jump line if it's commented
	{
		while (token)
		{
			parsedLine.push_back(string(token));
			token = strtok_r(NULL, delimiters, &brkt);
		}
	}

	return parsedLine;
}

// Open and parse a CSV file, return the data
CsvElements parseCsvWithHeader(const std::string& fileName)
{
	validateFile(fileName);
	
	ifstream is(fileName.c_str());

	unsigned elementCount=0;
	std::map<string, unsigned> keywordCols;
	CsvElements data;

	bool firstLine(true);
	unsigned lineCount=0;
	while (!is.eof())
	{
		char line[1024];
		is.getline(line, sizeof(line));
		line[sizeof(line)-1] = 0;

		if(firstLine)
		{
			std::vector<string> header = csvLineToVector(line);
				
			elementCount = header.size();
			for(unsigned int i = 0; i < elementCount; i++)
			{
				keywordCols[header[i]] = i;
			}

			firstLine = false;
		}
		else // load the rest of the file
		{
			std::vector<string> parsedLine = csvLineToVector(line);
			if(parsedLine.size() != elementCount && parsedLine.size() !=0)
			{
				stringstream errorMsg;
				errorMsg << "Error at line " << lineCount+1 << ": expecting " << elementCount << " columns but read " << parsedLine.size() << " elements.";
				throw runtime_error(errorMsg.str());	
			}

			for(unsigned int i = 0; i < parsedLine.size(); i++)
			{
				for(BOOST_AUTO(it,keywordCols.begin()); it!=keywordCols.end(); it++)
				{
					if(i == (*it).second)
					{
						data[(*it).first].push_back(parsedLine[i]);	
					}
				}
			}
		}

		lineCount++;
	}
	
	// Use for debug
	
	//for(BOOST_AUTO(it,data.begin()); it!=data.end(); it++)
	//{
	//	cout << "--------------------------" << endl;
	//	cout << "Header: |" << (*it).first << "|" << endl;
	//	//for(unsigned i=0; i<(*it).second.size(); i++)
	//	//{
	//	//	cout << (*it).second[i] << endl;
	//	//}
	//}
	

	return data;
}


//! Constructor, leave fields blank if unused
template<typename T>
PointMatcherIO<T>::FileInfo::FileInfo(const std::string& readingFileName, const std::string& referenceFileName, const std::string& configFileName, const TransformationParameters& initialTransformation, const TransformationParameters& groundTruthTransformation, const Vector& grativity):
	readingFileName(readingFileName),
	referenceFileName(referenceFileName),
	configFileName(configFileName),
	initialTransformation(initialTransformation),
	groundTruthTransformation(groundTruthTransformation),
	gravity(gravity)
{}

template struct PointMatcherIO<float>::FileInfo;
template struct PointMatcherIO<double>::FileInfo;

// Empty constructor
template<typename T>
PointMatcherIO<T>::FileInfoVector::FileInfoVector()
{
}

//! Load a vector of FileInfo from a CSV file.
/**
	@param fileName name of the CSV file
	@param dataPath path relative to which the point cloud CSV or VTK will be resolved
	@param configPath path relative to which the yaml configuration files will be resolved
	
	The first line of the CSV file must contain a header. The supported tags are:
	- reading: file name of the reading point cloud
	- reference: file name of the reference point cloud
	- config: file name of the YAML configuration of the ICP chain
	- iTxy: initial transformation, coordinate x,y
	- gTxy: ground-truth transformation, coordinate x,y
	Note that the header must at least contain "reading".
*/
template<typename T>
PointMatcherIO<T>::FileInfoVector::FileInfoVector(const std::string& fileName, std::string dataPath, std::string configPath)
{
	if (dataPath.empty())
	{
		#if BOOST_FILESYSTEM_VERSION >= 3
		dataPath = boost::filesystem::path(fileName).parent_path().string();
		#else
		dataPath = boost::filesystem::path(fileName).parent_path().file_string();
		#endif
	}
	if (configPath.empty())
	{
		#if BOOST_FILESYSTEM_VERSION >= 3
		configPath = boost::filesystem::path(fileName).parent_path().string();
		#else
		configPath = boost::filesystem::path(fileName).parent_path().file_string();
		#endif
	}
	
	const CsvElements data = parseCsvWithHeader(fileName);
	
	// Look for transformations
	const bool found3dInitialTrans(findTransform(data, "iT", 3));
	bool found2dInitialTrans(findTransform(data, "iT", 2));
	const bool found3dGroundTruthTrans(findTransform(data, "gT", 3));
	bool found2dGroundTruthTrans(findTransform(data, "gT", 2));
	if (found3dInitialTrans)
		found2dInitialTrans = false;
	if (found3dGroundTruthTrans)
		found2dGroundTruthTrans = false;
	
	// Check for consistency
	if (found3dInitialTrans && found2dGroundTruthTrans)
		throw runtime_error("Initial transformation is in 3D but ground-truth is in 2D");
	if (found2dInitialTrans && found3dGroundTruthTrans)
		throw runtime_error("Initial transformation is in 2D but ground-truth is in 3D");
	CsvElements::const_iterator readingIt(data.find("reading"));
	if (readingIt == data.end())
		throw runtime_error("Error transfering CSV to structure: The header should at least contain \"reading\".");
	CsvElements::const_iterator referenceIt(data.find("reference"));
	CsvElements::const_iterator configIt(data.find("config"));
	
	// Load reading
	const std::vector<string>& readingFileNames = readingIt->second;
	const unsigned lineCount = readingFileNames.size();
	boost::optional<std::vector<string> > referenceFileNames;
	boost::optional<std::vector<string> > configFileNames;
	if (referenceIt != data.end())
	{
		referenceFileNames = referenceIt->second;
		assert (referenceFileNames->size() == lineCount);
	}
	if (configIt != data.end())
	{
		configFileNames = configIt->second;
		assert (configFileNames->size() == lineCount);
	}

	// for every lines
	for(unsigned line=0; line<lineCount; line++)
	{
		FileInfo info;
		
		// Files
		info.readingFileName = localToGlobalFileName(dataPath, readingFileNames[line]);
		if (referenceFileNames)
			info.referenceFileName = localToGlobalFileName(dataPath, (*referenceFileNames)[line]);
		if (configFileNames)
			info.configFileName = localToGlobalFileName(configPath, (*configFileNames)[line]);
		
		// Load transformations
		if(found3dInitialTrans)
			info.initialTransformation = getTransform(data, "iT", 3, line);
		if(found2dInitialTrans)
			info.initialTransformation = getTransform(data, "iT", 2, line);
		if(found3dGroundTruthTrans)
			info.groundTruthTransformation = getTransform(data, "gT", 3, line);
		if(found2dGroundTruthTrans)
			info.groundTruthTransformation = getTransform(data, "gT", 2, line);
		
		// Build the list
		this->push_back(info);
	}
	
	// Debug: Print the list
	/*for(unsigned i=0; i<list.size(); i++)
	{
		cout << "\n--------------------------" << endl;
		cout << "Sequence " << i << ":" << endl;
		cout << "Reading path: " << list[i].readingFileName << endl;
		cout << "Reference path: " << list[i].referenceFileName << endl;
		cout << "Extension: " << list[i].fileExtension << endl;
		cout << "Tranformation:\n" << list[i].initialTransformation << endl;
		cout << "Grativity:\n" << list[i].gravity << endl;
	}
	*/
}

//! Join parentPath and fileName and return the result as a global path
template<typename T>
std::string PointMatcherIO<T>::FileInfoVector::localToGlobalFileName(const std::string& parentPath, const std::string& fileName)
{
	std::string globalFileName(fileName);
	if (!boost::filesystem::exists(globalFileName))
	{
		const boost::filesystem::path globalFilePath(boost::filesystem::path(parentPath) /  boost::filesystem::path(fileName));
		#if BOOST_FILESYSTEM_VERSION >= 3
		globalFileName = globalFilePath.string();
		#else
		globalFileName = globalFilePath.file_string();
		#endif
	}
	validateFile(globalFileName);
	return globalFileName;
}

//! Return whether there is a valid transformation named prefix in data
template<typename T>
bool PointMatcherIO<T>::FileInfoVector::findTransform(const CsvElements& data, const std::string& prefix, unsigned dim)
{
	bool found(true);
	for(unsigned i=0; i<dim+1; i++)
	{
		for(unsigned j=0; j<dim+1; j++)
		{
			stringstream transName;
			transName << prefix << i << j;
			found = found && (data.find(transName.str()) != data.end());
		}
	}
	return found;
}

//! Return the transformation named prefix from data
template<typename T>
typename PointMatcherIO<T>::TransformationParameters PointMatcherIO<T>::FileInfoVector::getTransform(const CsvElements& data, const std::string& prefix, unsigned dim, unsigned line)
{
	TransformationParameters transformation(TransformationParameters::Identity(dim+1, dim+1));
	for(unsigned i=0; i<dim+1; i++)
	{
		for(unsigned j=0; j<dim+1; j++)
		{
			stringstream transName;
			transName << prefix << i << j;
			CsvElements::const_iterator colIt(data.find(transName.str()));
			const T value = boost::lexical_cast<T> (colIt->second[line]);
			transformation(i,j) = value;
		}
	}
	return transformation;
}

template struct PointMatcherIO<float>::FileInfoVector;
template struct PointMatcherIO<double>::FileInfoVector;

//! Throw a runtime_error exception if fileName cannot be opened
void PointMatcherSupport::validateFile(const std::string& fileName)
{
	boost::filesystem::path fullPath(fileName);

	ifstream ifs(fileName.c_str());
	if (!ifs.good())
	#if BOOST_FILESYSTEM_VERSION >= 3
		#if BOOST_VERSION >= 105000
				throw runtime_error(string("Cannot open file ") + boost::filesystem::complete(fullPath).generic_string());
		#else
				throw runtime_error(string("Cannot open file ") + boost::filesystem3::complete(fullPath).generic_string());
		#endif
	#else
		throw runtime_error(string("Cannot open file ") + boost::filesystem::complete(fullPath).native_file_string());
    #endif
}


//! Load a point cloud from a file, determine format from extension
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::DataPoints::load(const std::string& fileName)
{
	const boost::filesystem::path path(fileName);
	const string& ext(boost::filesystem::extension(path));
	if (boost::iequals(ext, ".vtk"))
		return PointMatcherIO<T>::loadVTK(fileName);
	else if (boost::iequals(ext, ".csv"))
		return PointMatcherIO<T>::loadCSV(fileName);
	else if (boost::iequals(ext, ".ply"))
		return PointMatcherIO<T>::loadPLY(fileName);
	else
		throw runtime_error("loadAnyFormat(): Unknown extension \"" + ext + "\" for file \"" + fileName + "\", extension must be either \".vtk\" or \".csv\"");
}

template
PointMatcher<float>::DataPoints PointMatcher<float>::DataPoints::load(const std::string& fileName);
template
PointMatcher<double>::DataPoints PointMatcher<double>::DataPoints::load(const std::string& fileName);


//! @brief Load comma separated values (csv) file
//! @param fileName a string containing the path and the file name
//! 
//! This loader has 3 behaviors since there is no official standard for
//! csv files. A 2D or 3D point cloud will be created automatically if:
//!   - there is a header with columns named x, y and optionnaly z
//!   - there are only 2 or 3 columns in the file
//!
//! Otherwise, the user is asked to enter column id manually which might 
//! block automatic processing.
//!
//! @todo Add support to load descriptors (ex. color, ids, etc.)
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcherIO<T>::loadCSV(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	
	validateFile(fileName);

	return loadCSV(ifs);
}

//! @brief Load comma separated values (csv) file
//! @see loadCSV()
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcherIO<T>::loadCSV(std::istream& is)
{
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	
	vector<T> xData;
	vector<T> yData;
	vector<T> zData;
	vector<T> padData;
	vector<string> header;
	int dim(0);
	bool firstLine(true);
	bool hasHeader(false);
	Labels labels;
	int xCol(-1);
	int yCol(-1);
	int zCol(-1);

	char delimiters[] = " \t,;";
	char *token;
	while (!is.eof())
	{
		char line[1024];
		is.getline(line, sizeof(line));
		line[sizeof(line)-1] = 0;
	
		// Look for text header
		unsigned int len = strspn(line, " ,+-.1234567890Ee");
		if(len != strlen(line))
		{
			//cout << "Header detected" << endl;
			hasHeader = true;
		}
		else
		{
			hasHeader = false;
		}

		// Count dimension using first line
		if(firstLine)
		{
			char tmpLine[1024];
			char *brkt = 0;
			strcpy(tmpLine, line);
			token = strtok_r(tmpLine, delimiters, &brkt);
			while (token)
			{
				dim++;
				
				// Load text header
				if(hasHeader)
				{
					header.push_back(string(token));
				}

				token = strtok_r(NULL, delimiters, &brkt);
			}
		
				
			if (hasHeader)
			{
				// Search for x, y and z tags
				for(unsigned int i = 0; i < header.size(); i++)
				{
					if(header[i].compare("x") == 0)
						xCol = i;
				
					if(header[i].compare("y") == 0)
						yCol = i;
				
					if(header[i].compare("z") == 0)
						zCol = i;
				}
				if(xCol == -1 || yCol == -1)
				{
					for(unsigned int i = 0; i < header.size(); i++)
					{
						cout << "(" << i << ") " << header[i] << endl;
					}
					cout << endl << "Enter ID for x: ";
					cin >> xCol;
					cout << "Enter ID for y: ";
					cin >> yCol;
					cout << "Enter ID for z (-1 if 2D data): ";
					cin >> zCol;
				}
			}
			else
			{
				// Check if it is a simple file with only coordinates
				if (!(dim == 2 || dim == 3))
				{
					cout << "WARNING: " << dim << " columns detected. Not obivious which columns to load for x, y or z." << endl;
					cout << endl << "Enter column ID (starting from 0) for x: ";
					cin >> xCol;
					cout << "Enter column ID (starting from 0) for y: ";
					cin >> yCol;
					cout << "Enter column ID (starting from 0, -1 if 2D data) for z: ";
					cin >> zCol;
				}
				else
				{
					// Assume logical order...
					xCol = 0;
					yCol = 1;
					if(dim == 3)
						zCol = 2;
				}
			}

			if(zCol != -1)
				dim = 3;
			else
				dim = 2;
		}

		// Load data!
		char *brkt = 0;
		token = strtok_r(line, delimiters, &brkt);
		int currentCol = 0;
		while (token)
		{
			// Load data only if no text is on the line
			if(!hasHeader)
			{
				if(currentCol == xCol)
					xData.push_back(atof(token));
				if(currentCol == yCol)
					yData.push_back(atof(token));
				if(currentCol == zCol)
					zData.push_back(atof(token));
			}

			token = strtok_r(NULL, delimiters, &brkt);
			currentCol++;
		}
		
		// Add one for uniform coordinates
		padData.push_back(1);
		
		if (firstLine)
		{
			// create standard labels
			for (int i=0; i < dim; i++)
			{
				string text;
				text += char('x' + i);
				labels.push_back(Label(text, 1));
			}
			labels.push_back(Label("pad", 1));
		}

		firstLine = false;
	}

	assert(xData.size() == yData.size());
	int nbPoints = xData.size();

	// Transfer loaded points in specific structure (eigen matrix)
	Matrix features(dim+1, nbPoints);
	for(int i=0; i < nbPoints; i++)
	{
		features(0,i) = xData[i];
		features(1,i) = yData[i];
		if(dim == 3)
		{
			features(2,i) = zData[i];
			features(3,i) = 1;
		}
		else
		{
			features(2,i) = 1;
		}
	}
	
	DataPoints dataPoints(features, labels);
	//cout << "Loaded " << dataPoints.features.cols() << " points." << endl;
	//cout << "Find " << dataPoints.features.rows() << " dimensions." << endl;
	//cout << features << endl;

	return dataPoints;
}

template
PointMatcher<float>::DataPoints PointMatcherIO<float>::loadCSV(const std::string& fileName);
template
PointMatcher<double>::DataPoints PointMatcherIO<double>::loadCSV(const std::string& fileName);

//! Save a point cloud to a file, determine format from extension
template<typename T>
void PointMatcher<T>::DataPoints::save(const std::string& fileName) const
{
	const boost::filesystem::path path(fileName);
	const string& ext(boost::filesystem::extension(path));
	if (boost::iequals(ext, ".vtk"))
		return PointMatcherIO<T>::saveVTK(*this, fileName);
	else if (boost::iequals(ext, ".csv"))
		return PointMatcherIO<T>::saveCSV(*this, fileName);
	else if (boost::iequals(ext, ".ply"))
		return PointMatcherIO<T>::savePLY(*this, fileName);
	else
		throw runtime_error("saveAnyFormat(): Unknown extension \"" + ext + "\" for file \"" + fileName + "\", extension must be either \".vtk\" or \".csv\"");
}

template
void PointMatcher<float>::DataPoints::save(const std::string& fileName) const;
template
void PointMatcher<double>::DataPoints::save(const std::string& fileName) const;

//! Save point cloud to a file as CSV
template<typename T>
void PointMatcherIO<T>::saveCSV(const DataPoints& data, const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	saveCSV(data, ofs);
}

//! Save point cloud to a stream as CSV
template<typename T>
void PointMatcherIO<T>::saveCSV(const DataPoints& data, std::ostream& os)
{
	const int pointCount(data.features.cols());
	const int dimCount(data.features.rows());
	
	if (pointCount == 0)
	{
		cerr << "Warning, no points, doing nothing" << endl;
		return;
	}
	
	// write points
	for (int p = 0; p < pointCount; ++p)
	{
		for (int i = 0; i < dimCount-1; ++i)
		{
			os << data.features(i, p);
			if(i != dimCount-2)
				os << " , ";
		}
		os << "\n";
	}
	
}

template
void PointMatcherIO<float>::saveCSV(const DataPoints& data, const std::string& fileName);
template
void PointMatcherIO<double>::saveCSV(const DataPoints& data, const std::string& fileName);

//! Load point cloud from a file as VTK
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcherIO<T>::loadVTK(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadVTK(ifs);
}

//! Load point cloud from a stream as VTK
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcherIO<T>::loadVTK(std::istream& is)
{
	//typedef typename DataPoints::Label Label;
	//typedef typename DataPoints::Labels Labels;
	
	DataPoints loadedPoints;

	// parse header
	string line;
	getline(is, line);
	if (line.find("# vtk DataFile Version") != 0)
		throw runtime_error(string("Wrong magic header, found ") + line);
	getline(is, line);
	getline(is, line);
	if (line != "ASCII")
		throw runtime_error(string("Wrong file type, expecting ASCII, found ") + line);
	getline(is, line);
	if (line != "DATASET POLYDATA")
		throw runtime_error(string("Wrong data type, expecting DATASET POLYDATA, found ") + line);


	// parse points and descriptors
	string fieldName;
	string name;
	int pointCount = 0;
	string type;
	while (is.good())
	{
		is >> fieldName;
		
		// load features
		if(fieldName == "POINTS")
		{
			is >> pointCount;
			is >> type;
			
			if(!(type == "float" || type == "double"))
					throw runtime_error(string("Field POINTS can only be of type double or float"));

			Matrix features(4, pointCount);
			for (int p = 0; p < pointCount; ++p)
			{
				is >> features(0, p);
				is >> features(1, p);
				is >> features(2, p);
				features(3, p) = 1.0;
			}
			loadedPoints.addFeature("x", features.row(0));
			loadedPoints.addFeature("y", features.row(1));
			loadedPoints.addFeature("z", features.row(2));
			loadedPoints.addFeature("pad", features.row(3));
		}
		else if(fieldName == "VERTICES")
		{
			int size;
			int verticeSize;
			is >> size >> verticeSize;
			// Skip vertice definition
			for (int p = 0; p < pointCount; p++)
			{
				getline(is, line); 
				if(line == "")
					p--;
			}
		}
		else if(fieldName == "POINT_DATA")
		{
			int descriptorCount;
			is >> descriptorCount;
			if(pointCount != descriptorCount)
				throw runtime_error(string("The size of POINTS is different than POINT_DATA"));
		}
		else // Load descriptors
		{
			// descriptor name
			is >> name;

			int dim = 0;
			bool skipLookupTable = false;
			if(fieldName == "SCALARS")
			{
				dim = 1;
				is >> type;
				skipLookupTable = true;
			}
			else if(fieldName == "VECTORS")
			{
				dim = 3;
				is >> type;
			}
			else if(fieldName == "TENSORS")
			{
				dim = 9;
				is >> type;
			}
			else if(fieldName == "NORMALS")
			{
				dim = 3;
				is >> type;
			}
			else if(fieldName == "COLOR_SCALARS")
			{
				is >> dim;
				type = "float";
			}
			else
				throw runtime_error(string("Unknown field name " + fieldName + ", expecting SCALARS, VECTORS, TENSORS, NORMALS or COLOR_SCALARS."));

			
			if(!(type == "float" || type == "double"))
					throw runtime_error(string("Field " + fieldName + " is " + type + " but can only be of type double or float"));
					 
			// Skip LOOKUP_TABLE line
			if(skipLookupTable)
			{
				//FIXME: FP - why the first line is aways empty?
				getline(is, line); 
				getline(is, line); 
			}

			Matrix descriptor(dim, pointCount);
			for (int p = 0; p < pointCount; ++p)
			{
				for(int d = 0; d < dim; d++)
				{
					is >> descriptor(d, p);
				}
			}
			loadedPoints.addDescriptor(name, descriptor);
		}
			 
	}
	
	return loadedPoints;
}

template
PointMatcherIO<float>::DataPoints PointMatcherIO<float>::loadVTK(const std::string& fileName);
template
PointMatcherIO<double>::DataPoints PointMatcherIO<double>::loadVTK(const std::string& fileName);


//! Save point cloud to a file as VTK
template<typename T>
void PointMatcherIO<T>::saveVTK(const DataPoints& data, const std::string& fileName)
{
	typedef typename InspectorsImpl<T>::VTKFileInspector VTKInspector;
	
	Parametrizable::Parameters param;
	boost::assign::insert(param) ("baseFileName", "");
	VTKInspector vtkInspector(param);
	vtkInspector.dumpDataPoints(data, fileName);
}


template
void PointMatcherIO<float>::saveVTK(const PointMatcherIO<float>::DataPoints& data, const std::string& fileName);
template
void PointMatcherIO<double>::saveVTK(const PointMatcher<double>::DataPoints& data, const std::string& fileName);

//! @brief Load polygon file format (ply) file
//! @param fileName a string containing the path and the file name
//!
//! Note that the PLY does not define a standard for point clouds
//! Only PLY files with elements named "vertex" are supported
//! "vertex" should have 2 or 3 properties names "x", "y", "z" to define features.
//!
//! The following additional properties may be defined and will be added as descriptors
//! nx, ny, nz: components of a normal vector
//!
template<typename T>
typename PointMatcherIO<T>::DataPoints PointMatcherIO<T>::loadPLY(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadPLY(ifs);
}

template
PointMatcherIO<float>::DataPoints PointMatcherIO<float>::loadPLY(const string& fileName);
template
PointMatcherIO<double>::DataPoints PointMatcherIO<double>::loadPLY(const string& fileName);

//! @brief Load polygon file format (PLY) file
//! @see loadPLY()
template <typename T>
typename PointMatcherIO<T>::DataPoints PointMatcherIO<T>::loadPLY(std::istream& is)
{
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef vector<PLYElement*> Elements;

	///////////////////////////
	// PARSE PLY HEADER
	bool format_defined = false;
	bool header_processed = false;

	Elements elements;
	PLYElementF element_f; // factory
	PLYElement* current_element = NULL;
	bool skip_props = false; // flag to skip properties if element is not supported
	unsigned elem_offset = 0; // keep track of line position of elements that are supported
	string line;
	getline(is, line);

	if (line.find("ply") != 0) {
		throw runtime_error(string("PLY parse error: wrong magic header, found ") + line);
	}

	while (!header_processed)
	{
		if(!getline(is, line))
			throw runtime_error("PLY parse error: reached end of file before end of header definition");

		if ( line.empty() )
			continue;
		istringstream stringstream (line);

		string keyword;
		stringstream >> keyword;

		// ignore comment
		if (keyword == "comment") {
			continue;
		}

		if (keyword == "format")
		{
			if (format_defined)
				throw runtime_error("PLY parse error: format already defined");

			string format_str, version_str;
			stringstream >> format_str >> version_str;

			if (format_str != "ascii" && format_str != "binary_little_endian" && format_str != "binary_big_endian")
				throw runtime_error(string("PLY parse error: format ") + format_str + string(" is not supported"));

			if (version_str != "1.0")
			{
				throw runtime_error(string("PLY parse error: version ") + version_str + string(" of ply is not supported"));
			}

			format_defined = true;

		}
		else if (keyword == "element")
		{
			if (current_element != NULL && current_element->getNumSupportedProperties() == 0)
			{
				LOG_WARNING_STREAM("PLY parse warning: no supported properties in element " << current_element->name);
			}

			string elem_name, elem_num_s;
			stringstream >> elem_name >> elem_num_s;

			unsigned elem_num;
			try
			{
				elem_num = boost::lexical_cast<unsigned>(elem_num_s);
			}
			catch (boost::bad_lexical_cast& e)
			{
				throw runtime_error(string("PLY parse error: bad number of elements ") + elem_num_s + string(" for element ") + elem_name);
			}

			if (element_f.elementSupported(elem_name))
			{
				// Initialize current element
				PLYElement* elem = element_f.createElement(elem_name, elem_num, elem_offset);
				current_element = elem;

				// check that element is not already defined
				for (typename Elements::const_iterator it = elements.begin(); it != elements.end(); it++ )
				{
					if (**it == *elem) {
						throw runtime_error(string("PLY parse error: element: ") + elem_name + string( "is already defined"));
					}
				}
				elements.push_back(elem);
				skip_props = false;
			}
			else
			{
				LOG_WARNING_STREAM("PLY parse warning: element " << elem_name << " not supported. Skipping.");
				skip_props = true;
			}

			elem_offset += elem_num;
		}
		else if (keyword == "property")
		{
			if (current_element == NULL)
			{
				throw runtime_error("PLY parse error: property listed without defining an element");
			}

			if (skip_props)
				continue;

			string next, prop_type, prop_name;
			stringstream >> next;

			// PLY list property
			if (next == "list")
			{
				string prop_idx_type;
				stringstream >> prop_idx_type >> prop_type >> prop_name;

				PLYProperty list_prop(prop_idx_type, prop_type, prop_name, current_element->total_props);

				if (current_element->supportsProperty(list_prop))
				{
					current_element->addProperty(list_prop);
				}
				else
				{
					LOG_WARNING_STREAM("PLY parse error: element " << current_element->name
							<< " does not support property " << prop_name << " " << prop_type  );
				}
			}
			// PLY regular property
			else
			{
				prop_type = next;
				stringstream >> prop_name;
				PLYProperty prop(prop_type, prop_name, current_element->total_props);

				if (current_element->supportsProperty(prop))
				{
					current_element->addProperty(prop);
				}
				else
				{
					LOG_WARNING_STREAM("PLY parse error: element " << current_element->name <<
							" does not support property " << prop_name << " " << prop_type  );
				}
			}

			current_element->total_props++;
		}
		else if (keyword == "end_header")
		{
			if (!format_defined)
			{
				throw runtime_error(string("PLY parse error: format not defined in header"));
			}

			if (elements.size() == 0)
			{
				throw runtime_error(string("PLY parse error: no elements defined in header"));
			}

			if (current_element != NULL && current_element->getNumSupportedProperties() == 0)
			{
				LOG_WARNING_STREAM("PLY parse warning: no supported properties in element " << current_element->name);
			}

			header_processed = true;
		}
	}

	///////////////////////////
	// PARSE PLY DATA
	Matrix features, descriptors;
	Labels feat_labels, desc_labels;

	int l = 0; // number of elements instances that were processed
	for (typename Elements::const_iterator el_it = elements.begin(); el_it != elements.end(); el_it++)
	{
		vector<PLYProperty> feature_props = (*el_it)->getFeatureProps();
		vector<PLYProperty> descriptor_props = (*el_it)->getDescriptorProps();
		const unsigned n_points = (*el_it)->num;
		const unsigned offset = (*el_it)->offset;
		const unsigned n_feat = (*el_it)->getNumFeatures();
		const unsigned n_desc = (*el_it)->getNumDescriptors();
		int feat_offset = features.rows();
		int desc_offset = descriptors.rows();

		// Get labels
		for (typename vector<PLYProperty>::const_iterator it = feature_props.begin(); it != feature_props.end(); it++ )
		{
			feat_labels.push_back(Label(it->name));
		}
		for (typename vector<PLYProperty>::const_iterator it = descriptor_props.begin(); it != descriptor_props.end(); it++ )
		{
			desc_labels.push_back(Label(it->name));
		}

		// Allocate features and descriptors matrix
		// If there are more than one element, grow the features matrix
		// to accommodate new features in this element
		features.conservativeResize(n_feat+feat_offset,n_points);
		descriptors.conservativeResize(n_desc+desc_offset,n_points);

		while (l < (offset + n_points) )
		{
			if(!getline(is, line))
				throw runtime_error("PLY parse error: expected more data lines");

			if ( line.empty() )
				continue;

			istringstream ss (line);

			string first_word, prop_s;
			ss >> first_word;

			// ignore comment
			if (first_word == "comment") {
				continue;
			}

			// move to offset line
			if (l < offset)
			{
				l++; // increment line count
				continue;
			}

			unsigned f = 0; // features dimension
			unsigned d = 0; // descriptor dimension

			for (int pr = 0; f < n_feat || d < n_desc ; pr++)
			{
				unsigned next_f = feature_props[f].pos; // get next supported feature property column
				unsigned next_d = descriptor_props[d].pos; // get next supported descriptor property column
				T prop_val;

				if (pr > 0)
				{
					if(!(ss >> prop_val))
						throw runtime_error(string("PLY parse error: at line ") + boost::lexical_cast<string>(l));
				}
				else
					prop_val = boost::lexical_cast<T>(first_word);

				if (pr == next_f)
				{
					features(f+feat_offset,l-offset) = prop_val;
					f++;
				}

				else if (pr == next_d)
				{
					descriptors(d+desc_offset,l-offset) = prop_val;
					d++;
				}
			}

			l++; // increment line count
		}
	}

	// Add homogeneous coordinates padding
	feat_labels.push_back(Label("pad"));
	features.conservativeResize(features.rows()+1,features.cols());
	features.row(features.rows()-1) = Vector::Ones(features.cols());
	DataPoints loadedPoints(features,feat_labels,descriptors,desc_labels);
	return loadedPoints;
}

template<typename T>
void PointMatcherIO<T>::savePLY(const DataPoints& data,
		const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);

	const int pointCount(data.features.cols());
	const int featCount(data.features.rows());
	const int descCount(data.descriptors.rows());

	if (pointCount == 0)
	{
		cerr << "Warning, no points, doing nothing" << endl;
		return;
	}

	ofs << "ply\n" <<"format ascii 1.0\n";
	ofs << "element vertex " << pointCount << "\n";
	for (int f=0; f <(featCount-1); f++)
	{
		ofs << "property float " << data.featureLabels[f].text << "\n";
	}

	for (int d=0; d <descCount; d++)
	{
		ofs << "property float " << data.descriptorLabels[d].text << "\n";
	}

	ofs << "end_header\n";

	// write points
	for (int p = 0; p < pointCount; ++p)
	{
		for (int f = 0; f < featCount - 1; ++f)
		{
			ofs << data.features(f, p);
			if(f != featCount-2)
				ofs << " ";
		}
		for (int d = 0; d < descCount; ++d)
		{
			ofs << data.descriptors(d, p);
			if(d != descCount-1)
				ofs << " ";
		}
		ofs << "\n";
	}

	ofs.close();
}

template
void PointMatcherIO<float>::savePLY(const DataPoints& data, const std::string& fileName);
template
void PointMatcherIO<double>::savePLY(const DataPoints& data, const std::string& fileName);

//! @(brief) Regular PLY property constructor
template<typename T>
PointMatcherIO<T>::PLYProperty::PLYProperty(const std::string& type,
		const std::string& name, const unsigned pos, const bool is_feature) :
		name(name), type(type), pos(pos), is_feature(is_feature)  {
	if (plyPropTypeValid(type))
	{
		is_list = false;
	}
	else
	{
		throw std::runtime_error(
				std::string("PLY parse error: property type ") + type
						+ std::string(" for property ") + name
						+ std::string(" is invalid"));
	}
}

//! @(brief) PLY property list constructor
template<typename T>
PointMatcherIO<T>::PLYProperty::PLYProperty(const std::string& idx_type,
		const std::string& type, const std::string& name, const unsigned pos, const bool is_feature) :
		name(name), type(type), idx_type(idx_type), pos(pos), is_feature(is_feature)
{
	if (plyPropTypeValid(idx_type) && plyPropTypeValid(type)) {
		is_list = true;
	} else
		throw std::runtime_error(
				std::string("PLY parse error: property list type ") + idx_type
						+ std::string(" ") + type
						+ std::string(" for property ") + name
						+ std::string(" is invalid"));
}

template
class PointMatcherIO<float>::PLYElement;
template
class PointMatcherIO<double>::PLYElement;

template
class PointMatcherIO<float>::PLYProperty;
template
class PointMatcherIO<double>::PLYProperty;

template <typename T>
void PointMatcherIO<T>::PLYElement::addProperty(
		PLYProperty& prop) {
	if (supportsProperty(prop))
	{
		prop.is_feature = (getPMType(prop) == FEATURE);

		if (prop.is_feature)
		{
			features.push_back(prop);
		}
		else
		{
			descriptors.push_back(prop);
		}
	}
	else
		throw std::runtime_error(
				std::string("PLY parse error: property ") + prop.name
						+ std::string(" not supported by element ") + name);
}

template <typename T>
int PointMatcherIO<T>::PLYElement::getNumFeatures() const
{
	return features.size();
}

template <typename T>
int PointMatcherIO<T>::PLYElement::getNumDescriptors() const
{
	return descriptors.size();
}

template <typename T>
const std::vector<typename PointMatcherIO<T>::PLYProperty>& PointMatcherIO<T>::PLYElement::getFeatureProps() const
{
	return features;
}

template <typename T>
const std::vector<typename PointMatcherIO<T>::PLYProperty>& PointMatcherIO<T>::PLYElement::getDescriptorProps() const
{
	return descriptors;
}

template <typename T>
size_t PointMatcherIO<T>::PLYElement::getNumSupportedProperties() const {
	return features.size() + descriptors.size() ;
}

template <typename T>
bool PointMatcherIO<T>::PLYElement::supportsProperty(const PLYProperty& prop) const
{
	return getPMType(prop) != UNSUPPORTED;
}

template<typename T>
typename PointMatcherIO<T>::PLYElement::PMPropTypes PointMatcherIO<T>::PLYVertex::getPMType(const PLYProperty& prop) const
{
	if (prop.name == "x" || prop.name == "y" || prop.name == "z")
		return this->FEATURE;
	else if (prop.name == "nx" || prop.name == "ny" || prop.name == "nz")
		return this->DESCRIPTOR;
	else
		return this->UNSUPPORTED;
}

template <typename T>
typename PointMatcherIO<T>::PLYElementF::ElementTypes PointMatcherIO<T>::PLYElementF::getElementType(const std::string& elem_name)
{
	string lc = elem_name;
	boost::algorithm::to_lower(lc);
	if (lc == "vertex")
	{
		return VERTEX;
	}
	else
	{
		return UNSUPPORTED;
	}
}

template <typename T>
bool PointMatcherIO<T>::PLYElementF::elementSupported(const std::string& elem_name)
{
	return getElementType(elem_name) != UNSUPPORTED;
}

template<typename T>
typename PointMatcherIO<T>::PLYElement* PointMatcherIO<T>::PLYElementF::createElement(
		const std::string& elem_name, const int elem_num, const unsigned offset) {
	ElementTypes type = getElementType(elem_name);
	if (type == VERTEX)
		return new PLYVertex(elem_num, offset);
	else
		return NULL;
}

template<typename T>
bool PointMatcherIO<T>::plyPropTypeValid(const std::string& type) {
	return (type == "char" || type == "uchar" || type == "short"
			|| type == "ushort" || type == "int" || type == "uint"
			|| type == "float" || type == "double");
}

template <typename T>
bool PointMatcherIO<T>::PLYElement::operator==(const PLYElement& rhs) const
{
	return name == rhs.name;
}


template <typename T>
bool PointMatcherIO<T>::PLYProperty::operator==(const PLYProperty& rhs) const
{
	return name == rhs.name && type == rhs.type;
}


