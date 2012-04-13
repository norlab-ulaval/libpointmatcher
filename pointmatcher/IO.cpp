// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
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
#include "InspectorsImpl.h"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <ctype.h>
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/lexical_cast.hpp"

using namespace std;

template<typename T>
PointMatcher<T>::FileInfo::FileInfo(const std::string& readingFileName, const std::string& referenceFileName, const std::string& configFileName, const TransformationParameters& initialTransformation, const TransformationParameters& groundTruthTransformation, const Vector& grativity):
	readingFileName(readingFileName),
	referenceFileName(referenceFileName),
	configFileName(configFileName),
	initialTransformation(initialTransformation),
	groundTruthTransformation(groundTruthTransformation),
	gravity(gravity)
{}

template<typename T>
std::string PointMatcher<T>::FileInfo::readingExtension() const
{
	#if BOOST_FILESYSTEM_VERSION >= 3
	return boost::filesystem::path(readingFileName).extension().string();
	#else
	return boost::filesystem::path(readingFileName).extension();
	#endif
}

template<typename T>
std::string PointMatcher<T>::FileInfo::referenceExtension() const
{
	#if BOOST_FILESYSTEM_VERSION >= 3
	return boost::filesystem::path(referenceFileName).extension().string();
	#else
	return boost::filesystem::path(referenceFileName).extension();
	#endif
}

template struct PointMatcher<float>::FileInfo;
template struct PointMatcher<double>::FileInfo;


//! Load a vector of FileInfo from a CSV file.
/**
	The supported tags are:
	- reading: file name of the reading point cloud
	- reference: file name of the reference point cloud
	- iTxy: initial transformation, coordinate x,y
	- gTxy: ground-truth transformation, coordinate x,y
	Note that the header must at least contain "Reading".
*/
template<typename T>
PointMatcher<T>::FileInfoVector::FileInfoVector(const std::string& fileName)
{
	#if BOOST_FILESYSTEM_VERSION >= 3
	const string parentPath = boost::filesystem::path(fileName).parent_path().string();
	#else
	const string parentPath = boost::filesystem::path(fileName).parent_path().file_string();
	#endif
	
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
	
	// Load reading
	const std::vector<string>& readingName = readingIt->second;
	const unsigned lineCount = readingName.size();

	// for every lines
	for(unsigned line=0; line<lineCount; line++)
	{
		FileInfo info;
		// Reading info
		info.readingFileName = parentPath+"/"+readingName[line];
		validateFile(info.readingFileName);
		
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
		push_back(info);
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

//! Return whether there is a valid transformation named prefix in data
template<typename T>
bool PointMatcher<T>::FileInfoVector::findTransform(const CsvElements& data, const std::string& prefix, unsigned dim)
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
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::FileInfoVector::getTransform(const CsvElements& data, const std::string& prefix, unsigned dim, unsigned line)
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

template struct PointMatcher<float>::FileInfoVector;
template struct PointMatcher<double>::FileInfoVector;


template<typename T>
void PointMatcher<T>::validateFile(const std::string& fileName)
{
	boost::filesystem::path fullPath(fileName);

	ifstream ifs(fileName.c_str());
	if (!ifs.good())
	#if BOOST_FILESYSTEM_VERSION >= 3
		throw runtime_error(string("Cannot open file ") + boost::filesystem3::complete(fullPath).native());
	#else
		throw runtime_error(string("Cannot open file ") + boost::filesystem::complete(fullPath).native_file_string());
	#endif

}

template
void PointMatcher<float>::validateFile(const std::string& fileName);
template
void PointMatcher<double>::validateFile(const std::string& fileName);

template<typename T>
std::vector<string> PointMatcher<T>::csvLineToVector(const char* line)
{
	std::vector<string> parsedLine;
	char delimiters[] = " \t,;";
	char *token;
	char tmpLine[1024];
	char *brkt;
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

template
std::vector<string> PointMatcher<float>::csvLineToVector(const char* line);
template
std::vector<string> PointMatcher<double>::csvLineToVector(const char* line);


template<typename T>
typename PointMatcher<T>::CsvElements PointMatcher<T>::parseCsvWithHeader(const std::string& fileName)
{
	ifstream is(fileName.c_str());
	
	validateFile(fileName);


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
				for(auto it=keywordCols.begin(); it!=keywordCols.end(); it++)
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
	/*
	for(auto it=data.begin(); it!=data.end(); it++)
	{
		cout << "--------------------------" << endl;
		cout << "Header: " << (*it).first << endl;
		for(unsigned i=0; i<(*it).second.size(); i++)
		{
			cout << (*it).second[i] << endl;
		}
	}
	*/

	return data;
}

template
PointMatcher<float>::CsvElements PointMatcher<float>::parseCsvWithHeader(const std::string& fileName);
template
PointMatcher<double>::CsvElements PointMatcher<double>::parseCsvWithHeader(const std::string& fileName);

//! @brief Load comma separated values (csv) file
//! @param fileName a string containing the path and the file name
//! 
//! This loader has 3 behaviors since there is not official standard for
//! csv files. A 2D or 3D point cloud will be created automatically if:
//!   - there is an header with columns named x, y and optionnaly z
//!   - there is only 2 or 3 columns in the file
//!
//! Otherwise, the user is asked to enter column id manually which might 
//! block automatic processing.
//!
//! @todo Add support to load descriptors (ex. color, ids, etc.)
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::loadCSV(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	
	validateFile(fileName);

	return loadCSV(ifs);
}

//! @brief Load comma separated values (csv) file
//! @see loadCSV()
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::loadCSV(std::istream& is)
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
			char *brkt;
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
		char *brkt;
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
		}
		
		labels.push_back(Label("pad", 1));

		firstLine = false;
	}

	assert(xData.size() == yData.size());
	int nbPoints = xData.size();

	// Transfer loaded points in specific structure (eigen matrix)
	typename DataPoints::Features features(dim+1, nbPoints);
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
PointMatcher<float>::DataPoints PointMatcher<float>::loadCSV(const std::string& fileName);
template
PointMatcher<double>::DataPoints PointMatcher<double>::loadCSV(const std::string& fileName);

//! Save point cloud to a file as CSV
template<typename T>
void PointMatcher<T>::saveCSV(const DataPoints& data, const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	saveCSV(data, ofs);
}

//! Save point cloud to a stream as CSV
template<typename T>
void PointMatcher<T>::saveCSV(const DataPoints& data, std::ostream& os)
{
	typedef typename DataPoints::Descriptors Descriptors;
	
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
void PointMatcher<float>::saveCSV(const DataPoints& data, const std::string& fileName);
template
void PointMatcher<double>::saveCSV(const DataPoints& data, const std::string& fileName);

//! Load point cloud from a file as VTK
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::loadVTK(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadVTK(ifs);
}

//! Load point cloud from a stream as VTK
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::loadVTK(std::istream& is)
{
	typedef typename DataPoints::Features Features;
	typedef typename DataPoints::Descriptors Descriptors;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	
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
	
	// parse points
	string type;
	int pointCount;
	string pad0;
	is >> type >> pointCount >> pad0;
	if (type != "POINTS" || pad0 != "float")
		throw runtime_error(string("Wrong header for points"));
	
	// read points (features)
	Features features(4, pointCount);
	for (int p = 0; p < pointCount; ++p)
	{
		is >> features(0, p);
		is >> features(1, p);
		is >> features(2, p);
		features(3, p) = 1.0;
	}
	Labels featureLabels;
	featureLabels.push_back(Label("x", 1));
	featureLabels.push_back(Label("y", 1));
	featureLabels.push_back(Label("z", 1));
	featureLabels.push_back(Label("w", 1));
	
	// read intermediate data (junk)
	string s;
	do {
		is >> s;
	} while (s != "POINT_DATA" && is.good());
	if (!is.good())
		return DataPoints(features, featureLabels);
	int pointDataCount;
	is >> pointDataCount;
	if (pointDataCount != pointCount)
		throw runtime_error("Different amount of points in the geometry and in attribute sections");
	getline(is, line);
	getline(is, line);
	//if (line != "COLOR_SCALARS lut 4")
		//throw runtime_error(string("Wrong point data, found ") + line);

	DataPoints loadedPoints;

	if (line == "COLOR_SCALARS lut 4")
	{
		// read color (descriptors)
		Descriptors descriptors(4, pointCount);
		for (int p = 0; p < pointCount; ++p)
		{
			is >> descriptors(0, p);
			is >> descriptors(1, p);
			is >> descriptors(2, p);
			is >> descriptors(3, p);
		}
		Labels descriptorLabels;
		descriptorLabels.push_back(Label("color", 4));
		loadedPoints = DataPoints(features, featureLabels, descriptors, descriptorLabels);
	}
	else
	{
		loadedPoints = DataPoints(features, featureLabels);
	}
	
	
	return loadedPoints;
}

template
PointMatcher<float>::DataPoints PointMatcher<float>::loadVTK(const std::string& fileName);
template
PointMatcher<double>::DataPoints PointMatcher<double>::loadVTK(const std::string& fileName);

//! Save point cloud to a file as VTK
template<typename T>
void PointMatcher<T>::saveVTK(const DataPoints& data, const std::string& fileName)
{
	Parameters param({{"baseFileName", ""}});
	typedef typename InspectorsImpl<T>::VTKFileInspector VTKInspector;
	VTKInspector vtkInspector(param);
	vtkInspector.dumpDataPoints(data, fileName);
	
}


template
void PointMatcher<float>::saveVTK(const PointMatcher<float>::DataPoints& data, const std::string& fileName);
template
void PointMatcher<double>::saveVTK(const PointMatcher<double>::DataPoints& data, const std::string& fileName);

