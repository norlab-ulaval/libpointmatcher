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

#include "IO.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <ctype.h>


using namespace std;

template<typename T>
typename PointMatcher<T>::DataPoints loadCSV(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadCSV<T>(ifs);
}

template<typename T>
typename PointMatcher<T>::DataPoints loadCSV(std::istream& is)
{
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename DataPoints::Labels Labels;
	typedef typename DataPoints::Label Label;
	
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
			strcpy(tmpLine, line);
			token = strtok(tmpLine, delimiters);
			while (token)
			{
				dim++;
				
				// Load text header
				if(hasHeader)
				{
					header.push_back(string(token));
				}

				token = strtok(NULL, delimiters); // FIXME: non reentrant, use strtok_r
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
		token = strtok(line, delimiters);
		int currentCol = 0;
		while (token)
		{
			// Load data only if no text is on the line
			if(!hasHeader)
			{
				//TODO: look for specific columns
				if(currentCol == xCol)
					xData.push_back(atof(token));
				if(currentCol == yCol)
					yData.push_back(atof(token));
				if(currentCol == zCol)
					zData.push_back(atof(token));
			}

			token = strtok(NULL, delimiters); // FIXME: non reentrant, use strtok_r
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
PointMatcher<float>::DataPoints loadCSV<float>(const std::string& fileName);
template
PointMatcher<double>::DataPoints loadCSV<double>(const std::string& fileName);

template<typename T>
void saveCSV(const typename PointMatcher<T>::DataPoints& data, const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	saveCSV<T>(data, ofs);
}

template<typename T>
void saveCSV(const typename PointMatcher<T>::DataPoints& data, std::ostream& os)
{
	typedef typename PointMatcher<T>::DataPoints::Descriptors Descriptors;
	
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
void saveCSV<float>(const PointMatcher<float>::DataPoints& data, const std::string& fileName);
template
void saveCSV<double>(const PointMatcher<double>::DataPoints& data, const std::string& fileName);

template<typename T>
typename PointMatcher<T>::DataPoints loadVTK(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadVTK<T>(ifs);
}

template<typename T>
typename PointMatcher<T>::DataPoints loadVTK(std::istream& is)
{
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename DataPoints::Descriptors Descriptors;
	typedef typename DataPoints::Features Features;
	typedef typename DataPoints::Labels Labels;
	typedef typename DataPoints::Label Label;
	
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
PointMatcher<float>::DataPoints loadVTK<float>(const std::string& fileName);
template
PointMatcher<double>::DataPoints loadVTK<double>(const std::string& fileName);


template<typename T>
void saveVTK(const typename PointMatcher<T>::DataPoints& data, const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	saveVTK<T>(data, ofs);
}

template<typename T>
void saveVTK(const typename PointMatcher<T>::DataPoints& data, std::ostream& os)
{
	typedef typename PointMatcher<T>::DataPoints::Descriptors Descriptors;
	
	const int pointCount(data.features.cols());
	const int dimCount(data.features.rows());
	
	if (pointCount == 0)
	{
		cerr << "Warning, no points, doing nothing" << endl;
		return;
	}
	
	// write header
	os << "# vtk DataFile Version 3.0\n";
	os << "data\n";
	os << "ASCII\n";
	os << "DATASET POLYDATA\n";
	
	// write points
	os << "POINTS " << pointCount << " float\n";
	for (int p = 0; p < pointCount; ++p)
	{
		for (int i = 0; i < dimCount-1; ++i)
		{
			os << data.features(i, p) << " ";
			if(i == 1 && dimCount-1 == 2)
				os << " 0";
		}
		os << "\n";
	}
	os << "VERTICES " << pointCount << " " << pointCount * 2 << "\n";
	for (int i = 0; i < pointCount; ++i)
	{
		os << "1 " << i << "\n";
	}
	/*os << "VERTICES " << pointCount << " " << pointCount + 1 << "\n";
	os << pointCount;
	for (size_t i = 0; i < pointCount; ++i)
		os << " " << i;
	os << "\n";*/

	os << "POINT_DATA " << pointCount << "\n";
	
	Descriptors colors = data.getDescriptorByName("color");
	if (colors.cols() != 0)
	{
		//cerr << "Warning: cannot find color in descriptors" << endl;
		assert(colors.cols() == pointCount);
		assert(colors.rows() == 4);
	
		// write colors
		os << "COLOR_SCALARS lut 4\n";
		for (int p = 0; p < pointCount; ++p)
		{
			os << colors(0, p) << " " << colors(1, p) << " " << colors(2, p) << " " << colors(3, p) << "\n";
		}
	}

	Descriptors normals = data.getDescriptorByName("normals");
	if (normals.cols() != 0)
	{
		assert(normals.cols() == pointCount);
		assert(normals.rows() == 3);

		os << "NORMALS triangle_normals float\n";
		for (int p = 0; p < pointCount; ++p)
		{
			os << normals(0, p) << " " << normals(1, p) << " " << normals(2, p) << "\n";
		}
	}
}

template
void saveVTK<float>(const PointMatcher<float>::DataPoints& data, const std::string& fileName);
template
void saveVTK<double>(const PointMatcher<double>::DataPoints& data, const std::string& fileName);
