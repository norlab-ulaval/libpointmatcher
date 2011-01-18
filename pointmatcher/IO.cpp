#include "IO.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

using namespace std;

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadCSV(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadCSV<T>(ifs);
}

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadCSV(std::istream& is)
{
	typedef typename MetricSpaceAligner<T>::DataPoints DataPoints;
	typedef typename DataPoints::Labels Labels;
	typedef typename DataPoints::Label Label;
	
	vector<T> data;
	int dim(0);
	bool firstLine(true);
	Labels labels;
	
	while (!is.eof())
	{
		char line[1024];
		is.getline(line, sizeof(line));
		line[sizeof(line)-1] = 0;
		
		char *token = strtok(line, " \t,;");
		while (token)
		{
			if (firstLine)
				++dim;
			data.push_back(atof(token));
			//cout << atof(token) << " ";
			token = strtok(NULL, " \t,;"); // FIXME: non reentrant, use strtok_r
		}
		if (firstLine)
			++dim;
		data.push_back(1);
		//cout << "\n";
		if (firstLine)
		{
			// create labels
			size_t i=0;
			while (i < 3 && i <data.size())
			{
				string text;
				text += char('x' + i);
				labels.push_back(Label(text, 1));
				++i;
			}
			if (i + 1 < data.size())
				labels.push_back(Label("pad", data.size() - i));
		}
		firstLine = false;
	}
	
	return DataPoints(DataPoints::Features::Map(&data[0], dim, data.size() / dim), labels);
}

template
MetricSpaceAligner<float>::DataPoints loadCSV<float>(const std::string& fileName);
template
MetricSpaceAligner<double>::DataPoints loadCSV<double>(const std::string& fileName);


template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadVTK(const std::string& fileName)
{
	ifstream ifs(fileName.c_str());
	if (!ifs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	return loadVTK<T>(ifs);
}

template<typename T>
typename MetricSpaceAligner<T>::DataPoints loadVTK(std::istream& is)
{
	typedef typename MetricSpaceAligner<T>::DataPoints DataPoints;
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
	if (line != "COLOR_SCALARS lut 4")
		throw runtime_error(string("Wrong point data, found ") + line);
	
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
	
	return DataPoints(
		features,
		featureLabels,
		descriptors,
		descriptorLabels
	);
}

template
MetricSpaceAligner<float>::DataPoints loadVTK<float>(const std::string& fileName);
template
MetricSpaceAligner<double>::DataPoints loadVTK<double>(const std::string& fileName);


template<typename T>
void saveVTK(const typename MetricSpaceAligner<T>::DataPoints& data, const std::string& fileName)
{
	ofstream ofs(fileName.c_str());
	if (!ofs.good())
		throw runtime_error(string("Cannot open file ") + fileName);
	saveVTK<T>(data, ofs);
}

template<typename T>
void saveVTK(const typename MetricSpaceAligner<T>::DataPoints& data, std::ostream& os)
{
	typedef typename MetricSpaceAligner<T>::DataPoints::Descriptors Descriptors;
	
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
		}
		os << "\n";
	}
	os << "VERTICES 1 " << pointCount * 2 << "\n";
	for (int i = 0; i < pointCount; ++i)
	{
		os << "1 " << i << "\n";
	}
	/*os << "VERTICES " << pointCount << " " << pointCount + 1 << "\n";
	os << pointCount;
	for (size_t i = 0; i < pointCount; ++i)
		os << " " << i;
	os << "\n";*/
	
	Descriptors colors = data.getDescriptorByName("color");
	if (colors.cols() == 0)
	{
		cerr << "Warning: cannot find color in descriptors" << endl;
		return;
	}
	
	assert(colors.cols() == pointCount);
	assert(colors.rows() == 4);
	
	// write colors
	os << "POINT_DATA " << pointCount << "\n";
	os << "COLOR_SCALARS lut 4\n";
	for (int p = 0; p < pointCount; ++p)
	{
		os << colors(0, p) << " " << colors(1, p) << " " << colors(2, p) << " " << colors(3, p) << "\n";
	}
}

template
void saveVTK<float>(const MetricSpaceAligner<float>::DataPoints& data, const std::string& fileName);
template
void saveVTK<double>(const MetricSpaceAligner<double>::DataPoints& data, const std::string& fileName);