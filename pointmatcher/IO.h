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
};

#endif // __POINTMATCHER_IO_H
