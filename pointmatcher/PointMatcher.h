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

#ifndef __POINTMATCHER_CORE_H
#define __POINTMATCHER_CORE_H

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR
#define EIGEN2_SUPPORT
#include "Eigen/StdVector"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "nabo/nabo.h"

#include <boost/thread/mutex.hpp>

#include <stdexcept>
#include <limits>
#include <iostream>
#include <ostream>
#include <memory>

#include "Parametrizable.h"
#include "Registrar.h"

#if NABO_VERSION_INT < 10001
	#error "You need libnabo version 1.0.1 or greater"
#endif

/*! 
	\file PointMatcher.h
	\brief public interface
*/

/*!
\mainpage libpointmatcher

from http://github.com/ethz-asl/libpointmatcher by François Pomerleau and Stéphane Magnenat (http://stephane.magnenat.net), ASL-ETHZ, Switzerland (http://www.asl.ethz.ch)

libpointmatcher is a modular ICP library, useful for robotics and computer vision. This help assumes that libpointmatcher is already installed, if not, please read the \c README.md file at the top-level of the source tree.

\section Test

To test, you can use the \c pmicp command provided in the \c example directory, or installed system-wide with the \c -bin deb package.

In 2D:
\code
pmicp ${SRC_DIR}/examples/data/2D_oneBox.csv ${SRC_DIR}/examples/data/data/2D_twoBoxes.csv
\endcode
In 3D:
\code
pmicp ${SRC_DIR}/examples/data/car_cloud401.csv ${SRC_DIR}/examples/data/car_cloud400.csv
\endcode
Use \ref Paraview to view the results. On \ref Ubuntu, you can install Paraview with:
\code
sudo apt-get install paraview
\endcode

You can list the available modules with:
\code
pmicp -l
\endcode

If you have compiled libpointmatcher with \ref yaml-cpp enabled, you can configure the ICP chain without any recompilation by passing a configuration file to the \c pmicp command using the \c --config switch. An example file is available in \c examples/data/default.yaml.

\section DevelopingUsing Developing using libpointmatcher

If you wish to develop using libpointmatcher, you can start by looking at the sources of icp_simple and pmicp (in \c example/icp_simple.cpp and \c example/icp.cpp). You can see how loading/saving of data files work by looking at convert (\c example/convert.cpp). If you want to see how libpointmatcher can align a sequence of clouds, you can have a look at align_sequence (\c example/align_sequence.cpp).

\section DevelopingSelf Extending libpointmatcher

You can also extend libpointmatcher relatively easily, by adding new modules. The file PointMatcher.h is the most important file, it defines the interfaces for all module types, as well as the ICP algorithms. Each interface is an inner class of the PointMatcher class, which is templatized on the scalar type. Instanciation is forced in \c Core.cpp for float and double scalar types. There are different types of modules corresponding to the different bricks of the ICP algorithm. The modules themselves are defined in their own files, for instance data-point filters live in the \c DataPointFiltersImpl.h/\c .cpp files. You can read a description of the ICP chain architecture in our \ref icppaper "IROS paper". Then, start from the documentation of PointMatcher to see the different interfaces, and then read the source code of the existing modules for the interface you are interested in, to get an idea of what you have to implement.

All modules have a common way to get parameters at initialization, and feature a self-documentation mechanism. This allows to configure the ICP chain from external descriptions such as \ref yaml-cpp "yaml files".

\subsection CreatingNewDataPointFilter Example: creating a new module of type DataPointsFilter

You have to modify 3 files to add a new \ref PointMatcher::DataPointsFilter "DataPointsFilter": \c DataPointsFiltersImpl.h, \c DataPointsFiltersImpl.cpp and \c Core.cpp. The easiest way is to start by copying and renaming \c IdentityDataPointsFilter, and then to modify it.

- In \c DataPointsFiltersImpl.h, copy the declaration of the struct \c IdentityDataPointsFilter at the end of the file.
- Rename the pasted structure with the new filter name.
- Fill the \c description() function with a short explanation of the filter's action.
- If you need parameters for the filter:
	- Uncomment the \c availableParameters() function and fill the description, default, min, max values as string.
	- The types of the parameters are used to properly cast the string value and can be:  \c &P::Comp<T>, \c &P::Comp<int>, \c &P::Comp<unsigned>, etc. See \c DataPointsFiltersImpl.h for examples.
	- Uncomment and rename the constructor.

- In \c DataPointsFiltersImpl.cpp, copy the implementation of \c IdentityDataPointsFilter at the end of the file including the predeclaration (i.e. \c template \c struct \c DataPointsFiltersImpl<float>::YourFilter; and \c template \c struct 
\c DataPointsFiltersImpl<double>::YourFilter;).
- Add the constructor if needed.
- At this stage, you should let the implementation of the filter function to hold only the statement that returns the input.

- In \c Core.cpp, search for \c "ADD_TO_REGISTRAR(DataPointsFilter,".
- You should see all available \c dataPointfilter there. At the end of that block, add your filter.

- Compile.

- Test if your new module is available with the \c pmicp \c -l command, which lists the documentation of all modules. You should be able to find yours in the output.

- Go back to \c DataPointsFiltersImpl.cpp and code the \c filter() function.

\subsection CodingStyle Coding Style

One shall:
- indent with tabs,
- put {} on single lines (\ref Allman_style "Allman coding style"),
- use \ref CamelCase "camel case" with classes beginning with Capitals and members with a small letter.

For documentation, one shall document classes and data members inside the header file and methods at the implementation location.
One exception is purely-virtual methods, which must be documented in the header file.

\section BugReporting Bug reporting

Please use <a href="http://github.com/ethz-asl/libpointmatcher/issues">github's issue tracker</a> to report bugs.

\section Citing

If you use libpointmatcher in an academic context, please cite the following publication:
\code
@INPROCEEDINGS{pomerleau11tracking,
	author = {François Pomerleau and Stéphane Magnenat and Francis Colas and Ming Liu and Roland Siegwart},
	title = {Tracking a Depth Camera: Parameter Exploration for Fast ICP},
	booktitle = {Proc. of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	publisher = {IEEE Press},
	pages = {3824--3829},
	year = {2011}
}
\endcode

\section License

libpointmatcher is released under a permissive BSD license.

\section References

- \anchor icppaper IROS paper: http://publications.asl.ethz.ch/files/pomerleau11tracking.pdf
- \anchor Eigen Eigen: http://eigen.tuxfamily.org
- \anchor libnabo libnabo: http://github.com/ethz-asl/libnabo
- \anchor yaml-cpp yaml-cpp: http://code.google.com/p/yaml-cpp/
- \anchor CMake CMake: http://www.cmake.org
- \anchor Boost Boost: http://www.boost.org
- \anchor Ubuntu Ubuntu: http://www.ubuntu.com
- \anchor CMake CMake: http://www.cmake.org
- \anchor CMakeDoc CMake documentation: http://www.cmake.org/cmake/help/cmake2.6docs.html
- \anchor git git: http://git-scm.com
- \anchor ROS ROS: http://www.ros.org/
- \anchor Paraview Paraview: http://www.paraview.org/
- \anchor Allman_style Allman coding style: http://en.wikipedia.org/wiki/Indent_style#Allman_style
- \anchor CamelCase Camel case style: http://en.wikipedia.org/wiki/CamelCase

*/

//! version of the Pointmatcher library as string
#define POINTMATCHER_VERSION "0.9.0"
//! version of the Pointmatcher library as an int
#define POINTMATCHER_VERSION_INT 900

//! Functions and classes that are not dependant on scalar type are defined in this namespace
namespace PointMatcherSupport
{
	//! A vector of std::shared_ptr<S> that behaves like a std::vector<S>
	template<typename S>
	struct SharedPtrVector: public std::vector<std::shared_ptr<S>>
	{
		//! Add an instance of S to the vector, take ownership
		void push_back(S* v) { std::vector<std::shared_ptr<S>>::push_back(std::shared_ptr<S>(v)); }
	};
	
	//! The logger interface, used to output warnings and informations
	struct Logger: public Parametrizable
	{
		Logger();
		Logger(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		virtual ~Logger();
		virtual bool hasInfoChannel() const;
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* infoStream();
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func);
		virtual bool hasWarningChannel() const;
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* warningStream();
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func);
	};
	
	void setLogger(Logger* newLogger);
	
	void validateFile(const std::string& fileName);
	
	//! Data from a CSV file
	typedef std::map<std::string, std::vector<std::string>> CsvElements;
}

//! Functions and classes that are dependant on scalar type are defined in this templatized class
template<typename T>
struct PointMatcher
{
	// ---------------------------------
	// macros for constants
	// ---------------------------------
	
	//! The smallest value larger than 0
	#define ZERO_PLUS_EPS (0. + std::numeric_limits<double>::epsilon())
	//! The largest value smaller than 1
	#define ONE_MINUS_EPS (1. - std::numeric_limits<double>::epsilon())
	
	// ---------------------------------
	// exceptions
	// ---------------------------------
	
	//! Point matcher did not converge
	struct ConvergenceError: std::runtime_error
	{
		//! Construct the exception with an error message
		ConvergenceError(const std::string& reason):runtime_error(reason) {}
	};
	
	// ---------------------------------
	// eigen and nabo-based types
	// ---------------------------------
	
	//! The scalar type
	typedef T ScalarType;
	//! A vector over ScalarType
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	//! A vector of vector over ScalarType, not a matrix
	typedef std::vector<Vector, Eigen::aligned_allocator<Vector> > VectorVector;
	//! A quaternion over ScalarType
	typedef typename Eigen::Quaternion<T> Quaternion;
	//! A vector of quaternions over ScalarType
	typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionVector;
	//! A dense matrix over ScalarType
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	//! A dense integer matrix
	typedef typename Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IntMatrix;
	
	//! The parameters of a transformation; a dense matrix over ScalarType
	typedef Matrix TransformationParameters;
	
	// alias for scope reasons
	typedef PointMatcherSupport::Parametrizable Parametrizable; //!< alias
	typedef Parametrizable::Parameters Parameters; //!< alias
	typedef Parametrizable::ParameterDoc ParameterDoc; //!< alias
	typedef Parametrizable::ParametersDoc ParametersDoc; //!< alias
	typedef Parametrizable::InvalidParameter InvalidParameter; //!< alias
	
	// ---------------------------------
	// input types
	// ---------------------------------
	
	//! A point cloud
	struct DataPoints
	{
		//! A view on a feature or descriptor
		typedef Eigen::Block<Matrix> View;
		//! A view on a const feature or const descriptor
		typedef const Eigen::Block<const Matrix> ConstView;
		
		//! The name for a certain number of dim
		struct Label
		{
			std::string text; //!< name of the label
			size_t span; //!< number of data dimensions the label spans
			Label(const std::string& text = "", const size_t span = 0);
			bool operator ==(const Label& that) const;
		};
		//! A vector of Label
		struct Labels: std::vector<Label>
		{
			typedef typename std::vector<Label>::const_iterator const_iterator; //!< alias
			Labels();
			Labels(const Label& label);
			bool contains(const std::string& text) const;
			size_t totalDim() const;
		};
		
		//! An exception thrown when one tries to access features or descriptors unexisting or of wrong dimensions
		struct InvalidField: std::runtime_error
		{
			//! Construct the exception with an error message
			InvalidField(const std::string& reason):runtime_error(reason) {}
		};
		
		DataPoints();
		DataPoints(const Labels& featureLabels, const Labels& descriptorLabels, const size_t pointCount);
		DataPoints(const Matrix& features, const Labels& featureLabels);
		DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels);
		bool operator ==(const DataPoints& that) const;
		
		void concatenate(const DataPoints& dp);
		
		void allocateFeature(const std::string& name, const unsigned dim);
		void allocateFeatures(const Labels& newLabels);
		void addFeature(const std::string& name, const Matrix& newFeature);
		Matrix getFeatureCopyByName(const std::string& name) const;
		ConstView getFeatureViewByName(const std::string& name) const;
		View getFeatureViewByName(const std::string& name);
		bool featureExists(const std::string& name) const;
		bool featureExists(const std::string& name, const unsigned dim) const;
		unsigned getFeatureDimension(const std::string& name) const;
		unsigned getFeatureStartingRow(const std::string& name) const;
		
		void allocateDescriptor(const std::string& name, const unsigned dim);
		void allocateDescriptors(const Labels& newLabels);
		void addDescriptor(const std::string& name, const Matrix& newDescriptor);
		Matrix getDescriptorCopyByName(const std::string& name) const;
		ConstView getDescriptorViewByName(const std::string& name) const;
		View getDescriptorViewByName(const std::string& name);
		bool descriptorExists(const std::string& name) const;
		bool descriptorExists(const std::string& name, const unsigned dim) const;
		unsigned getDescriptorDimension(const std::string& name) const;
		unsigned getDescriptorStartingRow(const std::string& name) const;
		void assertDescriptorConsistency() const;
		
		Matrix features; //!< features of points in the cloud
		Labels featureLabels; //!< labels of features
		Matrix descriptors; //!< descriptors of points in the cloud, might be empty
		Labels descriptorLabels; //!< labels of descriptors
	
	private:
		void allocateFields(const Labels& newLabels, Labels& labels, Matrix& data) const;
		void allocateField(const std::string& name, const unsigned dim, Labels& labels, Matrix& data) const;
		void addField(const std::string& name, const Matrix& newField, Labels& labels, Matrix& data) const;
		ConstView getConstViewByName(const std::string& name, const Labels& labels, const Matrix& data) const;
		View getViewByName(const std::string& name, const Labels& labels, Matrix& data) const;
		bool fieldExists(const std::string& name, const unsigned dim, const Labels& labels) const;
		unsigned getFieldDimension(const std::string& name, const Labels& labels) const;
		unsigned getFieldStartingRow(const std::string& name, const Labels& labels) const;
	};
	
	static void swapDataPoints(DataPoints& a, DataPoints& b);
	
	// ---------------------------------
	// IO functions
	// ---------------------------------

	// Generic load and save
	
	static DataPoints loadAnyFormat(const std::string& fileName);
	static void saveAnyFormat(const DataPoints& data, const std::string& fileName);
	
	// CSV
	
	static DataPoints loadCSV(const std::string& fileName);
	static DataPoints loadCSV(std::istream& is);

	static void saveCSV(const DataPoints& data, const std::string& fileName);
	static void saveCSV(const DataPoints& data, std::ostream& os);

	// VTK
	
	static DataPoints loadVTK(const std::string& fileName);
	static DataPoints loadVTK(std::istream& is);

	static void saveVTK(const DataPoints& data, const std::string& fileName);
	
	// TODO: The FileInfo and FileInfoVector structs should go somewhere else than this main file
	//! Information required to exploit a reading from a file using this library. Fields might be left blank if unused.
	struct FileInfo 
	{
		std::string readingFileName; //!< file name of the reading point cloud
		std::string referenceFileName; //!< file name of the reference point cloud
		std::string configFileName; //!< file name of the yaml configuration
		TransformationParameters initialTransformation; //!< matrix of initial estimate transform
		TransformationParameters groundTruthTransformation; //!< matrix of the ground-truth transform
		Eigen::Matrix<T, 3, 1> gravity; //!< gravity vector

		FileInfo(const std::string& readingPath="", const std::string& referencePath="", const std::string& configFileName="", const TransformationParameters& initialTransformation=TransformationParameters(), const TransformationParameters& groundTruthTransformation=TransformationParameters(),  const Vector& grativity=Eigen::Matrix<T,3,1>::Zero());
	};

	//! A vector of file info, to be used in batch
	struct FileInfoVector: public std::vector<FileInfo>
	{
		FileInfoVector(const std::string& fileName, std::string dataPath = "", std::string configPath = "");
	
	protected:
		std::string localToGlobalFileName(const std::string& path, const std::string& fileName);
		bool findTransform(const PointMatcherSupport::CsvElements& data, const std::string& prefix, unsigned dim);
		TransformationParameters getTransform(const PointMatcherSupport::CsvElements& data, const std::string& prefix, unsigned dim, unsigned line);
	};

	// ---------------------------------
	// intermediate types
	// ---------------------------------
	
	//! Result of Matcher::findClosests
	struct Matches
	{
		typedef Matrix Dists; //!< Squared distances to closest points, dense matrix of ScalarType
		typedef IntMatrix Ids; //!< Identifiers of closest points, dense matrix of integers
	
		Matches();
		Matches(const Dists& dists, const Ids ids);
		Matches(const int knn, const int pointsCount);
		
		Dists dists; //!< squared distances to closest points
		Ids ids; //!< identifiers of closest points
		
		T getDistsQuantile(const T quantile) const;
	};

	//! Weights resulting from the application of OutlierFilter or DescriptorOutlierFilter; a dense matrix over ScalarType
	typedef Matrix OutlierWeights;
	
	// ---------------------------------
	// types of processing bricks
	// ---------------------------------
	
	//! A function that transforms points and their descriptor given parameters
	struct Transformation: public Parametrizable
	{
		Transformation();
		Transformation(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Transformation();
		
		//! Transform input using parameters
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const = 0; 
	};
	
	//! A chain of Transformation
	struct Transformations: public PointMatcherSupport::SharedPtrVector<Transformation>
	{
		void apply(DataPoints& cloud, const TransformationParameters& parameters) const;
	};
	typedef typename Transformations::iterator TransformationsIt; //!< alias
	typedef typename Transformations::const_iterator TransformationsConstIt; //!< alias
	
	DEF_REGISTRAR(Transformation)
	
	// ---------------------------------
	
	//! A data filter takes a point cloud as input, transforms it, and produces another point cloud as output.
	struct DataPointsFilter: public Parametrizable
	{
		DataPointsFilter();
		DataPointsFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~DataPointsFilter();
		virtual void init();
		
		//! Apply this filter to input
		virtual DataPoints filter(const DataPoints& input) = 0;
	};
	
	//! A chain of DataPointsFilter
	struct DataPointsFilters: public PointMatcherSupport::SharedPtrVector<DataPointsFilter>
	{
		DataPointsFilters();
		DataPointsFilters(std::istream& in);
		void init();
		void apply(DataPoints& cloud);
	};
	typedef typename DataPointsFilters::iterator DataPointsFiltersIt; //!< alias
	typedef typename DataPointsFilters::const_iterator DataPointsFiltersConstIt; //!< alias
	
	DEF_REGISTRAR(DataPointsFilter)
	
	// ---------------------------------
	
	//! A matcher links points in the reading to points in the reference
	struct Matcher: public Parametrizable
	{
		unsigned long visitCounter; //!< number of points visited
		
		Matcher();
		Matcher(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Matcher();
		
		void resetVisitCount();
		unsigned long getVisitCount() const;
		
		//! Init this matcher to find nearest neighbor in filteredReference
		virtual void init(const DataPoints& filteredReference) = 0;
		//! Find the closest neighbors of filteredReading in filteredReference passed to init()
		virtual Matches findClosests(const DataPoints& filteredReading) = 0;
	};
	
	DEF_REGISTRAR(Matcher)
	
	// ---------------------------------
	
	//! An outlier filter removes links between points in reading and their matched points in reference, depending on some criteria. Points with no link will be ignored in the subsequent minimization step.
	struct OutlierFilter: public Parametrizable
	{
		OutlierFilter();
		OutlierFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		virtual ~OutlierFilter();
		
		//! Detect outliers using features
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input) = 0;
	};
	
	
	//! A chain of outlier filters
	struct OutlierFilters: public PointMatcherSupport::SharedPtrVector<OutlierFilter>
	{
		OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input);
	};
	
	typedef typename OutlierFilters::const_iterator OutlierFiltersConstIt; //!< alias
	typedef typename OutlierFilters::iterator OutlierFiltersIt; //!< alias
	
	DEF_REGISTRAR(OutlierFilter)

	// ---------------------------------
	
	//! An error minimizer will compute a transformation matrix such as to minimize the error between the reading and the reference. 
	struct ErrorMinimizer: public Parametrizable
	{
		//! A structure holding data ready for minimization. The data are "normalized", for instance there are no points with 0 weight, etc.
		struct ErrorElements
		{
			DataPoints reading; //!< reading point cloud
			DataPoints reference; //!< reference point cloud
			OutlierWeights weights; //!< weights for every association
			Matches matches; //!< associations

			ErrorElements(const DataPoints& reading=DataPoints(), const DataPoints reference = DataPoints(), const OutlierWeights weights = OutlierWeights(), const Matches matches = Matches());
		};
		
		ErrorMinimizer();
		virtual ~ErrorMinimizer();
		
		T getPointUsedRatio() const;
		T getWeightedPointUsedRatio() const;
		virtual T getOverlap() const;
		
		//! Find the parameters that minimize the error
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) = 0;
		
		
	protected:
		// helper functions
		static Matrix crossProduct(const Matrix& A, const Matrix& B);
		ErrorElements& getMatchedPoints(const DataPoints& reading, const DataPoints& reference, const Matches& matches, const OutlierWeights& outlierWeights);
		
	protected:
		T pointUsedRatio; //!< the ratio of how many points were used for error minimization
		T weightedPointUsedRatio; //!< the ratio of how many points were used (with weight) for error minimization
		ErrorElements lastErrorElements; //!< memory of the last computed error
	};
	
	DEF_REGISTRAR(ErrorMinimizer)
	
	// ---------------------------------
	
	//! A transformation checker can stop the iteration depending on some conditions.
	struct TransformationChecker: public Parametrizable
	{
	protected:
		typedef std::vector<std::string> StringVector; //!< a vector of strings
		Vector limits; //!< values of limits
		// FIXME: values is a very bad name
		Vector values; //!< collected values
		StringVector limitNames; //!< names of limits
		StringVector valueNames; //!< names of values

	public:
		TransformationChecker();
		TransformationChecker(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~TransformationChecker() {}
		//! Init, set iterate to false if iteration should stop
		virtual void init(const TransformationParameters& parameters, bool& iterate) = 0;
		//! Set iterate to false if iteration should stop
		virtual void check(const TransformationParameters& parameters, bool& iterate) = 0;
		
		const Vector& getLimits() const;
		const Vector& getValues() const;
		const StringVector& getLimitNames() const;
		const StringVector& getValueNames() const;
		
	protected:
		static Vector matrixToAngles(const TransformationParameters& parameters);
	};
	
	//! A chain of TransformationChecker
	struct TransformationCheckers: public PointMatcherSupport::SharedPtrVector<TransformationChecker>
	{
		void init(const TransformationParameters& parameters, bool& iterate);
		void check(const TransformationParameters& parameters, bool& iterate);
	};
	typedef typename TransformationCheckers::iterator TransformationCheckersIt; //!< alias
	typedef typename TransformationCheckers::const_iterator TransformationCheckersConstIt; //!< alias
	
	DEF_REGISTRAR(TransformationChecker)

	// ---------------------------------
	
	//! An inspector allows to log data at the different steps, for analysis.
	struct Inspector: public Parametrizable
	{
		
		Inspector();
		Inspector(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		// 
		virtual ~Inspector();
		virtual void init();
		
		// performance statistics
		virtual void addStat(const std::string& name, double data);
		virtual void dumpStats(std::ostream& stream);
		virtual void dumpStatsHeader(std::ostream& stream);
		
		// data statistics 
		virtual void dumpIteration(const size_t iterationNumber, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& outlierWeights, const TransformationCheckers& transformationCheckers);
		virtual void finish(const size_t iterationCount);
	};
	
	DEF_REGISTRAR(Inspector) 
	
	// ---------------------------------
	
	DEF_REGISTRAR_IFACE(Logger, PointMatcherSupport::Logger)

	// ---------------------------------
	
	// algorithms
	
	//! Stuff common to all ICP algorithms
	struct ICPChainBase
	{
	public:
		DataPointsFilters readingDataPointsFilters; //!< filters for reading, applied once
		DataPointsFilters readingStepDataPointsFilters; //!< filters for reading, applied at each step
		DataPointsFilters referenceDataPointsFilters; //!< filters for reference
		Transformations transformations; //!< transformations
		std::shared_ptr<Matcher> matcher; //!< matcher
		OutlierFilters outlierFilters; //!< outlier filters
		std::shared_ptr<ErrorMinimizer> errorMinimizer; //!< error minimizer
		TransformationCheckers transformationCheckers; //!< transformation checkers
		std::shared_ptr<Inspector> inspector; //!< inspector
		
		virtual ~ICPChainBase();

		virtual void setDefault();
		
		void loadFromYaml(std::istream& in);
		unsigned getPrefilteredReadingPtsCount() const;
		unsigned getPrefilteredReferencePtsCount() const;
		
	protected:
		unsigned prefilteredReadingPtsCount; //!< remaining number of points after prefiltering but before the iterative process
		unsigned prefilteredReferencePtsCount; //!< remaining number of points after prefiltering but before the iterative process

		ICPChainBase();
		
		void cleanup();
		
		#ifdef HAVE_YAML_CPP
		virtual void loadAdditionalYAMLContent(YAML::Node& doc) {}
		
		template<typename R>
		void createModulesFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, PointMatcherSupport::SharedPtrVector<typename R::TargetType>& modules);
		
		template<typename R>
		void createModuleFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, std::shared_ptr<typename R::TargetType>& module);
		
		/*template<typename R>
		typename R::TargetType* createModuleFromRegistrar(const YAML::Node& module, const R& registrar);*/
		#endif // HAVE_YAML_CPP
	};
	
	//! ICP algorithm
	struct ICP: ICPChainBase
	{
		TransformationParameters operator()(
			const DataPoints& readingIn,
			const DataPoints& referenceIn);

		TransformationParameters operator()(
			const DataPoints& readingIn,
			const DataPoints& referenceIn,
			const TransformationParameters& initialTransformationParameters);
		
		TransformationParameters compute(
			const DataPoints& readingIn,
			const DataPoints& referenceIn,
			const TransformationParameters& initialTransformationParameters);
	
	protected:
		TransformationParameters computeWithTransformedReference(
			const DataPoints& readingIn, 
			const DataPoints& reference, 
			const TransformationParameters& T_refIn_refMean,
			const TransformationParameters& initialTransformationParameters);
	};
	
	//! ICP alogrithm, taking a sequence of clouds and using a map
	struct ICPSequence: public ICP
	{
		TransformationParameters operator()(
			const DataPoints& cloudIn);
		TransformationParameters operator()(
			const DataPoints& cloudIn,
			const TransformationParameters& initialTransformationParameters);
		TransformationParameters compute(
			const DataPoints& cloudIn,
			const TransformationParameters& initialTransformationParameters);
		
		bool hasMap() const;
		bool setMap(const DataPoints& map);
		void clearMap();
		const DataPoints& getInternalMap() const;
		const DataPoints getMap() const;
		
	protected:
		DataPoints mapPointCloud; //!< point cloud of the map, always in global frame (frame of first point cloud)
		TransformationParameters T_refIn_refMean; //!< offset for centered map
	};
	
	// ---------------------------------
	// Instance-related functions
	// ---------------------------------
	
	PointMatcher();
	
	static const PointMatcher& get();
}; // PointMatcher<T>

#endif // __POINTMATCHER_CORE_H

