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

If you have compiled libpointmatcher with \ref yaml-cpp enabled, you can configure the ICP chain without any recompilation by passing a configuration file to the \c pmicp command using the \c --config switch. An example file is available in \c data/examples/default.yaml.

\section DevelopingUsing Developing using libpointmatcher

If you wish to develop using libpointmatcher, you can start by looking at the sources of icp_simple and icp (in \c example/icp_simple.cpp and \c example/icp.cpp). You can see how loading/saving of data files work by looking at convertCSVtoVTK (\c example/convertCSVtoVTK.cpp). If you want to see how libpointmatcher can align a sequence of clouds, you can have a look at align_sequence (\c example/align_sequence.cpp).

\section DevelopingSelf Extending libpointmatcher

You can also extend libpointmatcher relatively easily, by adding new modules. The file PointMatcher.h is the most important file, it defines the interfaces for all module types, as well as the ICP algorithms. Each interface is an inner class of the PointMatcher class, which is templatized on the scalar type. Instanciation is forced in \c Core.cpp for float and double scalar types. There are different types of modules corresponding to the different bricks of the ICP algorithm. The modules themselves are defined in their own files, for instance data-point filters live in the \c DataPointFilters.h/\c .cpp files. You can read a description of the ICP chain architecture in our \ref icppaper "IROS paper". Then, start from the documentation of PointMatcher to see the different interfaces, and then read the source code of the existing modules for the interface you are interested in, to get an idea of what you have to implement.

All modules have a common way to get parameters at initialization, and feature a self-documentation mechanism. This allows to configure the ICP chain from external descriptions such as \ref yaml-cpp "yaml files".

\subsection CreatingNewDataPointFilter Example: creating a new module of type DataPointsFilter

You have to modify 3 files to add a new \ref PointMatcher::DataPointsFilter "DataPointsFilter": \c DataPointsFiltersImpl.h, \c DataPointsFiltersImpl.cpp and \c Core.cpp. The easiest way is to start by copying and renaming \c IdentityDataPointsFilter, and then to modify it.

- In \c DataPointsFiltersImpl.h, copy the declaration of the struct \c IdentityDataPointsFilter at the end of the file,
- Rename the pasted structure with the new filter name,
- Fill the \c description() function with a short explanation of the filter's action,
- If you need parameters for the filter:
	- Uncomment the \c availableParameters() function and filled the description, default, min, max values as string,
	- The types of the parameters are used to properly cast the string value and can be:  \c &P::Comp<T>, \c &P::Comp<int>, \c &P::Comp<unsigned>, etc. See \c DataPointsFiltersImpl.h for examples,
	- Uncomment and rename the constructor.

- In \c DataPointsFiltersImpl.cpp, copy the implementation of \c IdentityDataPointsFilter at the end of the file including the predeclaration (i.e. \c template \c struct \c DataPointsFiltersImpl<float>::YourFilter; and \c template \c struct 
\c DataPointsFiltersImpl<double>::YourFilter;),
- Add the constructor if needed,
- At this stage, you should let the implementation of the filter function to hold only the statement that returns the input.

- In \c Core.cpp, search for \c "ADD_TO_REGISTRAR(DataPointsFilter,",
- You should see all available \c dataPointfilter there. At the end of that block, add your filter.

- Compile.

- Test if your new module is available with the \c pmicp \c -l command, which lists the documentation of all modules. You should be able to find yours in the output.

- Go back to \c DataPointFilter.cpp and code the \c filter() function.

\subsection CodingStyle Coding Style

One shall:
- indent with tabs,
- put {} on single lines (\ref Allman_style "Allman coding style"),
- use \ref CamelCase "camel case" with classes beginning with Capitals and members with a small letter.

For documentation, one shall document classes and data members inside the header file and methods at the implementation location.

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

#ifdef HAVE_YAML_CPP
namespace YAML
{
	class Node;
}
#endif // HAVE_YAML_CPP

//! version of the Pointmatcher library as string
#define POINTMATCHER_VERSION "0.9.0"
//! version of the Pointmatcher library as an int
#define POINTMATCHER_VERSION_INT "900"

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
		Logger() {}
		Logger(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):Parametrizable(className,paramsDoc,params) {}
		
		virtual ~Logger() {}
		virtual bool hasInfoChannel() const { return false; };
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func) {}
		virtual std::ostream* infoStream() { return 0; }
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func) {}
		virtual bool hasWarningChannel() const { return false; }
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func) {}
		virtual std::ostream* warningStream() { return 0; }
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func) {}
	};
	
	void setLogger(Logger* newLogger);
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
		//! Feature points of size (dims+1) x ptcount
		typedef Matrix Features;
		//! Descriptor points of size descriptor_dims x ptcount
		typedef Matrix Descriptors;
		
		//! The name for a certain number of dim
		struct Label
		{
			std::string text; //!< name of the label
			size_t span; //!< number of data dimensions the label spans
			Label(const std::string& text = "", const size_t span = 0);
		};
		//! A vector of Label
		struct Labels: std::vector<Label>
		{
			typedef typename std::vector<Label>::const_iterator const_iterator; //!< alias
			bool contains(const std::string& text) const;
		};
		
		DataPoints();
		DataPoints(const Features& features, const Labels& featureLabels);
		DataPoints(const Features& features, const Labels& featureLabels, const Descriptors& descriptors, const Labels& descriptorLabels);
		
		void concatenate(const DataPoints dp);
		void addDescriptor(const std::string& name, Descriptors newDescriptor);
		Descriptors getDescriptorByName(const std::string& name) const;
		bool isDescriptorExist(const std::string& name) const;
		bool isDescriptorExist(const std::string& name, const unsigned dim) const;
		int getDescriptorDimension(const std::string& name) const;
		int getDescriptorStartingRow(const std::string& name) const;
		
		Features features; //!< features of points in the cloud
		Labels featureLabels; //!< labels of features
		Descriptors descriptors; //!< descriptors of points in the cloud, might be empty
		Labels descriptorLabels; //!< labels of descriptors
	};
	
	// ---------------------------------
	// IO functions
	// ---------------------------------
	typedef std::map<std::string, std::vector<std::string>> CsvElements;

	static void validateFile(const std::string& fileName);
	static std::vector<std::string> csvLineToVector(const char* line);
	static CsvElements parseCsvWithHeader(const std::string& fileName);

	//Should it be here?
	static DataPoints concatenateDataPoints(const DataPoints dp1, const DataPoints dp2);

	// CSV
	
	static DataPoints loadCSV(const std::string& fileName);
	static DataPoints loadCSV(std::istream& is);

	static void saveCSV(const DataPoints& data, const std::string& fileName);
	static void saveCSV(const DataPoints& data, std::ostream& os);

	// VTK
	
	static DataPoints loadVTK(const std::string& fileName);
	static DataPoints loadVTK(std::istream& is);

	static void saveVTK(const DataPoints& data, const std::string& fileName);
	
	//! Information required to exploit a reading from a file using this library. Fields might be left blank if unused.
	struct FileInfo 
	{
		std::string readingFileName;
		std::string referenceFileName;
		std::string configFileName;
		TransformationParameters initialTransformation;
		TransformationParameters groundTruthTransformation;
		Eigen::Matrix<T, 3, 1> gravity;

		FileInfo(const std::string& readingPath="", const std::string& referencePath="", const std::string& configFileName="", const TransformationParameters& initialTransformation=TransformationParameters(), const TransformationParameters& groundTruthTransformation=TransformationParameters(),  const Vector& grativity=Eigen::Matrix<T,3,1>::Zero());
		
		std::string readingExtension() const;
		std::string referenceExtension() const;
	};

	//! A vector of file info, to be used in batch
	struct FileInfoVector: public std::vector<FileInfo>
	{
		FileInfoVector(const std::string& fileName, std::string parentPath = "");
	
	protected:
		std::string localToGlobalFileName(const std::string& path, const std::string& fileName);
		bool findTransform(const CsvElements& data, const std::string& prefix, unsigned dim);
		TransformationParameters getTransform(const CsvElements& data, const std::string& prefix, unsigned dim, unsigned line);
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
		
		Dists dists; //!< squared distances to closest points
		Ids ids; //!< identifiers of closest points
		
		T getDistsQuantile(const T quantile) const;
	};

	//! Weights resulting of the application of FeatureOutlierFilter or DescriptorOutlierFilter; a dense matrix over ScalarType
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
		
		virtual void init(const DataPoints& filteredReference) = 0;
		virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference) = 0;
	};
	
	DEF_REGISTRAR(Matcher)
	
	// ---------------------------------
	
	//! A feature outlier filter removes links between points in reading and their matched points in reference, depending on some criteria on the features. Points with no link will be ignored in the subsequent minimization step.
	struct FeatureOutlierFilter: public Parametrizable
	{
		FeatureOutlierFilter();
		FeatureOutlierFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		virtual ~FeatureOutlierFilter();
		
		//! Detect outliers using features
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input) = 0;
	};
	
	//! A descriptor outlier filter removes links between points in reading and their matched points in reference, depending on some criteria on the descriptors. Points with no link will be ignored in the subsequent minimization step.
	struct DescriptorOutlierFilter
	{
		virtual ~DescriptorOutlierFilter();
		
		//! Detect outliers using descriptors
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input) = 0;
	};
	
	//! A chain of outlier filters of type F
	template<typename F>
	struct OutlierFilters: public PointMatcherSupport::SharedPtrVector<F>
	{
		typedef PointMatcherSupport::SharedPtrVector<F> Vector; //!< alias
		
		OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input);
	};
	
	//! A chain of FeatureOutlierFilter
	typedef OutlierFilters<FeatureOutlierFilter> FeatureOutlierFilters;
	//! A chain of DescriptorOutlierFilter
	typedef OutlierFilters<DescriptorOutlierFilter> DescriptorOutlierFilters;
	typedef typename FeatureOutlierFilters::const_iterator FeatureOutlierFiltersConstIt; //!< alias
	typedef typename FeatureOutlierFilters::iterator FeatureOutlierFiltersIt; //!< alias
	typedef typename DescriptorOutlierFilters::const_iterator DescriptorOutlierFiltersConstIt; //!< alias
	typedef typename DescriptorOutlierFilters::iterator DescriptorOutlierFiltersIt; //!< alias
	
	DEF_REGISTRAR(FeatureOutlierFilter)
	DEF_REGISTRAR(DescriptorOutlierFilter)

	// ---------------------------------
	
	//! An error minimizer will compute a transformation matrix such as to minimize the error between the reading and the reference. 
	struct ErrorMinimizer: public Parametrizable
	{
		//! A structure holding data ready for minimization. The data are "normalized", for instance there are no point with 0 weight, etc.
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
		ErrorElements getMatchedPoints(const DataPoints& reading, const DataPoints& reference, const Matches& matches, const OutlierWeights& outlierWeights);
		
	protected:
		T pointUsedRatio; //!< the ratio of how many points where used for error minimization
		T weightedPointUsedRatio; //!< the ratio of how many points where used (with weight) for error minimization
		ErrorElements lastErrorElements; //!< Memory of the last error computed
	};
	
	DEF_REGISTRAR(ErrorMinimizer)
	
	// ---------------------------------
	
	//! A transformation checker can stop the iteration depending on some conditions.
	struct TransformationChecker: public Parametrizable
	{
	protected:
		typedef std::vector<std::string> StringVector;
		Vector limits;
		Vector values;
		StringVector limitNames;
		StringVector valueNames;

	public:
		TransformationChecker();
		TransformationChecker(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):Parametrizable(className,paramsDoc,params) {}
		virtual ~TransformationChecker() {}
		virtual void init(const TransformationParameters& parameters, bool& iterate) = 0;
		virtual void check(const TransformationParameters& parameters, bool& iterate) = 0;
		
		const Vector& getLimits() const { return limits; }
		const Vector& getValues() const { return values; }
		const StringVector& getLimitNames() const { return limitNames; }
		const StringVector& getValueNames() const { return valueNames; }
		
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
		virtual void dumpFilteredReference(const DataPoints& filteredReference);
		virtual void dumpIteration(const size_t iterationCount, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& featureOutlierWeights, const OutlierWeights& descriptorOutlierWeights, const TransformationCheckers& transformationCheckers);
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
		DataPointsFilters keyframeDataPointsFilters; //!< filters for keyframe
		Transformations transformations; //!< transformations
		std::shared_ptr<Matcher> matcher; //!< matcher
		FeatureOutlierFilters featureOutlierFilters; //!< outlier filters on features
		DescriptorOutlierFilters descriptorOutlierFilters; //!< outlier filters on descriptors
		std::shared_ptr<ErrorMinimizer> errorMinimizer; //!< error minimizer
		TransformationCheckers transformationCheckers; //!< transformation checkers
		std::shared_ptr<Inspector> inspector; //!< inspector
		T outlierMixingWeight; //!< weighting ratio of feature vs outlier filters
		
		virtual ~ICPChainBase();

		virtual void setDefault();
		
		void loadFromYaml(std::istream& in);
		unsigned getPrefilteredReadingPtsCount() const;
		unsigned getPrefilteredKeyframePtsCount() const;
		
	protected:
		unsigned prefilteredReadingPtsCount; //!< remaining number of points after prefiltering but before the iterative process
		unsigned prefilteredKeyframePtsCount; //!< remaining number of points after prefiltering but before the iterative process

		ICPChainBase();
		
		void cleanup();
		
		#ifdef HAVE_YAML_CPP
		virtual void loadAdditionalYAMLContent(YAML::Node& doc) {}
		
		template<typename R>
		void createModulesFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, PointMatcherSupport::SharedPtrVector<typename R::TargetType>& modules);
		
		template<typename R>
		void createModuleFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, std::shared_ptr<typename R::TargetType>& module);
		
		template<typename R>
		typename R::TargetType* createModuleFromRegistrar(const YAML::Node& module, const R& registrar);
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
	};
	
	//! ICP alogrithm, taking a sequence of clouds and using keyframing
	struct ICPSequence: ICPChainBase
	{
		T ratioToSwitchKeyframe; //!< when the ratio of matching points is below this, create a new keyframe
		
		ICPSequence();
		~ICPSequence();
		
		TransformationParameters operator()(const DataPoints& inputCloudIn);
	
		TransformationParameters getTransform() const;
		TransformationParameters getDeltaTransform() const;
		bool keyFrameCreatedAtLastCall() const;
		bool hasKeyFrame() const;
		
		void resetTracking(DataPoints& inputCloud);
		
		virtual void setDefault();
		
	protected:
		#ifdef HAVE_YAML_CPP
		virtual void loadAdditionalYAMLContent(YAML::Node& doc);
		#endif // HAVE_YAML_CPP
		
	private:
		bool keyFrameCreated; //!< true if the key frame was created at least once
		DataPoints keyFrameCloud; //!< point cloud of the keyframe
		TransformationParameters keyFrameTransform; //!< pose of keyframe
		
		TransformationParameters T_refIn_refMean; //!< offset for centered keyframe
		//TransformationParameters keyFrameTransformOffset; //old T_refIn_refMean 
		
		TransformationParameters T_refIn_dataIn; //!< transform of last frame wrt keyframe (last call to operator())
		//TransformationParameters curTransform; //old T_refMean_dataIn

		TransformationParameters lastTransformInv; //!< inv of previous computed transform (using getTransform())
		
		void createKeyFrame(DataPoints& inputCloud);
	};
	
	//! Constructor, fill registrars
	PointMatcher();
}; // PointMatcher<T>

#endif // __POINTMATCHER_CORE_H

