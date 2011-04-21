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
#include <stdexcept>
#include <limits>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdint.h>
#include <sys/time.h>

template<typename T>
T anyabs(const T& v)
{
	if (v < T(0))
		return -v;
	else
		return v;
}

/*
	High-precision timer class, using gettimeofday().
	The interface is a subset of the one boost::timer provides,
	but the implementation is much more precise
	on systems where clock() has low precision, such as glibc.
*/
struct timer
{
	typedef unsigned long long Time;
	
	timer():_start_time(curTime()){ } 
	void restart() { _start_time = curTime(); }
	double elapsed() const                  // return elapsed time in seconds
    { return  double(curTime() - _start_time) / double(1000000000); }

private:
	Time curTime() const {
		struct timespec ts;
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
		return Time(ts.tv_sec) * Time(1000000000) + Time(ts.tv_nsec);
	}
	Time _start_time;
};

template<typename T>
struct Histogram: public std::vector<T>
{
	typedef typename std::vector<T>::iterator Iterator;
	const size_t binCount;
	const std::string name;
	const std::string filePrefix;
	const bool dumpStdErrOnExit;
	
	Histogram(const size_t binCount, const std::string& name, const std::string& 
	filePrefix, const bool dumpStdErrOnExit):binCount(binCount), name(name), filePrefix(filePrefix), dumpStdErrOnExit(dumpStdErrOnExit) {}
	
	virtual ~Histogram()
	{
		T meanV, varV, medianV, lowQt, highQt, minV, maxV;
		uint64_t bins[binCount];
		uint64_t maxBinC;
		if (dumpStdErrOnExit || filePrefix.size() > 0)
			computeStats(meanV, varV, medianV, lowQt, highQt, minV, maxV, bins, maxBinC);
		
		if (filePrefix.size() > 0)
		{
			std::cerr << "writing to " << (filePrefix + name) << std::endl;
			std::ofstream ofs((filePrefix + name).c_str());
			for (size_t i = 0; i < this->size(); ++i)
				ofs << ((*this)[i]) << "\n";
		}
		
		if (dumpStdErrOnExit)
		{
			std::fill(bins, bins+binCount, uint64_t(0));
			std::cerr.precision(3);
			std::cerr.fill(' ');
			std::cerr.flags(std::ios::left);
			std::cerr << "Histogram " << name << ":\n";
			std::cerr << "  count: " << this->size() << ", mean: " << meanV << "\n";
			for (size_t i = 0; i < binCount; ++i)
			{
				const T v(minV + i * (maxV - minV) / T(binCount));
				std::cerr << "  " << std::setw(10) << v << " (" << std::setw(6) << bins[i] << ") : ";
				//std::cerr << (bins[i] * 60) / maxBinC << " " ;
				for (size_t j = 0; j < (bins[i] * 60) / maxBinC; ++j)
					std::cerr << "*";
				std::cerr << "\n";
			}
			std::cerr << std::endl;
		}
	}
	
	// this function compute statistics and writes them into the variables passed as reference
	void computeStats(T& meanV, T& varV, T& medianV, T& lowQt, T& highQt, T& minV, T& maxV, uint64_t* bins, uint64_t& maxBinC)
	{
		//assert(this->size() > 0);
		if(this->size() > 0)
		{
			// basic stats
			meanV = 0;
			minV = std::numeric_limits<T>::max();
			maxV = std::numeric_limits<T>::min();
			for (size_t i = 0; i < this->size(); ++i)
			{
				const T v((*this)[i]);
				meanV += v;
				minV = std::min<T>(minV, v);
				maxV = std::max<T>(maxV, v);
			}
			meanV /= T(this->size());
			// var and hist
			std::fill(bins, bins+binCount, uint64_t(0));
			maxBinC = 0;
			varV = 0;
			if (minV == maxV)
			{
				medianV = lowQt = highQt = minV;
				return;
			}
			for (size_t i = 0; i < this->size(); ++i)
			{
				const T v((*this)[i]);
				varV += (v - meanV)*(v - meanV);
				const size_t index((v - minV) * (binCount) / ((maxV - minV) * (1+std::numeric_limits<T>::epsilon()*10)));
				//std::cerr << "adding value " << v << " to index " << index << std::endl;
				++bins[index];
				maxBinC = std::max<uint64_t>(maxBinC, bins[index]);
			}
			varV /= T(this->size());
			// median
			const Iterator lowQtIt(this->begin() + (this->size() / 4));
			const Iterator medianIt(this->begin() + (this->size() / 2));
			const Iterator highQtIt(this->begin() + (3*this->size() / 4));
			std::nth_element(this->begin(), medianIt, this->end());
			medianV = *medianIt;
			std::nth_element(this->begin(), lowQtIt, this->end());
			lowQt = *lowQtIt;
			std::nth_element(this->begin(), highQtIt, this->end());
			highQt = *highQtIt;
		}
		else
		{
			meanV = std::numeric_limits<T>::quiet_NaN();
			varV = std::numeric_limits<T>::quiet_NaN();
			medianV = std::numeric_limits<T>::quiet_NaN();
			lowQt = std::numeric_limits<T>::quiet_NaN();
			highQt = std::numeric_limits<T>::quiet_NaN();
			minV = std::numeric_limits<T>::quiet_NaN();
			maxV = std::numeric_limits<T>::quiet_NaN();
			maxBinC = 0;
		}
	}
	
	void dumpStats(std::ostream& os)
	{
		T meanV, varV, medianV, lowQt, highQt, minV, maxV;
		uint64_t bins[binCount];
		uint64_t maxBinC;
		computeStats(meanV, varV, medianV, lowQt, highQt, minV, maxV, bins, maxBinC);
		os << meanV << " " << varV << " " << medianV << " " << lowQt << " " << highQt << " " << minV << " " << maxV << " " << binCount << " ";
		
		for (size_t i = 0; i < binCount; ++i)
			os << bins[i] << " ";
		os << maxBinC;
	}
};

template<typename T>
struct MetricSpaceAligner
{
	typedef T ScalarType;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	typedef typename Eigen::Matrix<T, 3, 1> Vector3;
	typedef std::vector<Vector, Eigen::aligned_allocator<Vector> > VectorVector;
	typedef typename Eigen::Quaternion<T> Quaternion;
	typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionVector;
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef typename Eigen::Matrix<T, 3, 3> Matrix3;
	typedef typename Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IntMatrix;
	typedef typename Nabo::NearestNeighbourSearch<T> NNS;
	typedef typename NNS::SearchType NNSearchType;
	
	typedef Matrix TransformationParameters;
	
	// input types
	struct DataPoints
	{
		typedef Matrix Features;
		typedef Matrix Descriptors;
		
		struct Label
		{
			std::string text;
			size_t span;
			Label(const std::string& text = "", const size_t span = 0):text(text), span(span) {}
		};
		typedef std::vector<Label> Labels;
		
		// Constructor
		DataPoints() {}
		// Constructor
		DataPoints(const Features& features, const Labels& featureLabels):
			features(features),
			featureLabels(featureLabels)
		{}
		// Constructor
		DataPoints(const Features& features, const Labels& featureLabels, const Descriptors& descriptors, const Labels& descriptorLabels):
			features(features),
			featureLabels(featureLabels),
			descriptors(descriptors),
			descriptorLabels(descriptorLabels)
		{}


		// Get descriptor by name
		// Return a matrix containing only the resquested descriptor
		Descriptors getDescriptorByName(const std::string& name) const
		{
			int row(0);
			
			for(unsigned int i = 0; i < descriptorLabels.size(); i++)
			{
				const int span(descriptorLabels[i].span);
				if(descriptorLabels[i].text.compare(name) == 0)
				{
					return descriptors.block(row, 0, 
							span, descriptors.cols());
				}

				row += span;
			}

			return Descriptors();
		}
		
		Features features;
		Labels featureLabels;
		Descriptors descriptors;
		Labels descriptorLabels;
	};
	
	struct ConvergenceError: std::runtime_error
	{
		ConvergenceError(const std::string& reason):runtime_error(reason) {}
	};
	
	// intermediate types
	struct Matches
	{
		typedef Matrix Dists;
		typedef IntMatrix Ids;
		// FIXME: shouldn't we have a matrix of pairs instead?
	
		Matches() {}
		Matches(const Dists& dists, const Ids ids):
			dists(dists),
			ids(ids)
		{}
		
		Dists dists;
		Ids ids;
	};

	typedef Matrix OutlierWeights;
	
	

	// type of processing bricks
	struct Transformation
	{
		virtual ~Transformation() {}
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const = 0;
	};
	struct Transformations: public std::vector<Transformation*>
	{
		void apply(DataPoints& cloud, const TransformationParameters& parameters) const;
	};
	typedef typename Transformations::iterator TransformationsIt;
	typedef typename Transformations::const_iterator TransformationsConstIt;
	
	struct TransformFeatures: public Transformation
	{
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
	};
	struct TransformDescriptors: Transformation
	{
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
	};
	
	
	// ---------------------------------
	struct DataPointsFilter
	{
		virtual ~DataPointsFilter() {}
		virtual void init() {}
		virtual DataPoints filter(const DataPoints& input, bool& iterate) = 0;
	};
	
	struct DataPointsFilters: public std::vector<DataPointsFilter*>
	{
		void init();
		void apply(DataPoints& cloud, bool iterate);
	};
	typedef typename DataPointsFilters::iterator DataPointsFiltersIt;
	typedef typename DataPointsFilters::const_iterator DataPointsFiltersConstIt;


	/* Data point operations */

	// Identidy
	struct IdentityDataPointsFilter: public DataPointsFilter
	{
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	};
	
	struct ClampOnAxisThresholdDataPointsFilter: public DataPointsFilter
	{
		const unsigned dim;
		const T threshold;
		
		ClampOnAxisThresholdDataPointsFilter(const unsigned dim, const T threshold);
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	};
	
	struct ClampOnAxisRatioDataPointsFilter: public DataPointsFilter
	{
		const unsigned dim;
		const T ratio;
		
		ClampOnAxisRatioDataPointsFilter(const unsigned dim, const T ratio);
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	};
	
	// Surface normals
	class SurfaceNormalDataPointsFilter: public DataPointsFilter
	{
		const int knn;
		const double epsilon;
		const bool keepNormals;
		const bool keepDensities;
		const bool keepEigenValues;
		const bool keepEigenVectors;
		const bool keepMatchedIds;
		
	public:
		SurfaceNormalDataPointsFilter(const int knn = 5, 
			const double epsilon = 0,
			const bool keepNormals = true,
			const bool keepDensities = false,
			const bool keepEigenValues = false, 
			const bool keepEigenVectors = false,
			const bool keepMatchedIds = false);
		virtual ~SurfaceNormalDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	};
	
	// Sampling surface normals
	class SamplingSurfaceNormalDataPointsFilter: public DataPointsFilter
	{
		const int k;
		const bool averageExistingDescriptors;
		const bool keepNormals;
		const bool keepDensities;
		const bool keepEigenValues;
		const bool keepEigenVectors;
		
	public:
		SamplingSurfaceNormalDataPointsFilter(const int k = 10,
			const bool averageExistingDescriptors = true,
			const bool keepNormals = true,
			const bool keepDensities = false,
			const bool keepEigenValues = false, 
			const bool keepEigenVectors = false);
		virtual ~SamplingSurfaceNormalDataPointsFilter() {}
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
		
	protected:
		struct BuildData
		{
			typedef std::vector<int> Indices;
			
			Indices indices;
			const Matrix& inputFeatures;
			const Matrix& inputDescriptors;
			Matrix outputFeatures;
			Matrix outputDescriptors;
			int outputInsertionPoint;
			
			BuildData(const Matrix& inputFeatures, const Matrix& inputDescriptors, const int finalDescDim):
				inputFeatures(inputFeatures),
				inputDescriptors(inputDescriptors),
				outputFeatures(inputFeatures.rows(), inputFeatures.cols()),
				outputDescriptors(finalDescDim, inputFeatures.cols()),
				outputInsertionPoint(0)
			{
				const int pointsCount(inputFeatures.cols());
				indices.reserve(pointsCount);
				for (int i = 0; i < pointsCount; ++i)
					indices[i] = i;
			}
		};
		
		struct CompareDim
		{
			const int dim;
			const BuildData& buildData;
			CompareDim(const int dim, const BuildData& buildData):dim(dim),buildData(buildData){}
			bool operator() (const int& p0, const int& p1)
			{
				return  buildData.inputFeatures(dim, p0) < 
						buildData.inputFeatures(dim, p1);
			}
		};
		
	protected:
		void buildNew(BuildData& data, const int first, const int last, const Vector minValues, const Vector maxValues) const;
		void fuseRange(BuildData& data, const int first, const int last) const;
	};

	// Reorientation of normals
	class OrientNormalsDataPointsFilter: public DataPointsFilter
	{
	public:
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	};

	// Random sampling
	class RandomSamplingDataPointsFilter: public DataPointsFilter
	{
		// Probability to keep points, between 0 and 1
		const double prob;
		
	public:
		RandomSamplingDataPointsFilter(const double ratio = 0.5);
		virtual ~RandomSamplingDataPointsFilter() {};
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	private:
		DataPoints randomSample(const DataPoints& input) const;
	};
	
	// Systematic sampling
	class FixstepSamplingDataPointsFilter: public DataPointsFilter
	{
		// number of steps to skip
		const double startStep;
		const double endStep;
		const double stepMult;
		double step;
		
		
	public:
		FixstepSamplingDataPointsFilter(const double startStep = 10, const double endStep = 10, const double stepMult = 1);
		virtual ~FixstepSamplingDataPointsFilter() {};
		virtual void init();
		virtual DataPoints filter(const DataPoints& input, bool& iterate);
	private:
		DataPoints fixstepSample(const DataPoints& input);
	};

	// ---------------------------------
	struct Matcher
	{
		// FIXME: this is a rather ugly way to do stats
		unsigned long visitCounter;
		
		Matcher():visitCounter(0) {}
		
		void resetVisitCount() { visitCounter = 0; }
		unsigned long getVisitCount() { return visitCounter; }
		virtual ~Matcher() {}
		virtual void init(const DataPoints& filteredReference, bool& iterate) = 0;
		virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference, bool& iterate) = 0;
	};
	
	struct NullMatcher: public Matcher
	{
		virtual void init(const DataPoints& filteredReference, bool& iterate);
		virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference, bool& iterate);
	};
	
	class KDTreeMatcher: public Matcher
	{
		const int knn;
		const T epsilon;
		const NNSearchType searchType;
		const T maxDist;
		NNS* featureNNS;
	
	public:
		KDTreeMatcher(const int knn = 1, const T epsilon = 0, const NNSearchType searchType = NNS::KDTREE_LINEAR_HEAP, const T maxDist = std::numeric_limits<T>::infinity());
		virtual ~KDTreeMatcher();
		virtual void init(const DataPoints& filteredReference, bool& iterate);
		virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference, bool& iterate);
	};
	
	// ---------------------------------
	struct FeatureDistanceExtractor
	{
		virtual ~FeatureDistanceExtractor() {}
		virtual typename Matches::Dists compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const typename Matches::Ids& associations) = 0;
	};
	
	// ---------------------------------
	struct DescriptorDistanceExtractor
	{
		virtual ~DescriptorDistanceExtractor() {}
		virtual typename Matches::Dists compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const typename Matches::Ids& associations) = 0;
	};
	
	// ---------------------------------
	struct FeatureOutlierFilter
	{
		virtual ~FeatureOutlierFilter() {}
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate) = 0;
	};
	
	struct NullFeatureOutlierFilter: public FeatureOutlierFilter
	{
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};
	
	struct MaxDistOutlierFilter: public FeatureOutlierFilter
	{
		const T maxDist;
		
		MaxDistOutlierFilter(const T maxDist);

		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};
	
	// Generic function to get quartile
	// TODO: move that to Utils.h
	static T getQuantile(const Matches& input, const T quantile);


	struct MedianDistOutlierFilter: public FeatureOutlierFilter 
	{
		const T factor;
		
		MedianDistOutlierFilter(const T factor);
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};

	
	
	/* Hard rejection threshold using quantile.
	   Based on:
	     D Chetverikov, "The Trimmed Iterative Closest Point Algorithm" (2002)
	 */
	struct TrimmedDistOutlierFilter: public FeatureOutlierFilter
	{
		const T ratio;
		
		TrimmedDistOutlierFilter(const T ratio);
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};
	
	/* Hard rejection threshold using quantile and variable ratio.
	   Based on:
	     J. M. Phillips and al., "Outlier Robust ICP for Minimizing Fractional RMSD" (2007)
	 */
	struct VarTrimmedDistOutlierFilter: public FeatureOutlierFilter
	{
	    // default ratio
	    T ratio_;
		// min ratio
		T min_;
		// max ratio
		T max_;
		// lambda (part of the term that balance the rmsd: 1/ratio^lambda)
		T lambda_;

		VarTrimmedDistOutlierFilter(T r);
		VarTrimmedDistOutlierFilter(T r, T min, T max, T lambda);
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);

		private:
		// return the optimized ratio
		T optimizeInlierRatio(const Matches& matches, T min = 0.05, T max = 0.99, T lambda = 0.95);
};


	struct MinDistOutlierFilter: public FeatureOutlierFilter
	{
		const T minDist;
		
		MinDistOutlierFilter(const T minDist); 
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};
	
	
	// Vector of transformation checker
	struct FeatureOutlierFilters: public std::vector<FeatureOutlierFilter*>
	{
		OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};
	typedef typename FeatureOutlierFilters::iterator FeatureOutlierFiltersIt;

	
	// ---------------------------------
	struct DescriptorOutlierFilter
	{
		virtual ~DescriptorOutlierFilter() {}
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate) = 0;
	};
	
	struct NullDescriptorOutlierFilter: public DescriptorOutlierFilter
	{
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	};


	// ---------------------------------
	struct ErrorMinimizer
	{
		struct ErrorElements
		{
			DataPoints reading;
			DataPoints reference;
			OutlierWeights weights;
			Matches matches;

			// TODO: put that in ErrorMinimizer.cpp. Tried but didn't succeed
			ErrorElements(const DataPoints& reading, const DataPoints reference, const OutlierWeights weights, const Matches matches):
				reading(reading),
				reference(reference),
				weights(weights),
				matches(matches)
			{
				assert(reading.features.cols() == reference.features.cols());
				assert(reading.features.cols() == weights.cols());
				assert(reading.features.cols() == matches.dists.cols());
				// May have no descriptors... size 0
			}
		};
		
		ErrorMinimizer():pointUsedRatio(-1.),weightedPointUsedRatio(-1.) {}
		virtual ~ErrorMinimizer() {}
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches, bool& iterate) = 0;
		T getPointUsedRatio() const { return pointUsedRatio; }
		T getWeightedPointUsedRatio() const { return weightedPointUsedRatio; }
		
	protected:
		// helper functions
		Matrix crossProduct(const Matrix& A, const Matrix& B);
		ErrorElements getMatchedPoints(const DataPoints& reading, const DataPoints& reference, const Matches& matches, const OutlierWeights& outlierWeights);
		
	protected:
		T pointUsedRatio;
		T weightedPointUsedRatio;
	};
	
	struct IdentityErrorMinimizer: ErrorMinimizer
	{
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches, bool& iterate);
	};
	
	// Point-to-point error
	// Based on SVD decomposition
	struct PointToPointErrorMinimizer: ErrorMinimizer
	{
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches, bool& iterate);
	};
	
	// Point-to-plane error (or point-to-line in 2D)
	struct PointToPlaneErrorMinimizer: public ErrorMinimizer
	{
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches, bool& iterate);
	};
	
	
	// ---------------------------------
	struct TransformationChecker
	{
		Vector limits;
		Vector values;
		std::vector<std::string> limitNames;
		std::vector<std::string> valueNames;

		virtual ~TransformationChecker() {}
		virtual void init(const TransformationParameters& parameters, bool& iterate) = 0;
		virtual void check(const TransformationParameters& parameters, bool& iterate) = 0;
		
		static Vector matrixToAngles(const TransformationParameters& parameters);
	};

	struct CounterTransformationChecker: public TransformationChecker
	{
		CounterTransformationChecker(const int maxIterationCount = 20);
		
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};
	
	class ErrorTransformationChecker: public TransformationChecker
	{
	protected:
		QuaternionVector rotations;
		VectorVector translations;
		const unsigned int tail;

	public:
		ErrorTransformationChecker(const T minDeltaRotErr, const T minDeltaTransErr, const unsigned int tail = 3);
		
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};
	
	class BoundTransformationChecker: public TransformationChecker
	{
	protected:
		Quaternion initialRotation;
		Vector initialTranslation;
		
	public:
		BoundTransformationChecker(const T maxRotationNorm, const T maxTranslationNorm);
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};
	
	
	// Vector of transformation checker
	struct TransformationCheckers: public std::vector<TransformationChecker*>
	{
		void init(const TransformationParameters& parameters, bool& iterate);
		void check(const TransformationParameters& parameters, bool& iterate);
	};
	typedef typename TransformationCheckers::iterator TransformationCheckersIt;
	typedef typename TransformationCheckers::const_iterator TransformationCheckersConstIt;


	// ---------------------------------
	struct Inspector
	{
		virtual void init() {};
		virtual void dumpFilteredReference(const DataPoints& filteredReference) {}
		virtual void dumpIteration(const size_t iterationCount, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& featureOutlierWeights, const OutlierWeights& descriptorOutlierWeights, const TransformationCheckers& transformationCheckers) {}
		virtual void finish(const size_t iterationCount) {}
		virtual ~Inspector() {}
	};

	// Clearer name when no inspector is required
	typedef Inspector NullInspector;
	
	struct AbstractVTKInspector: public Inspector
	{
	protected:
		virtual std::ostream* openStream(const std::string& role) = 0;
		virtual std::ostream* openStream(const std::string& role, const size_t iterationCount) = 0;
		virtual void closeStream(std::ostream* stream) = 0;
		void dumpDataPoints(const DataPoints& data, std::ostream& stream);
		void dumpMeshNodes(const DataPoints& data, std::ostream& stream);
		void dumpDataLinks(const DataPoints& ref, const DataPoints& reading, 	const Matches& matches, const OutlierWeights& featureOutlierWeights, std::ostream& stream);
		
		std::ostream* streamIter;

	public:
		AbstractVTKInspector();
		virtual void init() {};
		virtual void dumpDataPoints(const DataPoints& cloud, const std::string& name);
		virtual void dumpMeshNodes(const DataPoints& cloud, const std::string& name);
		virtual void dumpIteration(const size_t iterationCount, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& featureOutlierWeights, const OutlierWeights& descriptorOutlierWeights, const TransformationCheckers& transformationCheckers);
		virtual void finish(const size_t iterationCount);
	
	private:
    	void buildGenericAttributeStream(std::ostream& stream, const std::string& attribute, const std::string& nameTag, const DataPoints& cloud, const int forcedDim);

		void buildScalarStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
    	void buildScalarStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
		
		void buildNormalStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
		void buildNormalStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
		
		void buildVectorStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
		void buildVectorStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
		
		void buildTensorStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
		void buildTensorStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);

		Matrix padWithZeros(const Matrix m, const int expectedRow, const int expectedCols); 
	};

	struct VTKFileInspector: public AbstractVTKInspector
	{

	protected:
		const std::string baseFileName;

		virtual std::ostream* openStream(const std::string& role);
		virtual std::ostream* openStream(const std::string& role, const size_t iterationCount);
		virtual void closeStream(std::ostream* stream);
		
	public:
		VTKFileInspector(const std::string& baseFileName);
		virtual void init();
		virtual void finish(const size_t iterationCount);
	};
	
	
	// ICP algorithm
	struct ICP
	{
		ICP();
		~ICP();
		
		TransformationParameters operator()(
			const TransformationParameters& initialTransformationParameters, 
			DataPoints reading,
			DataPoints reference);
		
		DataPointsFilters readingDataPointsFilters;
		DataPointsFilters referenceDataPointsFilters;
		Transformations transformations;
		Matcher* matcher;
		FeatureOutlierFilters featureOutlierFilters;
		DescriptorOutlierFilter* descriptorOutlierFilter;
		ErrorMinimizer* errorMinimizer;
		TransformationCheckers transformationCheckers;
		Inspector* inspector;
		T outlierMixingWeight;
	};
	
	// ICP sequence, with keyframing
	struct ICPSequence
	{
		ICPSequence(const int dim, const std::string& filePrefix = "", const bool dumpStdErrOnExit = true);
		~ICPSequence();
		
		TransformationParameters operator()(DataPoints& inputCloud);
		
		DataPointsFilters readingDataPointsFilters;
		DataPointsFilters readingStepDataPointsFilters;
		DataPointsFilters keyframeDataPointsFilters;
		Transformations transformations;
		Matcher* matcher;
		FeatureOutlierFilters featureOutlierFilters;
		DescriptorOutlierFilter* descriptorOutlierFilter;
		ErrorMinimizer* errorMinimizer;
		TransformationCheckers transformationCheckers;
		Inspector* inspector;
		T outlierMixingWeight;
		T ratioToSwitchKeyframe;
		
		Histogram<double> keyFrameDuration;
		Histogram<double> convergenceDuration;
		Histogram<unsigned> iterationsCount;
		Histogram<unsigned> pointCountIn;
		Histogram<unsigned> pointCountReading;
		Histogram<unsigned> pointCountKeyFrame;
		Histogram<unsigned> pointCountTouched;
		Histogram<double> overlapRatio;
		
		TransformationParameters getTransform() const { return keyFrameTransform * curTransform; }
		TransformationParameters getDeltaTransform() const { return lastTransformInv * getTransform(); }
		bool keyFrameCreatedAtLastCall() const { return keyFrameCreated; }
		//! Drop current key frame, create a new one with inputCloud, reset transformations
		void resetTracking(DataPoints& inputCloud);
		
	private:
		void createKeyFrame(DataPoints& inputCloud);
		
		bool keyFrameCreated;
		DataPoints keyFrameCloud; //!< point cloud of the keyframe
		TransformationParameters keyFrameTransform; //!< pose of keyframe
		TransformationParameters keyFrameTransformOffset; //!< offset for centered keyframe
		TransformationParameters curTransform; //!< transform of last frame wrt keyframe (last call to operator())
		TransformationParameters lastTransformInv; //!< inv of previous computed transform (using getTransform())
	};
}; // MetricSpaceAligner

template<typename T>
void swapDataPoints(typename MetricSpaceAligner<T>::DataPoints& a, typename MetricSpaceAligner<T>::DataPoints& b)
{
	a.features.swap(b.features);
	swap(a.featureLabels, b.featureLabels);
	a.descriptors.swap(b.descriptors);
	swap(a.descriptorLabels, b.descriptorLabels);
}

#endif // __POINTMATCHER_CORE_H

