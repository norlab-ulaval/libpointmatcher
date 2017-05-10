
#include "PointMatcher.h"


template<typename T>
struct PointToPlaneWithCovErrorMinimizer: public PointMatcher<T>::ErrorMinimizer
{
		typedef PointMatcherSupport::Parametrizable Parametrizable;
		typedef PointMatcherSupport::Parametrizable P;
		typedef Parametrizable::Parameters Parameters;
		typedef Parametrizable::ParameterDoc ParameterDoc;
		typedef Parametrizable::ParametersDoc ParametersDoc;

		typedef typename PointMatcher<T>::DataPoints DataPoints;
		typedef typename PointMatcher<T>::Matches Matches;
		typedef typename PointMatcher<T>::OutlierWeights OutlierWeights;
		typedef typename PointMatcher<T>::ErrorMinimizer ErrorMinimizer;
		typedef typename PointMatcher<T>::ErrorMinimizer::ErrorElements ErrorElements;
		typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
		typedef typename PointMatcher<T>::Vector Vector;
		typedef typename PointMatcher<T>::Matrix Matrix;

		inline static const std::string description()
		{
				return "Point-to-plane error (or point-to-line in 2D). Based on \\cite{Chen1991Point2Plane}. Covariance estimation based on \\cite{Censi2007ICPCovariance}.";
		}

		inline static const ParametersDoc availableParameters()
		{
				return boost::assign::list_of<ParameterDoc>
						( "force2D", "If set to true(1), the minimization will be force to give a solution in 2D (i.e., on the XY-plane) even with 3D inputs.", "0", "0", "1", &P::Comp<bool>)
						( "sensorStdDev", "sensor standard deviation", "0.01", "0.", "inf", &P::Comp<T>)
						;
		}

		const bool force2D;
		const T sensorStdDev;
		Matrix covMatrix;

		PointToPlaneWithCovErrorMinimizer(const Parameters& params = Parameters());
		virtual TransformationParameters compute(const ErrorElements& mPts);
		virtual T getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const;
		virtual T getOverlap() const;
		virtual Matrix getCovariance() const;
		Matrix estimateCovariance(const ErrorElements& mPts, const TransformationParameters& transformation);
};


template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x);
