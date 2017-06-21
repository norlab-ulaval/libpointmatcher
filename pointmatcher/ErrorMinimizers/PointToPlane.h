#ifndef POINT_TO_PLANE_ERROR_MINIMIZER_H
#define POINT_TO_PLANE_ERROR_MINIMIZER_H

#include "PointMatcher.h"

template<typename T>
struct PointToPlaneErrorMinimizer: public PointMatcher<T>::ErrorMinimizer
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

	virtual inline const std::string name()
	{
		return "PointToPlaneErrorMinimizer";
	}

    inline static const std::string description()
    {
        return "Point-to-plane error (or point-to-line in 2D). Per \\cite{Chen1991Point2Plane}.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return boost::assign::list_of<ParameterDoc>
            ( "force2D", "If set to true(1), the minimization will be force to give a solution in 2D (i.e., on the XY-plane) even with 3D inputs.", "0", "0", "1", &P::Comp<bool>)
            ;
    }

    const bool force2D;

    PointToPlaneErrorMinimizer(const Parameters& params = Parameters());
    PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params);
    //virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches);
    virtual TransformationParameters compute(const ErrorElements& mPts);
	TransformationParameters compute_in_place(ErrorElements& mPts);
    virtual T getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const;
    virtual T getOverlap() const;

    static T computeResidualError(ErrorElements mPts, const bool& force2D);
};

template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x);

#endif
