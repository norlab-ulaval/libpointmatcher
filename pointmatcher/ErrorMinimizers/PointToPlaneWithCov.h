#ifndef POINT_TO_PLANE_WITH_COV_ERROR_MINIMIZER_H
#define POINT_TO_PLANE_WITH_COV_ERROR_MINIMIZER_H

#include "PointMatcher.h"
#include "ErrorMinimizers/PointToPlane.h"

template<typename T>
struct PointToPlaneWithCovErrorMinimizer: public PointToPlaneErrorMinimizer<T>
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
        return "PointToPlaneWithCovErrorMinimizer";
    }

    inline static const std::string description()
    {
        return "Point-to-plane error (or point-to-line in 2D). Based on \\cite{Chen1991Point2Plane}. Covariance estimation based on \\cite{Censi2007ICPCovariance}.";
    }

    static inline const ParametersDoc availableParameters()
    {
        return boost::assign::list_of<ParameterDoc>
            ( "force2D", "If set to true(1), the minimization will be force to give a solution in 2D (i.e., on the XY-plane) even with 3D inputs.", "0", "0", "1", &P::Comp<bool>)
            ( "sensorStdDev", "sensor standard deviation", "0.01", "0.", "inf", &P::Comp<T>)
            ;
    }

    const T sensorStdDev;
    Matrix covMatrix;

    PointToPlaneWithCovErrorMinimizer(const Parameters& params = Parameters());
    virtual TransformationParameters compute(const ErrorElements& mPts);
    virtual Matrix getCovariance() const;
    Matrix estimateCovariance(const ErrorElements& mPts, const TransformationParameters& transformation);
};

#endif
