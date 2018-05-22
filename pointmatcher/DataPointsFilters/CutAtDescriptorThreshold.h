#pragma once

#include "PointMatcher.h"

#include <string>

//! Subsampling. Cut points with value of a given descriptor above or below a given threshold.
template<typename T>
struct CutAtDescriptorThresholdDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
  inline static const std::string description()
  {
    return "Subsampling. Cut points with value of a given descriptor above or below a given threshold.";
  }
  inline static const ParametersDoc availableParameters()
  {
    return boost::assign::list_of<ParameterDoc>
    ( "descName", "Descriptor name used to cut points", "none")
    ( "useLargerThan", "If set to 1 (true), points with values above the 'threshold' will be cut.  If set to 0 (false), points with values below the 'threshold' will be cut.", "1", "0", "1", P::Comp<bool>)
    ( "threshold", "Value at which to cut.", "0", "-inf", "inf", &P::Comp<T>)
    ;
  }

  const std::string descName;
  const bool useLargerThan;
  const T threshold;

  //! Constructor, uses parameter interface
  CutAtDescriptorThresholdDataPointsFilter(const Parameters& params = Parameters());
  virtual DataPoints filter(const DataPoints& input);
  virtual void inPlaceFilter(DataPoints& cloud);
};
