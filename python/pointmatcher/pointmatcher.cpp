#include "pypointmatcher_helper.h"

#include "datapoints.h"
#include "transformation.h"
#include "transformations.h"
#include "datapointsfilter.h"
#include "datapointsfilters.h"
#include "inspector.h"
#include "matcher.h"
#include "matches.h"
#include "outlierfilter.h"
#include "outlierfilters.h"
#include "errorminimizer.h"
#include "transformationchecker.h"
#include "transformationcheckers.h"
#include "icpchainbase.h"
#include "icp.h"
#include "icpsequence.h"

namespace pm = pointmatcher;

namespace pointmatcher
{
	void pybindPointMatcher(py::module& p_module)
	{
		py::class_<PM> pyPointmatcher(p_module, "PointMatcher");

		pyPointmatcher.doc() = "Functions and classes that are dependant on scalar type are defined in this templatized class";

		using ConvergenceError = PM::ConvergenceError;
		py::register_exception<ConvergenceError>(pyPointmatcher, "ConvergenceError");

		pybindDataPoints(pyPointmatcher);
		pybindMatches(pyPointmatcher);
		pybindTranformation(pyPointmatcher);
		pybindTranformations(pyPointmatcher);
		pybindDataPointsFilter(pyPointmatcher);
		pybindDataPointsFilters(pyPointmatcher);
		pybindMatcher(pyPointmatcher);
		pybindOutlierFilter(pyPointmatcher);
		pybindOutlierFilters(pyPointmatcher);
		pybindErrorMinimizer(pyPointmatcher);
		pybindTransformationChecker(pyPointmatcher);
		pybindTransformationCheckers(pyPointmatcher);
		pybindInspector(pyPointmatcher);
		pybindICPChainBase(pyPointmatcher);
		pybindICP(pyPointmatcher);
		pybindICPSequence(pyPointmatcher);

		pyPointmatcher
			.def_static("swapDataPoints", &PM::swapDataPoints, py::arg("a"), py::arg("b"))
			.def_readwrite("TransformationRegistrar", &PM::TransformationRegistrar)
			.def_readwrite("DataPointsFilterRegistrar", &PM::DataPointsFilterRegistrar)
			.def_readwrite("MatcherRegistrar", &PM::MatcherRegistrar)
			.def_readwrite("OutlierFilterRegistrar", &PM::OutlierFilterRegistrar)
			.def_readwrite("ErrorMinimizerRegistrar", &PM::ErrorMinimizerRegistrar)
			.def_readwrite("TransformationCheckerRegistrar", &PM::TransformationCheckerRegistrar)
			.def_readwrite("InspectorRegistrar", &PM::InspectorRegistrar)
			.def_readwrite("LoggerRegistrar", &PM::LoggerRegistrar)
			.def_static("get", &PM::get);
	}
}