#include "pypoint_matcher_helper.h"

#include "data_points.h"
#include "transformation.h"
#include "transformations.h"
#include "data_points_filter.h"
#include "data_points_filters.h"
#include "inspector.h"
#include "matcher.h"
#include "matches.h"
#include "outlier_filter.h"
#include "outlier_filters.h"
#include "error_minimizer.h"
#include "transformation_checker.h"
#include "transformation_checkers.h"
#include "icp_chain_base.h"
#include "icp.h"
#include "icp_sequence.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindPointMatcher(py::module& p_module)
		{
			py::class_<PM> pyPointmatcher(p_module, "PointMatcher");

			pyPointmatcher
				.doc() = "Functions and classes that are dependant on scalar type are defined in this templatized class";

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

			pyPointmatcher.def_static("swapDataPoints", &PM::swapDataPoints, py::arg("a"), py::arg("b"))
				.def_readwrite("TransformationRegistrar", &PM::TransformationRegistrar)
				.def_readwrite("DataPointsFilterRegistrar", &PM::DataPointsFilterRegistrar)
				.def_readwrite("MatcherRegistrar", &PM::MatcherRegistrar)
				.def_readwrite("OutlierFilterRegistrar", &PM::OutlierFilterRegistrar)
				.def_readwrite("ErrorMinimizerRegistrar", &PM::ErrorMinimizerRegistrar)
				.def_readwrite("TransformationCheckerRegistrar", &PM::TransformationCheckerRegistrar)
				.def_readwrite("InspectorRegistrar", &PM::InspectorRegistrar)
				.def_readwrite("LoggerRegistrar", &PM::LoggerRegistrar)
				.def_static("get", &PM::get, py::return_value_policy::reference);
		}
	}
}
