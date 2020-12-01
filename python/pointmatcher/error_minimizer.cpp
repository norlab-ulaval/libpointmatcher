#include "error_minimizer.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindErrorMinimizer(py::class_<PM>& p_class)
		{
			py::class_<ErrorMinimizer, std::shared_ptr<ErrorMinimizer>, Parametrizable> pyErrorMinimizer(p_class, "ErrorMinimizer");

			pyErrorMinimizer.doc() = R"pbdoc(
An error minimizer will compute a transformation matrix such as to minimize the error between the reading and the reference.

Typical error minimized are point-to-point and point-to-plane.
)pbdoc";

			py::class_<ErrorElements>(pyErrorMinimizer, "ErrorElements", "A structure holding data ready for minimization. The data are \"normalized\", for instance there are no points with 0 weight, etc.")
				.def_readwrite("reading", &ErrorElements::reading, "reading point cloud")
				.def_readwrite("reference", &ErrorElements::reference, "reference point cloud")
				.def_readwrite("weights", &ErrorElements::weights, "weights for every association")
				.def_readwrite("matches", &ErrorElements::matches, "associations")
				.def_readwrite("nbRejectedMatches", &ErrorElements::nbRejectedMatches, "number of matches with zero weights")
				.def_readwrite("nbRejectedPoints", &ErrorElements::nbRejectedPoints, "number of points with all matches set to zero weights")
				.def_readwrite("pointUsedRatio", &ErrorElements::pointUsedRatio, "the ratio of how many points were used for error minimization")
				.def_readwrite("weightedPointUsedRatio", &ErrorElements::weightedPointUsedRatio, "the ratio of how many points were used (with weight) for error minimization")

				.def(py::init<>())
				.def(py::init<const DataPoints&, const DataPoints&, const OutlierWeights&, const Matches&>(), py::arg("requestedPts"), py::arg("sourcePts"), py::arg("outlierWeights"), py::arg("matches"));

			pyErrorMinimizer.def("getPointUsedRatio", &ErrorMinimizer::getPointUsedRatio)
				.def("getWeightedPointUsedRatio", &ErrorMinimizer::getWeightedPointUsedRatio)
				.def("getErrorElements", &ErrorMinimizer::getErrorElements)
				.def("getOverlap", &ErrorMinimizer::getOverlap).def("getCovariance", &ErrorMinimizer::getCovariance)
				.def("getResidualError", &ErrorMinimizer::getResidualError, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				.def("compute", (TransformationParameters (ErrorMinimizer::*)(const DataPoints&, const DataPoints&, const OutlierWeights&, const Matches&)) &ErrorMinimizer::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				.def_static("crossProduct", &ErrorMinimizer::crossProduct, py::arg("A"), py::arg("B"));

		}
	}
}
