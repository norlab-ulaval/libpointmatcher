#include "point_to_point.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToPoint(py::module& p_module)
		{
			using PointToPointErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPointErrorMinimizer;
			py::class_<PointToPointErrorMinimizer, std::shared_ptr<PointToPointErrorMinimizer>, ErrorMinimizer>(p_module, "PointToPointErrorMinimizer")
				.def(py::init<>())
				.def(py::init<const std::string&, const ParametersDoc, const Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"))
				.def("compute", &PointToPointErrorMinimizer::compute, py::arg("mPts"))
				.def("compute_in_place", &PointToPointErrorMinimizer::compute_in_place, py::arg("mPts"))
				.def("getResidualError", &PointToPointErrorMinimizer::getResidualError, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				.def("getOverlap", &PointToPointErrorMinimizer::getOverlap)
				.def_static("computeResidualError", &PointToPointErrorMinimizer::computeResidualError, py::arg("mPts"));
		}
	}
}
