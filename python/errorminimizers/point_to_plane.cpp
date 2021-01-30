#include "point_to_plane.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToPlane(py::module& p_module)
		{
			using PointToPlaneErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPlaneErrorMinimizer;
			py::class_<PointToPlaneErrorMinimizer, std::shared_ptr<PointToPlaneErrorMinimizer>, ErrorMinimizer>(p_module, "PointToPlaneErrorMinimizer")
				.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
				.def(py::init<const ParametersDoc, const Parameters&>(), py::arg("paramsDoc"), py::arg("params") = Parameters())

				.def_readonly("force2D", &PointToPlaneErrorMinimizer::force2D)
				.def_readonly("force4DOF", &PointToPlaneErrorMinimizer::force4DOF)

				.def_static("description", &PointToPlaneErrorMinimizer::description)
				.def_static("availableParameters", &PointToPlaneErrorMinimizer::availableParameters)

				.def("name", &PointToPlaneErrorMinimizer::name)
				.def("compute", &PointToPlaneErrorMinimizer::compute, py::arg("mPts"))
				.def("compute_in_place", &PointToPlaneErrorMinimizer::compute_in_place, py::arg("mPts"))
				.def("getResidualError", &PointToPlaneErrorMinimizer::getResidualError, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				.def("getOverlap", &PointToPlaneErrorMinimizer::getOverlap)

				.def_static("computeResidualError", &PointToPlaneErrorMinimizer::computeResidualError, py::arg("mPts"), py::arg("force2D"));
		}
	}
}
