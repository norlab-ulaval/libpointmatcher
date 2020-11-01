#include "point_to_plane_with_cov.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToPlaneWithCov(py::module& p_module)
		{
			using PointToPlaneErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPlaneErrorMinimizer;
			using PointToPlaneWithCovErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPlaneWithCovErrorMinimizer;
			py::class_<PointToPlaneWithCovErrorMinimizer, std::shared_ptr<PointToPlaneWithCovErrorMinimizer>, PointToPlaneErrorMinimizer>(p_module, "PointToPlaneWithCovErrorMinimizer")
				.def("name", &PointToPlaneWithCovErrorMinimizer::name)

				.def_static("description", &PointToPlaneWithCovErrorMinimizer::description)
				.def_static("availableParameters", &PointToPlaneWithCovErrorMinimizer::availableParameters)

				.def_readonly("sensorStdDev", &PointToPlaneWithCovErrorMinimizer::sensorStdDev)
				.def_readwrite("covMatrix", &PointToPlaneWithCovErrorMinimizer::covMatrix)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
				.def("compute", &PointToPlaneWithCovErrorMinimizer::compute, py::arg("mPts"))
				.def("getCovariance", &PointToPlaneWithCovErrorMinimizer::getCovariance)
				.def("estimateCovariance", &PointToPlaneWithCovErrorMinimizer::estimateCovariance, py::arg("mPts"), py::arg("transformation"));
		}
	}
}
