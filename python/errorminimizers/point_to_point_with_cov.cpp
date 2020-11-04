#include "point_to_point_with_cov.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToPointWithCov(py::module& p_module)
		{
			using PointToPointErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPointErrorMinimizer;
			using PointToPointWithCovErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPointWithCovErrorMinimizer;
			py::class_<PointToPointWithCovErrorMinimizer, std::shared_ptr<PointToPointWithCovErrorMinimizer>, PointToPointErrorMinimizer>(p_module, "PointToPointWithCovErrorMinimizer")
				.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
				.def_readonly("sensorStdDev", &PointToPointWithCovErrorMinimizer::sensorStdDev)
				.def_readwrite("covMatrix", &PointToPointWithCovErrorMinimizer::covMatrix)
				.def_static("description", &PointToPointWithCovErrorMinimizer::description)
				.def_static("availableParameters", &PointToPointWithCovErrorMinimizer::availableParameters)
				.def("compute", &PointToPointWithCovErrorMinimizer::compute, py::arg("mPts"))
				.def("getCovariance", &PointToPointWithCovErrorMinimizer::getCovariance)
				.def("estimateCovariance", &PointToPointWithCovErrorMinimizer::estimateCovariance, py::arg("mPts"), py::arg("transformation"));
		}
	}
}
