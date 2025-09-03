#include "point_to_gaussian.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToGaussian(py::module& p_module)
		{
			using PointToGaussianErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToGaussianErrorMinimizer;
			py::class_<PointToGaussianErrorMinimizer, std::shared_ptr<PointToGaussianErrorMinimizer>, ErrorMinimizer>(p_module, "PointToGaussianErrorMinimizer")
				.def(py::init<const Parameters&>(), py::arg("params") = Parameters())

				.def_readonly("force2D", &PointToGaussianErrorMinimizer::force2D)
				.def_readonly("force4D", &PointToGaussianErrorMinimizer::force2D)
				.def_readonly("confidenceInPenalties", &PointToGaussianErrorMinimizer::confidenceInPenalties)

				.def_static("description", &PointToGaussianErrorMinimizer::description)
				.def_static("availableParameters", &PointToGaussianErrorMinimizer::availableParameters)

				.def("name", &PointToGaussianErrorMinimizer::name)
				.def("compute", &PointToGaussianErrorMinimizer::compute, py::arg("mPts"))
				.def("getResidualError", &PointToGaussianErrorMinimizer::getResidualError, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				// .def("convertCovariancesToNormals", &PointToGaussianErrorMinimizer::getOverlap, py::arg("mPts"))

				.def_static("computeResidualError", &PointToGaussianErrorMinimizer::computeResidualError, py::arg("mPts"), py::arg("force2D"));
		}
	}
}
