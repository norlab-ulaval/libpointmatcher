#include "point_to_point_similarity.h"

#include "pointmatcher/ErrorMinimizersImpl.h"

namespace python
{
	namespace errorminimizers
	{
		void pybindPointToPointSimilarity(py::module& p_module)
		{
			using PointToPointSimilarityErrorMinimizer = ErrorMinimizersImpl<ScalarType>::PointToPointSimilarityErrorMinimizer;
			py::class_<PointToPointSimilarityErrorMinimizer, std::shared_ptr<PointToPointSimilarityErrorMinimizer>, ErrorMinimizer>(p_module, "PointToPointSimilarityErrorMinimizer")
				.def(py::init<>()).def_static("description", &PointToPointSimilarityErrorMinimizer::description)
				.def("compute", &PointToPointSimilarityErrorMinimizer::compute, py::arg("mPts"))
				.def("getResidualError", &PointToPointSimilarityErrorMinimizer::getResidualError, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("outlierWeights"), py::arg("matches"))
				.def("getOverlap", &PointToPointSimilarityErrorMinimizer::getOverlap);
		}
	}
}
