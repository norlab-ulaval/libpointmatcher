#include "ellipsoids.h"

#include "DataPointsFilters/Elipsoids.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindEllipsoids(py::module& p_module)
		{
			using ElipsoidsDataPointsFilter = ElipsoidsDataPointsFilter<ScalarType>;
			py::class_<ElipsoidsDataPointsFilter, std::shared_ptr<ElipsoidsDataPointsFilter>, DataPointsFilter> ellipsoidClass(p_module, "EllipsoidsDataPointsFilter");

			ellipsoidClass.def_static("description", &ElipsoidsDataPointsFilter::description)
				.def_static("availableParameters", &ElipsoidsDataPointsFilter::availableParameters)

				.def_readonly("ratio", &ElipsoidsDataPointsFilter::ratio)
				.def_readonly("knn", &ElipsoidsDataPointsFilter::knn)
				.def_readonly("samplingMethod", &ElipsoidsDataPointsFilter::samplingMethod)
				.def_readonly("maxBoxDim", &ElipsoidsDataPointsFilter::maxBoxDim)
				.def_readonly("maxTimeWindow", &ElipsoidsDataPointsFilter::maxTimeWindow)
				.def_readonly("minPlanarity", &ElipsoidsDataPointsFilter::minPlanarity)
				.def_readonly("averageExistingDescriptors", &ElipsoidsDataPointsFilter::averageExistingDescriptors)
				.def_readonly("keepNormals", &ElipsoidsDataPointsFilter::keepNormals)
				.def_readonly("keepDensities", &ElipsoidsDataPointsFilter::keepDensities)
				.def_readonly("keepEigenValues", &ElipsoidsDataPointsFilter::keepEigenValues)
				.def_readonly("keepEigenVectors", &ElipsoidsDataPointsFilter::keepEigenVectors)
				.def_readonly("keepCovariances", &ElipsoidsDataPointsFilter::keepCovariances)
				.def_readonly("keepWeights", &ElipsoidsDataPointsFilter::keepWeights)
				.def_readonly("keepMeans", &ElipsoidsDataPointsFilter::keepMeans)
				.def_readonly("keepShapes", &ElipsoidsDataPointsFilter::keepShapes)
				.def_readonly("keepIndices", &ElipsoidsDataPointsFilter::keepIndices)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &ElipsoidsDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &ElipsoidsDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}
