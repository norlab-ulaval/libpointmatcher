#include "compression.h"

#include "DataPointsFilters/Compression.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindCompression(py::module& p_module)
		{
			using CompressionDataPointsFilter = CompressionDataPointsFilter<ScalarType>;
			py::class_<CompressionDataPointsFilter, std::shared_ptr<CompressionDataPointsFilter>, DataPointsFilter>(p_module, "CompressionDataPointsFilter")
					.def_static("description", &CompressionDataPointsFilter::description)
					.def_static("availableParameters", &CompressionDataPointsFilter::availableParameters)

					.def_readonly("knn", &CompressionDataPointsFilter::knn)
					.def_readonly("maxDist", &CompressionDataPointsFilter::maxDist)
					.def_readonly("epsilon", &CompressionDataPointsFilter::epsilon)
					.def_readonly("maxIterationCount", &CompressionDataPointsFilter::maxIterationCount)
					.def_readonly("initialVariance", &CompressionDataPointsFilter::initialVariance)
					.def_readonly("maxDeviation", &CompressionDataPointsFilter::maxDeviation)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

					.def("filter", &CompressionDataPointsFilter::filter, py::arg("input"))
					.def("inPlaceFilter", &CompressionDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}
