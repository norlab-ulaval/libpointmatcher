#include "covariance_sampling.h"

#include "DataPointsFilters/CovarianceSampling.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindCovarianceSampling(py::module& p_module)
		{
			using CovarianceSamplingDataPointsFilter = CovarianceSamplingDataPointsFilter<ScalarType>;
			py::class_<CovarianceSamplingDataPointsFilter, std::shared_ptr<CovarianceSamplingDataPointsFilter>, DataPointsFilter> cosamplingClass(p_module, "CovarianceSamplingDataPointsFilter");

			using TorqueNormMethod = CovarianceSamplingDataPointsFilter::TorqueNormMethod;
			py::enum_<TorqueNormMethod>(cosamplingClass, "TorqueNormMethod").value("L1", TorqueNormMethod::L1)
				.value("Lavg", TorqueNormMethod::Lavg).value("Lmax", TorqueNormMethod::Lmax).export_values();

			cosamplingClass.def_static("description", &CovarianceSamplingDataPointsFilter::description)
				.def_static("availableParameters", &CovarianceSamplingDataPointsFilter::availableParameters)
				.def_static("computeConditionNumber", &CovarianceSamplingDataPointsFilter::computeConditionNumber, py::arg("cov"))

				.def_readwrite("nbSample", &CovarianceSamplingDataPointsFilter::nbSample)
				.def_readonly("normalizationMethod", &CovarianceSamplingDataPointsFilter::normalizationMethod)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &CovarianceSamplingDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &CovarianceSamplingDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}
