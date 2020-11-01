#include "random_sampling.h"

#include "DataPointsFilters/RandomSampling.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindRandomSampling(py::module& p_module)
		{
			using RandomSamplingDataPointsFilter = RandomSamplingDataPointsFilter<ScalarType>;
			py::class_<RandomSamplingDataPointsFilter, std::shared_ptr<RandomSamplingDataPointsFilter>, DataPointsFilter>(p_module, "RandomSamplingDataPointsFilter", "Random sampling")

				.def_static("description", &RandomSamplingDataPointsFilter::description)
				.def_static("availableParameters", &RandomSamplingDataPointsFilter::availableParameters)

				.def_readonly("prob", &RandomSamplingDataPointsFilter::prob)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &RandomSamplingDataPointsFilter::filter)
				.def("inPlaceFilter", &RandomSamplingDataPointsFilter::inPlaceFilter);
		}
	}
}
