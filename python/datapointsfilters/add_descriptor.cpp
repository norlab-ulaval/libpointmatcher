#include "bounding_box.h"

#include "DataPointsFilters/AddDescriptor.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindAddDescriptor(py::module& p_module)
		{
			using AddDescriptorDataPointsFilter = AddDescriptorDataPointsFilter<ScalarType>;
			py::class_<AddDescriptorDataPointsFilter, std::shared_ptr<AddDescriptorDataPointsFilter>, DataPointsFilter>(p_module, "AddDescriptorDataPointsFilter")
				.def_static("description", &AddDescriptorDataPointsFilter::description)
				.def_static("availableParameters", &AddDescriptorDataPointsFilter::availableParameters)

				.def_readonly("descriptorName", &AddDescriptorDataPointsFilter::descriptorName)
				.def_readonly("descriptorDimension", &AddDescriptorDataPointsFilter::descriptorDimension)
				.def_readonly("descriptorValues", &AddDescriptorDataPointsFilter::descriptorValues)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &AddDescriptorDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &AddDescriptorDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}
