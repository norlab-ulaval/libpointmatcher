#include "outlier_filters_impl.h"

#include "pointmatcher/OutlierFiltersImpl.h"

namespace python
{
	namespace pointmatcher
	{
		namespace impls
		{
			void pybindOutlierFiltersImpl(py::module& p_module)
			{
				using OutlierFiltersImpl = OutlierFiltersImpl<ScalarType>;
				py::class_<OutlierFiltersImpl, std::shared_ptr<OutlierFiltersImpl>> pyOutlierFilterImpl(p_module, "OutlierFiltersImpl");

				using NullOutlierFilter = OutlierFiltersImpl::NullOutlierFilter;
				py::class_<NullOutlierFilter, std::shared_ptr<NullOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "NullOutlierFilter")
					.def_static("description", &NullOutlierFilter::description)

					.def(py::init<>())
					.def("compute", &NullOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using MaxDistOutlierFilter = OutlierFiltersImpl::MaxDistOutlierFilter;
				py::class_<MaxDistOutlierFilter, std::shared_ptr<MaxDistOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "MaxDistOutlierFilter")
					.def_static("description", &MaxDistOutlierFilter::description)
					.def_static("availableParameters", &MaxDistOutlierFilter::availableParameters)

					.def_readonly("maxDist", &MaxDistOutlierFilter::maxDist)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &MaxDistOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using MinDistOutlierFilter = OutlierFiltersImpl::MinDistOutlierFilter;
				py::class_<MinDistOutlierFilter, std::shared_ptr<MinDistOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "MinDistOutlierFilter")
					.def_static("description", &MinDistOutlierFilter::description)
					.def_static("availableParameters", &MinDistOutlierFilter::availableParameters)

					.def_readonly("minDist", &MinDistOutlierFilter::minDist)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &MinDistOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using MedianDistOutlierFilter = OutlierFiltersImpl::MedianDistOutlierFilter;
				py::class_<MedianDistOutlierFilter, std::shared_ptr<MedianDistOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "MedianDistOutlierFilter")
					.def_static("description", &MedianDistOutlierFilter::description)
					.def_static("availableParameters", &MedianDistOutlierFilter::availableParameters)

					.def_readonly("factor", &MedianDistOutlierFilter::factor)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &MedianDistOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using TrimmedDistOutlierFilter = OutlierFiltersImpl::TrimmedDistOutlierFilter;
				py::class_<TrimmedDistOutlierFilter, std::shared_ptr<TrimmedDistOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "TrimmedDistOutlierFilter")
					.def_static("description", &TrimmedDistOutlierFilter::description)
					.def_static("availableParameters", &TrimmedDistOutlierFilter::availableParameters)

					.def_readonly("ratio", &TrimmedDistOutlierFilter::ratio)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &TrimmedDistOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using VarTrimmedDistOutlierFilter = OutlierFiltersImpl::VarTrimmedDistOutlierFilter;
				py::class_<VarTrimmedDistOutlierFilter, std::shared_ptr<VarTrimmedDistOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "VarTrimmedDistOutlierFilter")
					.def_static("description", &VarTrimmedDistOutlierFilter::description)
					.def_static("availableParameters", &VarTrimmedDistOutlierFilter::availableParameters)

					.def_readonly("minRatio", &VarTrimmedDistOutlierFilter::minRatio)
					.def_readonly("maxRatio", &VarTrimmedDistOutlierFilter::maxRatio)
					.def_readonly("lambda", &VarTrimmedDistOutlierFilter::lambda)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &VarTrimmedDistOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using SurfaceNormalOutlierFilter = OutlierFiltersImpl::SurfaceNormalOutlierFilter;
				py::class_<SurfaceNormalOutlierFilter, std::shared_ptr<SurfaceNormalOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "SurfaceNormalOutlierFilter")
					.def_static("description", &SurfaceNormalOutlierFilter::description)
					.def_static("availableParameters", &SurfaceNormalOutlierFilter::availableParameters)

					.def_readonly("eps", &SurfaceNormalOutlierFilter::eps)
					.def_readonly("warningPrinted", &SurfaceNormalOutlierFilter::warningPrinted)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &SurfaceNormalOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using GenericDescriptorOutlierFilter = OutlierFiltersImpl::GenericDescriptorOutlierFilter;
				py::class_<GenericDescriptorOutlierFilter, std::shared_ptr<GenericDescriptorOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "GenericDescriptorOutlierFilter")
					.def_static("description", &GenericDescriptorOutlierFilter::description)
					.def_static("availableParameters", &GenericDescriptorOutlierFilter::availableParameters)

					.def_readonly("eps", &GenericDescriptorOutlierFilter::source)
					.def_readonly("eps", &GenericDescriptorOutlierFilter::descName)
					.def_readonly("eps", &GenericDescriptorOutlierFilter::useSoftThreshold)
					.def_readonly("eps", &GenericDescriptorOutlierFilter::useLargerThan)
					.def_readonly("warningPrinted", &GenericDescriptorOutlierFilter::threshold)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("compute", &GenericDescriptorOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"));

				using RobustOutlierFilter = OutlierFiltersImpl::RobustOutlierFilter;
				py::class_<RobustOutlierFilter, std::shared_ptr<RobustOutlierFilter>, OutlierFilter>(pyOutlierFilterImpl, "RobustOutlierFilter")
					.def_static("description", &RobustOutlierFilter::description)
					.def_static("availableParameters", &RobustOutlierFilter::availableParameters)
					.def("computePointToPlaneDistance", &RobustOutlierFilter::computePointToPlaneDistance)
					.def("compute", &RobustOutlierFilter::compute, py::arg("filteredReading"), py::arg("filteredReference"), py::arg("input"))

					.def(py::init<const std::string&, const ParametersDoc, const Parameters&>(), py::arg("className"), py::arg("paramsDoc"), py::arg("params"))
					.def(py::init<const Parameters&>(), py::arg("params") = Parameters());
			}
		}
	}
}
