#include <fstream>

#include "icp_chain_base.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindICPChainBase(py::class_<PM>& p_class)
		{
			py::class_<ICPChaineBase>(p_class, "ICPChainBase", "Stuff common to all ICP algorithms")
				.def_readwrite("readingDataPointsFilters", &ICPChaineBase::readingDataPointsFilters, "filters for reading, applied once")
				.def_readwrite("readingStepDataPointsFilters", &ICPChaineBase::readingStepDataPointsFilters, "filters for reading, applied at each step")
				.def_readwrite("referenceDataPointsFilters", &ICPChaineBase::referenceDataPointsFilters, "filters for reference")
				.def_readwrite("transformations", &ICPChaineBase::transformations, "transformations")
				.def_readwrite("matcher", &ICPChaineBase::matcher, "matcher")
				.def_readwrite("outlierFilters", &ICPChaineBase::outlierFilters, "outlier filters")
				.def_readwrite("errorMinimizer", &ICPChaineBase::errorMinimizer, "error minimizer")
				.def_readwrite("transformationCheckers", &ICPChaineBase::transformationCheckers, "transformations checkers")
				.def_readwrite("inspector", &ICPChaineBase::inspector, "inspector")

				.def("setDefault", &ICPChaineBase::setDefault)

				.def("loadFromYaml", [](ICPChaineBase& self, const std::string& in)
				{
					std::ifstream ifs(in.c_str());
					self.loadFromYaml(ifs);
				}).def("getPrefilteredReadingPtsCount", &ICPChaineBase::getPrefilteredReadingPtsCount)
				.def("getPrefilteredReferencePtsCount", &ICPChaineBase::getPrefilteredReferencePtsCount)

				.def("getMaxNumIterationsReached", &ICPChaineBase::getMaxNumIterationsReached);
		}
	}
}
