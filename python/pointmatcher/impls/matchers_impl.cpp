#include "matchers_impl.h"

#include "pointmatcher/MatchersImpl.h"

namespace python
{
	namespace pointmatcher
	{
		namespace impls
		{
			void pybindMatchersImpl(py::module& p_module)
			{
				using MatchersImpl = MatchersImpl<ScalarType>;
				py::class_<MatchersImpl, std::shared_ptr<MatchersImpl>> pyMatchersImpl(p_module, "MatchersImpl");

				using NullMatcher = MatchersImpl::NullMatcher;
				py::class_<NullMatcher, std::shared_ptr<NullMatcher>, Matcher>(pyMatchersImpl, "NullMatcher")
					.def_static("description", &NullMatcher::description)

					.def(py::init<>()).def("init", &NullMatcher::init).def("findClosests", &NullMatcher::findClosests);

				using KDTreeMatcher = MatchersImpl::KDTreeMatcher;
				py::class_<KDTreeMatcher, std::shared_ptr<KDTreeMatcher>, Matcher>(pyMatchersImpl, "KDTreeMatcher")
					.def_static("description", &KDTreeMatcher::description)
					.def_static("availableParameters", &KDTreeMatcher::availableParameters)

					.def_readonly("knn", &KDTreeMatcher::knn).def_readonly("epsilon", &KDTreeMatcher::epsilon)
					.def_readonly("searchType", &KDTreeMatcher::searchType)
					.def_readonly("maxDist", &KDTreeMatcher::maxDist)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("init", &KDTreeMatcher::init).def("findClosests", &KDTreeMatcher::findClosests);

				using KDTreeVarDistMatcher = MatchersImpl::KDTreeVarDistMatcher;
				py::class_<KDTreeVarDistMatcher, std::shared_ptr<KDTreeVarDistMatcher>, Matcher>(pyMatchersImpl, "KDTreeVarDistMatcher")
					.def_static("description", &KDTreeVarDistMatcher::description)
					.def_static("availableParameters", &KDTreeVarDistMatcher::availableParameters)

					.def_readonly("knn", &KDTreeVarDistMatcher::knn)
					.def_readonly("epsilon", &KDTreeVarDistMatcher::epsilon)
					.def_readonly("searchType", &KDTreeVarDistMatcher::searchType)
					.def_readonly("maxDistField", &KDTreeVarDistMatcher::maxDistField)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("init", &KDTreeVarDistMatcher::init).def("findClosests", &KDTreeVarDistMatcher::findClosests);
			}
		}
	}
}
