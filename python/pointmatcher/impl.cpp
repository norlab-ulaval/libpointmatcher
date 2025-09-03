#include "impl.h"

#include "impls/inspectors_impl.h"
#include "impls/matchers_impl.h"
#include "impls/outlier_filters_impl.h"
#include "impls/transformations_impl.h"
#include "impls/transformation_checkers_impl.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindImpl(py::module& p_module)
		{
			impls::pybindInspectorsImpl(p_module);
			impls::pybindMatchersImpl(p_module);
			impls::pybindOutlierFiltersImpl(p_module);
			impls::pybindTransformationsImpl(p_module);
			impls::pybindTransformationCheckersImpl(p_module);
		}
	}
}
