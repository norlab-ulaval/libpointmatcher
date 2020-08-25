#include "impl.h"

#include "impls/inspectors_impl.h"
#include "impls/matchers_impl.h"
#include "impls/outlier_filters_impl.h"
#include "impls/transformations_impl.h"
#include "impls/transformation_checkers_impl.h"

namespace pointmatcher
{
	void pybindImpl(py::module& p_module)
	{
		pybindInspectorsImpl(p_module);
		pybindMatchersImpl(p_module);
		pybindOutlierFiltersImpl(p_module);
		pybindTransformationsImpl(p_module);
		pybindTransformationCheckersImpl(p_module);
	}
}