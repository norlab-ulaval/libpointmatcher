#include "errorminimizers_module.h"

#include "errorminimizers/identity.h"
#include "errorminimizers/pointtoplane.h"
#include "errorminimizers/pointtoplanewithcov.h"
#include "errorminimizers/pointtopoint.h"
#include "errorminimizers/pointtopointsimilarity.h"
#include "errorminimizers/pointtopointwithcov.h"

namespace pointmatcher
{
	void pybindErrorMinimizersModule(py::module& p_module)
	{
		py::module errorminizersModule = p_module.def_submodule("errorminimizers");

		pybindIdentityEM(errorminizersModule);
		pybindPointToPlane(errorminizersModule);
		pybindPointToPlaneWithCov(errorminizersModule);
		pybindPointToPoint(errorminizersModule);
		pybindPointToPointSimilarity(errorminizersModule);
		pybindPointToPointWithCov(errorminizersModule);
	}
}
