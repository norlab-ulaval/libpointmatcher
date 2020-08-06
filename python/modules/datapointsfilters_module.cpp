#include "datapointsfilters_module.h"

#include "datapointsfilters/boundingbox.h"
#include "datapointsfilters/covariancesampling.h"
#include "datapointsfilters/cutatdescriptorthreshold.h"
#include "datapointsfilters/distancelimit.h"
#include "datapointsfilters/ellipsoids.h"
#include "datapointsfilters/fixstepsampling.h"
#include "datapointsfilters/gestalt.h"
#include "datapointsfilters/identity.h"
#include "datapointsfilters/incidenceangle.h"
#include "datapointsfilters/maxdensity.h"
#include "datapointsfilters/maxpointcount.h"
#include "datapointsfilters/maxquantileonaxis.h"
#include "datapointsfilters/normalspace.h"
#include "datapointsfilters/observationdirection.h"
#include "datapointsfilters/octreegrid.h"
#include "datapointsfilters/orientnormals.h"
#include "datapointsfilters/randomsampling.h"
#include "datapointsfilters/removenan.h"
#include "datapointsfilters/removesensorbias.h"
#include "datapointsfilters/samplingsurfacenormal.h"
#include "datapointsfilters/shadow.h"
#include "datapointsfilters/simplesensornoise.h"
#include "datapointsfilters/sphericality.h"
#include "datapointsfilters/surfacenormal.h"

namespace pointmatcher
{
	void pybindDataPointsFiltersModule(py::module& p_module)
	{
		py::module datapointsfilterModule = p_module.def_submodule("datapointsfilters");

		pybindBoundingBox(datapointsfilterModule);
		pybindCovarianceSampling(datapointsfilterModule);
		pybindCutAtDescriptorThreshold(datapointsfilterModule);
		pybindDistanceLimit(datapointsfilterModule);
		pybindEllipsoids(datapointsfilterModule);
		pybindFixStepSampling(datapointsfilterModule);
		pybindGestalt(datapointsfilterModule);
		pybindIdentityDPF(datapointsfilterModule);
		pybindIncidenceAngle(datapointsfilterModule);
		pybindMaxDensity(datapointsfilterModule);
		pybindMaxPointCount(datapointsfilterModule);
		pybindMaxQuantileOnAxis(datapointsfilterModule);
		pybindNormalSpace(datapointsfilterModule);
		pybindObservationDirection(datapointsfilterModule);
		pybindOctreeGrid(datapointsfilterModule);
		pybindOrientNormals(datapointsfilterModule);
		pybindRandomSampling(datapointsfilterModule);
		pybindRemoveNaN(datapointsfilterModule);
		pybindRemoveSensorBias(datapointsfilterModule);
		pybindSamplingSurfaceNormal(datapointsfilterModule);
		pybindShadow(datapointsfilterModule);
		pybindSimpleSensorNoise(datapointsfilterModule);
		pybindSphericality(datapointsfilterModule);
		pybindSurfaceNormal(datapointsfilterModule);
	}
}
