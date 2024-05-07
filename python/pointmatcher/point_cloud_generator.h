//
// Created by Matěj Boxan on 2024-05-02.
//

#ifndef LIBPOINTMATCHER_POINT_CLOUD_GENERATOR_H
#define LIBPOINTMATCHER_POINT_CLOUD_GENERATOR_H


#include "pypoint_matcher_helper.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindPointCloudGenerator(py::module& p_module);
	}
}

#endif //LIBPOINTMATCHER_POINT_CLOUD_GENERATOR_H
