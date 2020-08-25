#ifndef PYTHON_POINTMATCHER_IMPLS_TRANSFORMATIONS_IMPL_H
#define PYTHON_POINTMATCHER_IMPLS_TRANSFORMATIONS_IMPL_H

#include "TransformationsImpl.h"
#include "pypointmatcher_helper.h"

namespace pointmatcher
{
	using TransformationsImpl = TransformationsImpl<ScalarType>;
	using RigidTransformation = TransformationsImpl::RigidTransformation;
	using SimilarityTransformation = TransformationsImpl::SimilarityTransformation;
	using PureTranslation = TransformationsImpl::PureTranslation;

	void pybindTransformationsImpl(py::module& p_module);
}

#endif //PYTHON_POINTMATCHER_IMPLS_TRANSFORMATIONS_IMPL_H
