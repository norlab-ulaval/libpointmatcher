//
// Created by Matěj Boxan on 2024-05-02.
//

#include "point_cloud_generator.h"

#include "pointmatcher/PointMatcher.h"

namespace python
{
    namespace pointmatcher
    {

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param translation[in]   Translation.
        //! @param rotation[in]      Rotation.
        //! @param pointCloud[out]   Point cloud to transform
        void applyTransformation(
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation,
                DataPoints& pointCloud)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            Generator::applyTransformation(translation, quaternion, pointCloud);
        }

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param length[in]          Length of the box. [m]
        //! @param width[in]           Width of the box. [m]
        //! @param height[in]          Height of the box. [m]
        //! @param numberOfPoints[in]  Number of points.
        //! @param translation[in]     Translation with respect to the box origin, to be used for positioning the box.
        //! @param rotation[in]        Rotation with respect to the box origin, to be used for positioning the box.
        //! @return DataPoints  Box's point cloud.
        DataPoints generateUniformlySampledBoxWrapper(
                const ScalarType length, const ScalarType width, const ScalarType height, const DataPoints::Index numberOfPoints,
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            return Generator::generateUniformlySampledBox(length, width, height, numberOfPoints, translation, quaternion);
        }

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param dimensions[in]      Dimensions of the plane (length, width, height). [m]
        //! @param numberOfPoints[in]  Number of points.
        //! @param translation[in]     Translation with respect to the plane origin, to be used for positioning the plane.
        //! @param rotation[in]        Rotation with respect to the plane origin, to be used for positioning the plane.
        //! @return DataPoints  Plane's point cloud.
        DataPoints generateUniformlySampledPlane(
                const StaticCoordVector& dimensions, const DataPoints::Index numberOfPoints,
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            return Generator::generateUniformlySampledPlane(dimensions, numberOfPoints, translation, quaternion);
        }

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param radius[in]          Radius of the cylinder. [m]
        //! @param height[in]          Height of the cylinder. [m]
        //! @param numberOfPoints[in]  Number of points.
        //! @param translation[in]     Translation with respect to the cylinder origin, to be used for positioning the cylinder.
        //! @param rotation[in]        Rotation with respect to the cylinder origin, to be used for positioning the cylinder.
        //! @return DataPoints  Circle's point cloud.
        DataPoints generateUniformlySampledCylinder(
                const ScalarType radius, const ScalarType height, const DataPoints::Index numberOfPoints,
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            return Generator::generateUniformlySampledCylinder(radius, height, numberOfPoints, translation, quaternion);
        }

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param radius[in]          Radius of the circle. [m]
        //! @param numberOfPoints[in]  Number of points.
        //! @param translation[in]     Translation with respect to the circle origin, to be used for positioning the circle.
        //! @param rotation[in]        Rotation with respect to the circle origin, to be used for positioning the circle.
        //! @return DataPoints  Circle's point cloud.
        DataPoints generateUniformlySampledCircle(
                const ScalarType radius, const DataPoints::Index numberOfPoints,
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            return Generator::generateUniformlySampledCircle(radius, numberOfPoints, translation, quaternion);
        }

        //! @brief Wrapper class that converts general Eigen::Matrix, convertible from numpy array, to Eigen::Quaternion.
        //!
        //! @param radius[in]          Radius of the sphere. [m]
        //! @param numberOfPoints[in]  Number of points.
        //! @param translation[in]     Translation with respect to the sphere origin, to be used for positioning the sphere.
        //! @return DataPoints  Sphere's point cloud.
        DataPoints generateUniformlySampledSphere(
                const ScalarType radius, const DataPoints::Index numberOfPoints,
                const StaticCoordVector& translation, const Eigen::Matrix<ScalarType, 4, 1>& rotation)
        {
            PM::Quaternion quaternion(rotation(0), rotation(1), rotation(2), rotation(3));
            return Generator::generateUniformlySampledSphere(radius, numberOfPoints, translation, quaternion);
        }

        void pybindPointCloudGenerator(py::module& p_module)
        {
            py::class_<Generator>(p_module, "PointCloudGenerator", "Class containing methods to generate point clouds in the shape of geometric primitives.")
                    .def_static("buildUpTransformation",
                                &Generator::buildUpTransformation,
                                py::arg("translation"),
                                py::arg("rotation"))
                    .def_static("addEmpty3dPointFields", &Generator::addEmpty3dPointFields, py::arg("numberOfPoints"), py::arg("pointCloud"))
                    .def_static("applyTransformation", &applyTransformation, py::arg("translation"), py::arg("rotation"), py::arg("pointCloud"))
                    .def_static("computeNormalOfAxisAlignedPlane", &Generator::computeNormalOfAxisAlignedPlane, py::arg("axisAlignedPlaneDimensions"))
                    .def_static("generateUniformlySampledSphere", &generateUniformlySampledSphere, py::arg("radius"), py::arg("numberOfPoints"), py::arg("translation"),
                                py::arg("rotation"))
                    .def_static("generateUniformlySampledCircle", &generateUniformlySampledCircle, py::arg("radius"), py::arg("numberOfPoints"), py::arg("translation"),
                                py::arg("rotation"))
                    .def_static("generateUniformlySampledCylinder", &generateUniformlySampledCylinder, py::arg("radius"), py::arg("height"), py::arg("numberOfPoints"),
                                py::arg("translation"), py::arg("rotation"))
                    .def_static("generateUniformlySampledPlane", &generateUniformlySampledPlane, py::arg("dimensions"), py::arg("numberOfPoints"),
                                py::arg("translation"), py::arg("rotation"))
                    .def_static("generateUniformlySampledBox", &generateUniformlySampledBoxWrapper, py::arg("length"), py::arg("width"), py::arg("height"),
                                py::arg("numberOfPoints"), py::arg("translation"), py::arg("rotation"));
        }
    }
}