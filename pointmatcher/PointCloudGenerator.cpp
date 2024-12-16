// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2023,
Yoshua Nava, ANYbotics AG, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "PointMatcher.h"

#include <memory>
#include <random>

template<typename T>
typename PointMatcher<T>::PointCloudGenerator::AffineTransform PointMatcher<T>::PointCloudGenerator::buildUpTransformation(const StaticCoordVector& translation,
                                                                                                      const Quaternion& rotation)
{
    AffineTransform transformation;
    transformation.translation() = translation;
    transformation.linear() = rotation.normalized().toRotationMatrix();
    return transformation;
}

template<typename T>
void PointMatcher<T>::PointCloudGenerator::addEmpty3dPointFields(const Index numberOfPoints, DataPoints& pointCloud)
{
    using Label = typename DataPoints::Label;

    // Add features.
    pointCloud.features = Matrix::Ones(4, numberOfPoints);

    // Add feature labels.
    pointCloud.featureLabels.push_back(Label("x", 1));
    pointCloud.featureLabels.push_back(Label("y", 1));
    pointCloud.featureLabels.push_back(Label("z", 1));
    pointCloud.featureLabels.push_back(Label("pad", 1));

    // Add descriptors.
    pointCloud.addDescriptor("normals", Matrix::Constant(3, numberOfPoints, 0));
}

template<typename T>
void PointMatcher<T>::PointCloudGenerator::applyTransformation(const StaticCoordVector& translation, const Quaternion& rotation,
                                                               DataPoints& pointCloud)
{
    // Convert (translation, rotation) into a representation
    const AffineTransform transformation{ buildUpTransformation(translation, rotation) };

    // Transformation handler;
    std::shared_ptr<Transformation> transformator(PointMatcher<T>::get().REG(Transformation).create("RigidTransformation"));

    // Apply transformation.
    transformator->inPlaceCompute(transformation.matrix(), pointCloud);
}

template<typename T>
typename PointMatcher<T>::PointCloudGenerator::StaticCoordVector PointMatcher<T>::PointCloudGenerator::computeNormalOfAxisAlignedPlane(
    const StaticCoordVector& axisAlignedPlaneDimensions)
{
    StaticCoordVector normalVector{ StaticCoordVector::Zero() };
    for (Index i{ 0 }; i < 3; ++i)
    {
        if (axisAlignedPlaneDimensions(i) == 0)
        {
            normalVector(i) = 1;
        }
    }
    return normalVector;
}

// Reference: http://corysimon.github.io/articles/uniformdistn-on-sphere/
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::PointCloudGenerator::generateUniformlySampledSphere(
    const ScalarType radius,
    const Index numberOfPoints,
    const StaticCoordVector& translation,
    const Quaternion& rotation)
{
    // Create point cloud and add basic structure to fill.
    DataPoints pointCloud;
    addEmpty3dPointFields(numberOfPoints, pointCloud);

    // Create random distribution generators.
    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::uniform_real_distribution<ScalarType> uniformDistribution(0.0f, 1.0f);

    // Sampling in Spherical coordinates.
    auto normalsView{ pointCloud.getDescriptorViewByName("normals") };
    for (Index i{ 0 }; i < numberOfPoints; ++i)
    {
        // Sample random values of theta and phi.
        const ScalarType theta{ 2.0f * pi * uniformDistribution(randomNumberGenerator) };
        const ScalarType phi{ std::acos(1.0f - 2.0f * uniformDistribution(randomNumberGenerator)) };

        // Pre-compute values, such as sine and cosine of phi and theta.
        const ScalarType sinPhi{ std::sin(phi) };
        const ScalarType cosPhi{ std::cos(phi) };
        const ScalarType sinTheta{ std::sin(theta) };
        const ScalarType cosTheta{ std::cos(theta) };

        // Fill features (3D point coordinates)
        pointCloud.features(0, i) = sinPhi * cosTheta * radius; // x
        pointCloud.features(1, i) = sinPhi * sinTheta * radius; // y
        pointCloud.features(2, i) = cosPhi * radius; // z

        // For a sphere, it's possible to compute outward looking normals by simply normalizing the vector going from its center to the
        // form's body.
        const StaticCoordVector normalVector{ pointCloud.features.col(i).head(3).normalized() };

        // Fill normals.
        normalsView(0, i) = normalVector(0);
        normalsView(1, i) = normalVector(1);
        normalsView(2, i) = normalVector(2);
    }

    // Transform point cloud in space.
    applyTransformation(translation, rotation, pointCloud);

    return pointCloud;
}

// Reference: https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::PointCloudGenerator::generateUniformlySampledCircle(
    const ScalarType radius,
    const Index numberOfPoints,
    const StaticCoordVector& translation,
    const Quaternion& rotation)
{
    // Create point cloud and add basic structure to fill.
    DataPoints pointCloud;
    addEmpty3dPointFields(numberOfPoints, pointCloud);

    // Create random distribution generators.
    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::uniform_real_distribution<ScalarType> uniformDistribution(0.0f, 1.0f);

    // Sampling in Cartesian coordinates.
    auto normalsView{ pointCloud.getDescriptorViewByName("normals") };
    for (Index i{ 0 }; i < numberOfPoints; ++i)
    {
        // Sample random values of theta and phi.
        const ScalarType phi{ 2.0f * pi * uniformDistribution(randomNumberGenerator) };
        const ScalarType radiusSample{ static_cast<ScalarType>(radius * sqrt(uniformDistribution(randomNumberGenerator))) };

        // Pre-compute values, such as sine and cosine of phi and theta.
        const ScalarType sinPhi{ std::sin(phi) };
        const ScalarType cosPhi{ std::cos(phi) };

        // Fill features (3D point coordinates)
        pointCloud.features(0, i) = cosPhi * radiusSample; // x
        pointCloud.features(1, i) = sinPhi * radiusSample; // y
        pointCloud.features(2, i) = 0; // z

        // Fill normals.
        normalsView(0, i) = 0;
        normalsView(1, i) = 0;
        normalsView(2, i) = 1;
    }

    // We generated the circle on the ground plane, and now we rotate it based on the translation and rotation arguments of this function.
    applyTransformation(translation, rotation, pointCloud);

    return pointCloud;
}

// Reference: https://stackoverflow.com/questions/2678501/uniform-generation-of-3d-points-on-cylinder-cone
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::PointCloudGenerator::generateUniformlySampledCylinder(
    const ScalarType radius, const ScalarType height, const Index numberOfPoints, const StaticCoordVector& translation,
    const Quaternion& rotation)
{
    // Create point cloud.
    DataPoints pointCloud;

    // Define number of points per section of the volume.
    // Cylinder area S = 2 * pi * r * h + 2 * pi * r^2
    //  where r: radius
    //        h: height
    // The ratio between the caps and the body is given by the ratio between the area of the caps and the total area of the solid.
    // R = (2 * pi * r^2) / (2 * pi * r * h + 2 * pi * r^2) = r / (h + r)
    const ScalarType ratioCaps{ radius / (height + radius) };
    // Round to nearest even number
    const Index numberOfPointsBothCaps{ static_cast<Index>(std::round(static_cast<ScalarType>(numberOfPoints) * ratioCaps * 0.5f)) * 2u };
    // Split between the two caps
    const Index numberOfPointsCap{ numberOfPointsBothCaps / 2u };
    // Put the rest of the points in the body
    const Index numberOfPointsBody{ numberOfPoints - numberOfPointsBothCaps };

    // Create random distribution generators.
    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::uniform_real_distribution<ScalarType> uniformDistribution(-1.0f, 1.0f);

    addEmpty3dPointFields(numberOfPointsBody, pointCloud);

    // Sampling in Cylindrical coordinates.
    // This loop builds the Cylinder body.
    auto normalsView{ pointCloud.getDescriptorViewByName("normals") };
    for (Index i{ 0 }; i < numberOfPointsBody; ++i)
    {
        // Sample random values of theta and phi.
        const ScalarType phi{ 2.0f * pi * uniformDistribution(randomNumberGenerator) };
        const ScalarType z{ height * 0.5f * uniformDistribution(randomNumberGenerator) };

        // Pre-compute values, such as sine and cosine of phi and theta.
        const ScalarType sinPhi{ std::sin(phi) };
        const ScalarType cosPhi{ std::cos(phi) };

        // Fill features (3D point coordinates)
        pointCloud.features(0, i) = cosPhi * radius; // x
        pointCloud.features(1, i) = sinPhi * radius; // y
        pointCloud.features(2, i) = z; // z

        // For a cylinder, it's possible to compute outward looking normals by simply normalizing the vector going from its center to the
        // cylinder's body.
        const StaticCoordVector radiusVector(pointCloud.features(0, i), pointCloud.features(1, i), 0);
        const StaticCoordVector normalVector{ radiusVector.normalized() };

        // Fill normals.
        normalsView(0, i) = normalVector(0);
        normalsView(1, i) = normalVector(1);
        normalsView(2, i) = normalVector(2);
    }

    // Add top and bottom caps.
    const Quaternion topCapOrientation{ Quaternion::Identity() };
    const Quaternion bottomCapOrientation{ 0, 1, 0, 0 }; // Flip 180 degrees around x.
    const StaticCoordVector topCapTranslation{ 0.0f, 0.0f, height * 0.5f };
    const StaticCoordVector bottomCapTranslation{ 0.0f, 0.0f, -height * 0.5f };
    pointCloud.concatenate(generateUniformlySampledCircle(radius, numberOfPointsCap, topCapTranslation, topCapOrientation));
    pointCloud.concatenate(generateUniformlySampledCircle(radius, numberOfPointsCap, bottomCapTranslation, bottomCapOrientation));

    // Transform point cloud in space.
    applyTransformation(translation, rotation, pointCloud);

    return pointCloud;
}

// Reference: https://stackoverflow.com/questions/11815792/generation-of-3d-random-points-on-the-surface-of-a-cube
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::PointCloudGenerator::generateUniformlySampledPlane(
    const StaticCoordVector& dimensions,
    const Index numberOfPoints,
    const StaticCoordVector& translation,
    const Quaternion& rotation)
{
    // Create point cloud and add basic structure to fill.
    DataPoints pointCloud;
    addEmpty3dPointFields(numberOfPoints, pointCloud);

    // Create random distribution generators.
    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::uniform_real_distribution<ScalarType> lengthUniformDistribution(-dimensions(0), dimensions(0));
    std::uniform_real_distribution<ScalarType> widthUniformDistribution(-dimensions(1), dimensions(1));
    std::uniform_real_distribution<ScalarType> heightUniformDistribution(-dimensions(2), dimensions(2));

    const StaticCoordVector normalVector{ computeNormalOfAxisAlignedPlane(dimensions) };

    // Sampling in Cartesian coordinates.
    auto normalsView{ pointCloud.getDescriptorViewByName("normals") };
    for (Index i{ 0 }; i < numberOfPoints; ++i)
    {
        // Fill features (3D point coordinates)
        pointCloud.features(0, i) = lengthUniformDistribution(randomNumberGenerator); // x
        pointCloud.features(1, i) = widthUniformDistribution(randomNumberGenerator); // y
        pointCloud.features(2, i) = heightUniformDistribution(randomNumberGenerator); // z

        // Fill normals.
        normalsView(0, i) = normalVector(0);
        normalsView(1, i) = normalVector(1);
        normalsView(2, i) = normalVector(2);
    }

    // We generated the plane on the ground plane, and now we rotate it based on the translation and rotation arguments of this function.
    applyTransformation(translation, rotation, pointCloud);

    return pointCloud;
}

template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::PointCloudGenerator::generateUniformlySampledBox(
    const ScalarType length, const ScalarType width, const ScalarType height, const Index numberOfPoints,
    const StaticCoordVector& translation, const Quaternion& rotation)
{
    const Index numberOfFaces{ 6 };
    const Index numberOfPointsPerFace{ numberOfPoints / numberOfFaces };

    // Create point cloud and add basic structure to fill.
    DataPoints pointCloud;
    addEmpty3dPointFields(0, pointCloud);

    // Unit vectors for vertices.
    // TODO(ynava) Evaluate generating a unit-length box and then re-scaling it.
    const StaticCoordVector positiveXaxisFaceCenter{ length * 0.5f, 0.0f, 0.0f };
    const StaticCoordVector negativeXaxisFaceCenter{ -length * 0.5f, 0.0f, 0.0f };
    const StaticCoordVector positiveYaxisFaceCenter{ 0.0f, width * 0.5f, 0.0f };
    const StaticCoordVector negativeYaxisFaceCenter{ 0.0f, -width * 0.5f, 0.0f };
    const StaticCoordVector positiveZaxisFaceCenter{ 0.0f, 0.0f, height * 0.5f };
    const StaticCoordVector negativeZaxisFaceCenter{ 0.0f, 0.0f, -height * 0.5f };

    // Dimension vectors of each of the box faces.
    const Quaternion faceOrientation{ Quaternion::Identity() };
    const StaticCoordVector xyFaceDimensions{ length * 0.5f, width * 0.5f, 0.0f };
    const StaticCoordVector yzFaceDimensions{ 0.0f, width * 0.5f, height * 0.5f };
    const StaticCoordVector xzFaceDimensions{ length * 0.5f, 0.0f, height * 0.5f };

    // Add points from each face to the point cloud.
    pointCloud.concatenate(
        generateUniformlySampledPlane(yzFaceDimensions, numberOfPointsPerFace, positiveXaxisFaceCenter, faceOrientation));
    pointCloud.concatenate(
        generateUniformlySampledPlane(yzFaceDimensions, numberOfPointsPerFace, negativeXaxisFaceCenter, faceOrientation));
    pointCloud.concatenate(
        generateUniformlySampledPlane(xzFaceDimensions, numberOfPointsPerFace, positiveYaxisFaceCenter, faceOrientation));
    pointCloud.concatenate(
        generateUniformlySampledPlane(xzFaceDimensions, numberOfPointsPerFace, negativeYaxisFaceCenter, faceOrientation));
    pointCloud.concatenate(
        generateUniformlySampledPlane(xyFaceDimensions, numberOfPointsPerFace, positiveZaxisFaceCenter, faceOrientation));
    const Index missingPointsLastFace{ numberOfPoints - pointCloud.getNbPoints() - numberOfPointsPerFace };
    pointCloud.concatenate(generateUniformlySampledPlane(
        xyFaceDimensions, numberOfPointsPerFace + missingPointsLastFace, negativeZaxisFaceCenter, faceOrientation));

    // Transform point cloud in space.
    applyTransformation(translation, rotation, pointCloud);

    return pointCloud;
}

template struct PointMatcher<float>::PointCloudGenerator;
template struct PointMatcher<double>::PointCloudGenerator;
