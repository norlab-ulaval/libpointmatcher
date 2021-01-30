#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;


static inline Eigen::Transform<NumericType, 2, Eigen::Affine> buildUpTransformation2D(const Eigen::Matrix<NumericType, 2, 1>& translation,
                                                                                      const Eigen::Rotation2D<NumericType>& rotation,
                                                                                      const NumericType scale = 1.0)
{
    const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation =
        Eigen::Translation<NumericType, 2>(translation) * rotation * Eigen::Scaling(scale);
    return transformation;
}

static inline Eigen::Transform<NumericType, 3, Eigen::Affine> buildUpTransformation3D(const Eigen::Matrix<NumericType, 3, 1>& translation,
                                                                                      const Eigen::Quaternion<NumericType>& rotation,
                                                                                      const NumericType scale = 1.0)
{
    const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation =
        Eigen::Translation<NumericType, 3>(translation) * Eigen::AngleAxis<NumericType>(rotation.normalized()) * Eigen::Scaling(scale);
    return transformation;
}

static inline void assertOnDataPointsTransformation(const PM::DataPoints& cloud, const PM::TransformationParameters& transformation,
                                                    std::shared_ptr<PM::Transformation>& transformator,
                                                    const NumericType kEpsilonNumericalError = 0)
{
    // Transform point cloud.
    auto transformedCloud = cloud;
    transformator->inPlaceCompute(transformation, transformedCloud);
    // Assert on the result.
    // Transformation quality. 'Who tests the unit test?'
    EXPECT_TRUE(transformator->checkParameters(transformation));
    // Features.
    for (size_t i = 0; i < cloud.getNbPoints(); ++i)
    {
        const auto transformedFeature = transformation * cloud.features.col(i);
        ASSERT_TRUE(transformedFeature.isApprox(transformedCloud.features.col(i), kEpsilonNumericalError));
    }

    // Descriptors.
    int row(0);
    const int descCols(cloud.descriptors.cols());
    const unsigned int nbRows = transformation.rows() - 1;
    const unsigned int nbCols = transformation.cols() - 1;
    const PM::TransformationParameters R(transformation.topLeftCorner(nbRows, nbCols));
    for (size_t i = 0; i < cloud.descriptorLabels.size(); ++i)
    {
        const int span(cloud.descriptorLabels[i].span);
        const std::string& name(cloud.descriptorLabels[i].text);
        if (name == "normals" || name == "observationDirections")
        {
            const auto transformedDescriptor = R * cloud.descriptors.block(row, 0, span, descCols);
            ASSERT_TRUE(transformedDescriptor.isApprox(transformedCloud.descriptors.block(row, 0, span, descCols), kEpsilonNumericalError));
        }
        row += span;
    }
    EXPECT_EQ(cloud.featureLabels, transformedCloud.featureLabels);
    EXPECT_EQ(cloud.descriptorLabels, transformedCloud.descriptorLabels);
    EXPECT_EQ(cloud.timeLabels, transformedCloud.timeLabels);
    EXPECT_EQ(cloud.times, transformedCloud.times);
}

//---------------------------
// Transformation Checker modules
//---------------------------

// Utility classes
class TransformationCheckerTest : public IcpHelper
{
public:
    std::shared_ptr<PM::TransformationChecker> transformCheck;

    // Will be called for every tests
    virtual void SetUp()
    {
        icp.setDefault();
        // Uncomment for consol outputs
        //setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

        icp.transformationCheckers.clear();
    }

    // Will be called for every tests
    virtual void TearDown() {}

    void addFilter(string name, PM::Parameters params)
    {
        transformCheck = PM::get().TransformationCheckerRegistrar.create(name, params);

        icp.transformationCheckers.push_back(transformCheck);
    }
};


TEST_F(TransformationCheckerTest, CounterTransformationChecker)
{
    addFilter("CounterTransformationChecker", { { "maxIterationCount", toParam(20) } });
    validate2dTransformation();
}

TEST_F(TransformationCheckerTest, DifferentialTransformationChecker)
{
    addFilter("DifferentialTransformationChecker",
              { { "minDiffRotErr", toParam(0.001) }, { "minDiffTransErr", toParam(0.001) }, { "smoothLength", toParam(4) } });
    validate2dTransformation();
}

TEST_F(TransformationCheckerTest, BoundTransformationChecker)
{
    // Since that transChecker is trigger when the distance is growing
    // and that we do not expect that to happen in the test dataset, we
    // keep the Counter to get out of the looop
    std::shared_ptr<PM::TransformationChecker> extraTransformCheck;

    extraTransformCheck = PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker");
    icp.transformationCheckers.push_back(extraTransformCheck);

    addFilter("BoundTransformationChecker", { { "maxRotationNorm", toParam(1.0) }, { "maxTranslationNorm", toParam(1.0) } });
    validate2dTransformation();
}

//---------------------------
// Transformation
//---------------------------
TEST(Transformation, RigidTransformationParameterCheck)
{
    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    //-------------------------------------
    // Construct a 3D non-orthogonal matrix
    PM::Matrix T_3D = PM::Matrix::Identity(4, 4);
    //T_3D(0,0) = 2.3;
    //T_3D(0,1) = 0.03;
    T_3D << 0.99935116, 0.13669771, 0.03436585, 1.71138524, -0.02633967, 0.99326295, -0.04907545, -0.10860933, -0.03615132, 0.04400287,
        0.99820427, -0.04454497, 0., 0., 0., 1.;

    EXPECT_FALSE(rigidTrans->checkParameters(T_3D));

    EXPECT_THROW(rigidTrans->compute(data3D, T_3D), TransformationError);

    // Check stability over iterations
    for (int i = 0; i < 10; i++)
    {
        T_3D = rigidTrans->correctParameters(T_3D);
        ASSERT_TRUE(rigidTrans->checkParameters(T_3D));
    }

    //-------------------------------------
    // Construct a 2D non-orthogonal matrix
    PM::Matrix T_2D_non_ortho = PM::Matrix::Identity(3, 3);
    T_2D_non_ortho(0, 0) = 0.8;
    T_2D_non_ortho(0, 1) = -0.5;
    T_2D_non_ortho(1, 0) = 0.5;
    T_2D_non_ortho(1, 1) = 0.8;

    EXPECT_FALSE(rigidTrans->checkParameters(T_2D_non_ortho));

    EXPECT_THROW(rigidTrans->compute(data2D, T_2D_non_ortho), TransformationError);
    EXPECT_THROW(rigidTrans->inPlaceCompute(T_2D_non_ortho, data2D), TransformationError);

    // Check stability over iterations
    for (int i = 0; i < 10; i++)
    {
        T_2D_non_ortho = rigidTrans->correctParameters(T_2D_non_ortho);
        EXPECT_TRUE(rigidTrans->checkParameters(T_2D_non_ortho));
    }

    //-------------------------------------
    // Construct a 2D reflection matrix
    PM::Matrix T_2D_reflection = PM::Matrix::Identity(3, 3);
    T_2D_reflection(1, 1) = -1;

    EXPECT_THROW(rigidTrans->correctParameters(T_2D_reflection), TransformationError);
}

TEST(Transformation, ComputePureTranslationDataPoints2D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("PureTranslation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure translation.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ -1.0001, 34.5 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
}

TEST(Transformation, ComputePureTranslationDataPoints3D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("PureTranslation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }

    // Pure translation.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ -1.0001, 5, -12321.234 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
}

TEST(Transformation, ComputeRigidTransformDataPoints2D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("RigidTransformation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure translation.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ -1.0001, 34.5 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure rotation.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 3.53453 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Translation + rotation.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ -3.11, 100.222 };
        const Eigen::Rotation2D<NumericType> rotation{ -123.3 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
}

TEST(Transformation, ComputeRigidTransformDataPoints3D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("RigidTransformation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }

    // Pure translation.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ -1.0001, 5, -12321.234 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure rotation.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, -5, 23, 0.5 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Translation + rotation.
    {
        const NumericType kEpsilonNumericalError = 1e-6;
        const Eigen::Matrix<NumericType, 3, 1> translation{ 1, -3, -4 };
        const Eigen::Quaternion<NumericType> rotation{ 0, -2.54, 0, 0.5 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator, kEpsilonNumericalError);
    }
}

TEST(Transformation, ComputeSimilarityTransformDataPoints2D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("SimilarityTransformation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const NumericType scale{ 1.0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure Upscaling.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const NumericType scale{ 5.0 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure Downscaling.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const NumericType scale{ 0.1 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure translation + Downscaling.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ -1.0001, 34.5 };
        const Eigen::Rotation2D<NumericType> rotation{ 0 };
        const NumericType scale{ 0.5 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure rotation + Upscaling.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ 0, 0 };
        const Eigen::Rotation2D<NumericType> rotation{ 3.53453 };
        const NumericType scale{ 1.9 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Translation + rotation + Upscaling.
    {
        const Eigen::Matrix<NumericType, 2, 1> translation{ -3.11, 100.222 };
        const Eigen::Rotation2D<NumericType> rotation{ -123.3 };
        const NumericType scale{ 1.9 };
        const Eigen::Transform<NumericType, 2, Eigen::Affine> transformation = buildUpTransformation2D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
}

TEST(Transformation, ComputeSimilarityTransformDataPoints3D)
{
    std::shared_ptr<PM::Transformation> transformator = PM::get().REG(Transformation).create("SimilarityTransformation");

    // Identity.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const NumericType scale{ 1.0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure Upscaling.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const NumericType scale{ 5.0 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure Downscaling.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const NumericType scale{ 0.1 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure translation + Downscaling.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ -1.0001, 5, -12321.234 };
        const Eigen::Quaternion<NumericType> rotation{ 1, 0, 0, 0 };
        const NumericType scale{ 0.5 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Pure rotation + Upscaling.
    {
        const Eigen::Matrix<NumericType, 3, 1> translation{ 0, 0, 0 };
        const Eigen::Quaternion<NumericType> rotation{ 1, -5, 23, 0.5 };
        const NumericType scale{ 1.9 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator);
    }
    // Translation + rotation + Upscaling.
    {
        const NumericType kEpsilonNumericalError = 1e-6;
        const Eigen::Matrix<NumericType, 3, 1> translation{ 1, -3, -4 };
        const Eigen::Quaternion<NumericType> rotation{ 0, -2.54, 0, 0.5 };
        const NumericType scale{ 1.9 };
        const Eigen::Transform<NumericType, 3, Eigen::Affine> transformation = buildUpTransformation3D(translation, rotation, scale);
        // Transform and assert on the result.
        assertOnDataPointsTransformation(data3D, transformation.matrix(), transformator, kEpsilonNumericalError);
    }
}