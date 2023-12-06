//
// Created by Matěj Boxan on 2023-12-04.
//

#include "PointMatcher.h"
#include "utils/distribution.h"
#include <random>


template<typename T>
struct DecompressDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    typedef PointMatcher<T> PM;
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::Matrix Matrix;
	typedef typename PointMatcher<T>::Int64Matrix Int64Matrix;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
        return "TODO";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"pointsGenerator", "Type of the probability distribution used to generate points. Gaussian (0), Uniform (1)", "0", "0", "1", &P::Comp<unsigned>},
			{"seed", "Seed for random sampling (-1 means no seed is used)", "-1", "-1", "2147483647", &P::Comp<int>}
            };
	}

private:
    const std::size_t seed;

public:
    struct PointsGenerator
    {
        PointsGenerator(int seed): seed{seed} {
            if (seed == -1)
            {
                randomNumberGenerator = std::minstd_rand(randomDevice());
            }
            else
            {
                randomNumberGenerator = std::minstd_rand(seed);
            }
        }
		virtual ~PointsGenerator(){}
        void processCloud(DataPoints& cloud)
        {
            auto points = cloud.features;
            auto omegas = cloud.getDescriptorCopyByName("omega");
            auto deviations = cloud.getDescriptorCopyByName("deviation");

            unsigned nbPointsToGenerate = omegas.sum();

            DataPoints newCloud;
            newCloud.conservativeResize(nbPointsToGenerate);
            Matrix features = Matrix::Zero(4, nbPointsToGenerate);
            features.row(3).setConstant(1);
            Matrix descriptors = Matrix::Zero(cloud.descriptors.rows() - omegas.rows() - deviations.rows(), nbPointsToGenerate);
            Int64Matrix times = Int64Matrix::Zero(cloud.times.rows(), nbPointsToGenerate);

            cloud.removeDescriptor("omega");
            cloud.removeDescriptor("deviation");

            unsigned descriptorsCount = cloud.descriptors.rows();
            unsigned timesCount = cloud.times.rows();

            unsigned idx = 0;
            for(unsigned i = 0; i < cloud.getNbPoints(); ++i)
            {
                unsigned omega = omegas(0, i);
                assert(omega >= 1);
                Matrix newPoints;
                if (omega > 1)
                {
                    Eigen::Map<Eigen::Matrix<T, 3, 3>> deviationMatrix(deviations.block(0, i, 9, 1).data(), 3, 3);
                    Distribution<T> distribution(points.col(i).head(3),
                                                 omega,
                                                 deviationMatrix,
                                                 cloud.times.col(i),
                                                 cloud.descriptors.col(i));
                    newPoints = processPoint(distribution);
                }
                else
                {
                    newPoints = points.col(i).head(3);
                }
                unsigned nbNewPoints = newPoints.cols();
                features.block(0, idx, 3, nbNewPoints) = newPoints;
                descriptors.block(0, idx, descriptorsCount, nbNewPoints).colwise() = cloud.descriptors.col(i);

                times.block(0, idx, timesCount, nbNewPoints).colwise() = cloud.times.col(i);
                idx += nbNewPoints;
            }

            newCloud.features = features;
            newCloud.featureLabels = cloud.featureLabels;
            newCloud.descriptors = descriptors;
            newCloud.descriptorLabels = cloud.descriptorLabels;
            newCloud.times = times;
            newCloud.timeLabels = cloud.timeLabels;
            cloud = newCloud;
        }

        virtual Matrix processPoint(const Distribution<T>& distribution) = 0;

    protected:
        const int seed;
    public:
        std::minstd_rand randomNumberGenerator;
        std::random_device randomDevice;
    };

    struct GaussianGenerator : public PointsGenerator {
		GaussianGenerator(int seed): PointsGenerator(seed) {

        }
		virtual ~GaussianGenerator(){}

        Matrix processPoint(const Distribution<T>& distribution) override {
            using Matrix33 = Eigen::Matrix<T, 3, 3>;

            unsigned nbPoints = distribution.omega;
            std::normal_distribution<float> normalRealDistribution(0, 1);
            Matrix points = Matrix::NullaryExpr(3, nbPoints, [&](float) {
                return normalRealDistribution(PointsGenerator::randomNumberGenerator);
            });
            Matrix33 covariance = distribution.deviation / distribution.omega;
            const Eigen::SelfAdjointEigenSolver<Matrix33> solver(covariance);
            auto eigenVa = solver.eigenvalues().real();
            auto eigenVec = solver.eigenvectors().real();

            Matrix scales = eigenVa.array().sqrt();
            Matrix33 S = scales.asDiagonal();
            points = S * eigenVec * points;
            points.colwise() += distribution.point; // TODO replace this by transformation matrix
            return points;
        }
    };

    struct UniformGenerator : public PointsGenerator {
		UniformGenerator(int seed): PointsGenerator(seed) {

        }
		virtual ~UniformGenerator(){}

        Matrix processPoint(const Distribution<T>& distribution) override {
            using Matrix33 = Eigen::Matrix<T, 3, 3>;

            unsigned nbPoints = distribution.omega;
            std::uniform_real_distribution<float> uniformRealDistribution(-1, 1);
            Matrix points = Matrix::NullaryExpr(3, nbPoints, [&](float) {
                return uniformRealDistribution(PointsGenerator::randomNumberGenerator);
            });
            Matrix33 covariance = distribution.deviation / distribution.omega;
            const Eigen::SelfAdjointEigenSolver<Matrix33> solver(covariance);
            auto eigenVa = solver.eigenvalues().real();
            auto eigenVec = solver.eigenvectors().real();

            Matrix scales = eigenVa.array().sqrt() * 1.73205080757; // TODO better represent the sigma value
            Matrix33 S = scales.asDiagonal();
            points = S * eigenVec * points;
            points.colwise() += distribution.point; // TODO replace this by transformation matrix
            return points;
        }
    };

    enum PointsGeneratorType : unsigned {GAUSSIAN=0, UNIFORM=1};

	//! Constructor, uses parameter interface
	explicit DecompressDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
private:
	PointsGeneratorType pointsGeneratorType;
};
