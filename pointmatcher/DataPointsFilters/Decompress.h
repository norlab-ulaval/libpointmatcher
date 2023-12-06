//
// Created by Matěj Boxan on 2023-12-04.
//

#include "PointMatcher.h"
#include "utils/distribution.h"


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
			};
	}

public:
    struct PointsGenerator
    {
		virtual ~PointsGenerator(){}
        void processCloud(DataPoints& cloud)
        {
            auto points = cloud.features;
            auto omegas = cloud.getDescriptorViewByName("omega");
            auto deviations = cloud.getDescriptorViewByName("deviation");

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
                Eigen::Map<Eigen::Matrix<T, 3, 3>> deviationMatrix(deviations.block(0, i, 9, 1).data(), 3, 3);
                Distribution<T> distribution(points.col(i).head(3),
                                               omega,
                                               deviationMatrix,
                                               cloud.times.col(i),
                                               cloud.descriptors.col(i));
                Matrix newPoints = processPoint(distribution);
                features.block(0, idx, 3, omega) = newPoints;
                descriptors.block(0, idx, descriptorsCount, omega) = cloud.descriptors.col(i);
                times.block(0, idx, timesCount, omega) = cloud.times.col(i);
                idx += omega;
            }

            newCloud.descriptors = descriptors;
            newCloud.descriptorLabels = cloud.descriptorLabels;
            newCloud.times = times;
            newCloud.timeLabels = cloud.timeLabels;
            cloud = newCloud;
        }

        virtual Matrix processPoint(const Distribution<T>& distribution) = 0;
    };

    struct GaussianGenerator : public PointsGenerator {
		GaussianGenerator() {}
		virtual ~GaussianGenerator(){}

        Matrix processPoint(const Distribution<T>& distribution) override {
            unsigned numberOfPoints = distribution.omega;
            Matrix points = Matrix::Zero(3, numberOfPoints);
            return points;
        }
    };

    struct UniformGenerator : public PointsGenerator {
		UniformGenerator() {}
		virtual ~UniformGenerator(){}

        Matrix processPoint(const Distribution<T>& distribution) override {
            unsigned numberOfPoints = distribution.omega;
            Matrix points = Matrix::Zero(3, numberOfPoints);
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
