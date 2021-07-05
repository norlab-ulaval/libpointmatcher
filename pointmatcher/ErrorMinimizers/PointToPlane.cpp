// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
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

#include <iostream>

#include "Eigen/SVD"
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

using namespace Eigen;
using namespace std;

typedef PointMatcherSupport::Parametrizable Parametrizable;
typedef PointMatcherSupport::Parametrizable P;
typedef Parametrizable::Parameters Parameters;
typedef Parametrizable::ParameterDoc ParameterDoc;
typedef Parametrizable::ParametersDoc ParametersDoc;

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const Parameters& params):
	ErrorMinimizer(name(), availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D")),
	force4DOF(Parametrizable::get<T>("force4DOF")),
	forceXYZOnly(Parametrizable::get<T>("forceXYZOnly")),
    LiePenalty(Parametrizable::get<T>("LiePenalty")),
    priorCovariance(Parametrizable::get<T>("priorCovariance"))
{

	if((force2D?1:0)+(force4DOF?1:0)+(forceXYZOnly?1:0) >= 2)
	{
		throw PointMatcherSupport::ConfigurationError("Force2D, force4DOF and forceXYZOnly are mutually exclusive.");
	}

	if(force2D)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
	}
	else if(force4DOF)
    {
        LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
    }
	else if(forceXYZOnly)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will consider translation only(x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in full 6DOF.");
	}
}

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params):
	ErrorMinimizer(name(), paramsDoc, params),
	force2D(Parametrizable::get<T>("force2D")),
	force4DOF(Parametrizable::get<T>("force4DOF")),
	forceXYZOnly(Parametrizable::get<T>("forceXYZOnly")),
    LiePenalty(Parametrizable::get<T>("LiePenalty")),
    priorCovariance(Parametrizable::get<T>("priorCovariance"))
{
	if((force2D?1:0)+(force4DOF?1:0)+(forceXYZOnly?1:0) >= 2)
	{
		throw PointMatcherSupport::ConfigurationError("Force2D, force4DOF and forceXYZOnly are mutually exclusive.");
	}

	if(force2D)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
	}
	else if(force4DOF)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
	}
	else if(forceXYZOnly)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will consider translation only(x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in full 6DOF.");
	}
}


template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x) {
	assert(A.cols() == A.rows());
	assert(b.cols() == 1);
	assert(b.rows() == A.rows());
	assert(x.cols() == 1);
	assert(x.rows() == A.cols());

	typedef typename PointMatcher<T>::Matrix Matrix;

	BOOST_AUTO(Aqr, A.fullPivHouseholderQr());
	if (!Aqr.isInvertible())
	{
		// Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1 Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore A = QR ~= Q1 * R1
		const int rank = Aqr.rank();
		const int rows = A.rows();
		const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
		const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

		const bool findMinimalNormSolution = true; // TODO is that what we want?

		// The under-determined system R1 x = Q1^T b is made unique ..
		if(findMinimalNormSolution){
			// by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1 Q1^T b.
			x = R1.template triangularView<Eigen::Upper>().transpose() * (R1 * R1.transpose()).llt().solve(Q1t * b);
		} else {
			// by solving the simplest problem that yields fewest nonzero components in x
			x.block(0, 0, rank, 1) = R1.block(0, 0, rank, rank).template triangularView<Eigen::Upper>().solve(Q1t * b);
			x.block(rank, 0, rows - rank, 1).setZero();
		}

		x = Aqr.colsPermutation() * x;

		BOOST_AUTO(ax , (A * x).eval());
		if (!b.isApprox(ax, 1e-5)) {
			LOG_INFO_STREAM("PointMatcher::icp - encountered almost singular matrix while minimizing point to plane distance. QR solution was too inaccurate. Trying more accurate approach using double precision SVD.");
			x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
			ax = A * x;

			if((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())){
				LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane distance and the current workaround remained inaccurate."
						<< " b=" << b.transpose()
						<< " !~ A * x=" << (ax).transpose().eval()
						<< ": ||b- ax||=" << (b - ax).norm()
						<< ", ||b||=" << b.norm()
						<< ", ||ax||=" << ax.norm());
			}
		}
	}
	else {
		// Cholesky decomposition
		x = A.llt().solve(b);
	}
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
    ErrorElements mPts = mPts_const;

    //As for the Point to Gaussian, i rename the mPts ErrorElements to add new point.
    //With the LiePenalty, that change is applied
    //Without it, the normal point to plane is done
    if(LiePenalty) {
        const size_t dim(mPts_const.reference.features.rows() - 1);
        const size_t newSize((mPts.weights.cols()) + dim);
        mPts.weights.conservativeResize(Eigen::NoChange, newSize + dim);

        if (!mPts.reference.descriptorExists("normals")) {
            Labels cloudLabels;
            cloudLabels.push_back(Label("normals", dim));
            mPts.reference.allocateDescriptors(cloudLabels); // Reserve memory
        }
        View normals = mPts.reference.getDescriptorViewByName("normals");

        for (long i = 0; i < mPts_const.reference.features.cols(); ++i) {
            mPts.weights.block(0, i, 1, dim) = mPts_const.weights.block(0, i, 1, dim);
        }

        //The Lie Penalty is computed according to the prior rotation matrix and the ICP correction matrix found during the ICP loop
        //This penalty is changing at each iteration of ICP. That's why it's computed here before the minimization
        //The theory is presented in WorkReport_LieICP

        //Identity matrix use later
        Matrix identityMatrix;
        identityMatrix.resize(3, 3);
        identityMatrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Matrix3f identityMatrixf;
        identityMatrixf << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        //Noise covariance of the prior given by the yaml file
        //NoiseSensor is the equivalent to the noise covariance Sigma_e presented in equation (3), for the noise epsilon presented in WorkReport_LieICP
        Matrix NoiseSensor = priorCovariance * identityMatrix;

        //Rotation matrix of prior and ICP transform
        //rotation_matrix_prior is equivalent to C_s presented in WorkReport_LieICP
        //rotation_matrix_icp is equivalent to C_I presented in WorkReport_LieICP
        //TODO: maxime, simplify the assignment between matrix
        Matrix3f rotation_matrix_prior;       //Rotation matrix of the prior
        Matrix3f rotation_matrix_icp;         //Rotation matrix of ICP
        rotation_matrix_prior(0, 0) = mPts.T_prior_save(0, 0);
        rotation_matrix_prior(0, 1) = mPts.T_prior_save(0, 1);
        rotation_matrix_prior(0, 2) = mPts.T_prior_save(0, 2);
        rotation_matrix_prior(1, 0) = mPts.T_prior_save(1, 0);
        rotation_matrix_prior(1, 1) = mPts.T_prior_save(1, 1);
        rotation_matrix_prior(1, 2) = mPts.T_prior_save(1, 2);
        rotation_matrix_prior(2, 0) = mPts.T_prior_save(2, 0);
        rotation_matrix_prior(2, 1) = mPts.T_prior_save(2, 1);
        rotation_matrix_prior(2, 2) = mPts.T_prior_save(2, 2);

        rotation_matrix_icp(0, 0) = mPts.T_refMean_iter(0, 0);
        rotation_matrix_icp(0, 1) = mPts.T_refMean_iter(0, 1);
        rotation_matrix_icp(0, 2) = mPts.T_refMean_iter(0, 2);
        rotation_matrix_icp(1, 0) = mPts.T_refMean_iter(1, 0);
        rotation_matrix_icp(1, 1) = mPts.T_refMean_iter(1, 1);
        rotation_matrix_icp(1, 2) = mPts.T_refMean_iter(1, 2);
        rotation_matrix_icp(2, 0) = mPts.T_refMean_iter(2, 0);
        rotation_matrix_icp(2, 1) = mPts.T_refMean_iter(2, 1);
        rotation_matrix_icp(2, 2) = mPts.T_refMean_iter(2, 2);

        //Extract angle and axis of rotation for icp and prior rotation matrix
        //Convert matrix to extract the parameters
        AngleAxisf priorAngleAxisStruct;
        AngleAxisf icpAngleAxisStruct;
        priorAngleAxisStruct = rotation_matrix_prior;
        icpAngleAxisStruct = rotation_matrix_icp;
        //Extract the parameters with Eigen library
        //priorAngleAxis is equivalent to the normalized vector phi presented in WorkReport_LieICP
        //icpAngleAxis is equivalent to the normalized vector theta presented in WorkReport_LieICP
        //priorAngle is equivalent to the value phi presented in WorkReport_LieICP
        //icpAngle is equivalent to the value theta presented in WorkReport_LieICP
        Vector3f priorAngleAxis = priorAngleAxisStruct.axis();  //Axis of prior rotation
        Vector3f icpAngleAxis = icpAngleAxisStruct.axis();      //Axis of ICP correction rotation
        float priorAngle = priorAngleAxisStruct.angle();        //Angle of prior rotation
        float icpAngle = icpAngleAxisStruct.angle();            //Angle of ICP correction rotation

        //Compute the result of Lie Algebra rotation
        //Matrix of axis prior rotation in Lie Algebra
        //TODO: maxime, better way to define the matrix
        Matrix3f crossProdutPriorAngleAxis;
        crossProdutPriorAngleAxis(0, 0) = 0;
        crossProdutPriorAngleAxis(0, 1) = -priorAngleAxis(2, 0);
        crossProdutPriorAngleAxis(0, 2) = priorAngleAxis(1, 0);
        crossProdutPriorAngleAxis(1, 0) = priorAngleAxis(2, 0);
        crossProdutPriorAngleAxis(1, 1) = 0;
        crossProdutPriorAngleAxis(1, 2) = -priorAngleAxis(0, 0);
        crossProdutPriorAngleAxis(2, 0) = -priorAngleAxis(1, 0);
        crossProdutPriorAngleAxis(2, 1) = priorAngleAxis(0, 0);
        crossProdutPriorAngleAxis(2, 2) = 0;

        //Jprior is the result of the equation J(beta) presented at the end of equation (4) in WorkReport_LieICP
        Matrix3f Jprior = sin(priorAngle) * identityMatrixf
                          + (1 - sin(priorAngle)) * priorAngleAxis * priorAngleAxis.transpose()
                          + (1 - cos(priorAngle)) / priorAngle * crossProdutPriorAngleAxis;

        //Compute the error
        //priorAngleVector is equivalent to the vector phi presented in WorkReport_LieICP
        //icpAngleVector is equivalent to the vector theta presented in WorkReport_LieICP
        Vector3f priorAngleVector = priorAngle * priorAngleAxis;
        Vector3f icpAngleVector = icpAngle * icpAngleAxis;

        //beta is calculated according to the equation (9) in WorkReport_LieICP
        //TODO: maxime, better way to calculate beta
        Vector beta(4, 1);
        beta << Jprior(0, 0) * (priorAngleVector(0, 0) - icpAngleVector(0, 0)) +
                Jprior(0, 1) * (priorAngleVector(1, 0) - icpAngleVector(1, 0)) +
                Jprior(0, 2) * (priorAngleVector(2, 0) - icpAngleVector(2, 0)),
                Jprior(1, 0) * (priorAngleVector(0, 0) - icpAngleVector(0, 0)) +
                Jprior(1, 1) * (priorAngleVector(1, 0) - icpAngleVector(1, 0)) +
                Jprior(1, 2) * (priorAngleVector(2, 0) - icpAngleVector(2, 0)),
                Jprior(2, 0) * (priorAngleVector(0, 0) - icpAngleVector(0, 0)) +
                Jprior(2, 1) * (priorAngleVector(1, 0) - icpAngleVector(1, 0)) +
                Jprior(2, 2) * (priorAngleVector(2, 0) - icpAngleVector(2, 0)), 1;
        Vector beta_null(4, 1);
        beta_null << 0,0,0,1;

        cout << "beta Lie penalty: " << endl << beta.transpose() << endl;

        // To minimize both the distances from the point cloud and the penalty at the same time we convert the penalty to fake pair of point/normal.
        // For the penalty, n fake pair of point/normal will be created, where n is the dimensions of the covariance.
        // The eigen decomposition of the penalty's covariance give us the following:
        // W: Covariance a n by n matrix
        // W = N * L * N^T
        // N = [n1 n2 n3]
        // where L is a diagonal matrix of the eigen value and N is a rotation matrix.
        // n1, n2, n3 are column vectors. The fake pair will use these vectors as normal.
        // For the fake points of the reference and the reading, the translation part of penalty tf matrix and the current transformation matrix will be used respectively.

        Matrix penaltiesPtsRead(dim + 1, dim);
        Matrix penaltiesPtsReference(dim + 1, dim);
        Matrix penaltiesNormals(dim, dim);

        //Solve W = N * L * N^T
        const EigenSolver <Matrix> solver(NoiseSensor);   //W = NoiseSensor
        const Matrix eigenVec = solver.eigenvectors().real();  //N
        const Vector eigenVal = solver.eigenvalues().real();   //Values for L

        //According to equation (7) in WorkReport_LieICP, the error should be beta.transpose()*NoiseSensor.inverse()*beta
        //Compute points to add

        penaltiesPtsRead.block(0, 0, dim + 1 , dim) = (mPts.T_prior_save*beta).replicate(1, dim);   //Send beta in T_prior frame
        penaltiesPtsReference.block(0, 0, dim + 1, dim) = (mPts.T_prior_save*beta).replicate(1, dim); //Send beta in T_prior frame
        penaltiesNormals.block(0, 0, dim, dim) = eigenVec;
        mPts.weights.block(0, newSize, 1, dim) = eigenVal.diagonal().array().inverse().transpose();

        const Labels normalLabel({Label("normals", dim)});
        const DataPoints penaltiesReference(penaltiesPtsReference, mPts_const.reference.featureLabels, penaltiesNormals,
                                            normalLabel);
        const DataPoints penaltiesRead(penaltiesPtsRead, mPts.reading.featureLabels);

        //Add penalty to the minimization datapoints
        mPts.reference.concatenate(penaltiesReference);
        mPts.reading.concatenate(penaltiesRead);
    }
    return compute_in_place(mPts);
}


template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute_in_place(ErrorElements& mPts)
{
    const int dim = mPts.reading.features.rows();
    const int nbPts = mPts.reading.features.cols();

    // Adjust if the user forces 2D minimization on XY-plane
    int forcedDim = dim - 1;
    if(force2D && dim == 4)
    {
        mPts.reading.features.conservativeResize(3, Eigen::NoChange);
        mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
        mPts.reference.features.conservativeResize(3, Eigen::NoChange);
        mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
        forcedDim = dim - 2;
    }

    // Fetch normal vectors of the reference point cloud (with adjustment if needed)
    const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

    // Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
    assert(normalRef.rows() > 0);

    Matrix cross;
    Matrix Gamma(3,3);

    if(force4DOF)
    {
        //VK: Instead for "cross" as in 3D, we need only a dot product with the Gamma factor for 4DOF
        Gamma << 0,-1, 0,
                1, 0, 0,
                0, 0, 0;
        cross = ((Gamma*mPts.reading.features).transpose()*normalRef).diagonal().transpose();
    }
    else if(!forceXYZOnly)
    {
        // Compute cross product of cross = cross(reading X normalRef)
        cross = this->crossProduct(mPts.reading.features, normalRef);
    }

    // else the forceXYZOnly applies and the cross and gamma are not needed anymore

    Matrix wF;
    Matrix F;

    if(forceXYZOnly)
    {
        // wF = [weights*normals]
        // F  = [normals]
        wF.resize(normalRef.rows(), normalRef.cols());
        F.resize(normalRef.rows(), normalRef.cols());

        for (int i = 0; i < normalRef.rows(); i++) {
            wF.row(i) = mPts.weights.array() * normalRef.row(i).array();
            F.row(i) = normalRef.row(i);
        }

    }
    else {
        // wF = [weights*cross, weights*normals]
        // F  = [cross, normals]
        wF.resize(normalRef.rows() + cross.rows(), normalRef.cols());
        F.resize(normalRef.rows() + cross.rows(), normalRef.cols());

        for (int i = 0; i < cross.rows(); i++) {
            wF.row(i) = mPts.weights.array() * cross.row(i).array();
            F.row(i) = cross.row(i);
        }
        for (int i = 0; i < normalRef.rows(); i++) {
            wF.row(i + cross.rows()) = mPts.weights.array() * normalRef.row(i).array();
            F.row(i + cross.rows()) = normalRef.row(i);
        }

    }

    // Unadjust covariance A = wF * F'
    const Matrix A = wF * F.transpose();

    const Matrix deltas = mPts.reading.features - mPts.reference.features;

    // dot product of dot = dot(deltas, normals)
    Matrix dotProd = Matrix::Zero(1, normalRef.cols());

    for(int i=0; i<normalRef.rows(); i++)
    {
        dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
    }

    // b = -(wF' * dot)
    const Vector b = -(wF * dotProd.transpose());

    //TODO: VK: Remove this debug printing
    /*std::cout << "This is A" << std::endl << A << std::endl;
    std::cout << "This is b:" << std::endl << b << std::endl;*/


    Vector x(A.rows());

    solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x);

    // Transform parameters to matrix
    Matrix mOut;
    if(dim == 4 && !force2D)
    {
        Eigen::Transform<T, 3, Eigen::Affine> transform;
        // PLEASE DONT USE EULAR ANGLES!!!!
        // Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
        /*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
         * Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
         * Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());*/

        if(force4DOF)
        {
            Vector unitZ(3,1);
            unitZ << 0,0,1;
            transform = Eigen::AngleAxis<T>(x(0), unitZ);   //x=[gamma,x,y,z]
        }
        else if(forceXYZOnly)
        {
            Vector unitZ(3,1);
            unitZ << 0,0,1;
            transform = Eigen::AngleAxis<T>(T(0.0), unitZ);   //x=[x,y,z]
        }
        else
        {
            transform = Eigen::AngleAxis<T>(x.head(3).norm(), x.head(3).normalized()); //x=[alpha,beta,gamma,x,y,z]
        }

        // Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
        /*const T pitch = -asin(transform(2,0));
            const T roll = atan2(transform(2,1), transform(2,2));
            const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
            std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;*/
        if (force4DOF)
        {
            transform.translation() = x.segment(1, 3);  //x=[gamma,x,y,z]
        }
        else if(forceXYZOnly)
        {
            transform.translation() = x;  //x=[x,y,z]
        }
        else
        {
            transform.translation() = x.segment(3, 3);  //x=[alpha,beta,gamma,x,y,z]
        }

        mOut = transform.matrix();

        if (mOut != mOut)
        {
            // Degenerate situation. This can happen when the source and reading clouds
            // are identical, and then b and x above are 0, and the rotation matrix cannot
            // be determined, it comes out full of NaNs. The correct rotation is the identity.
            mOut.block(0, 0, dim-1, dim-1) = Matrix::Identity(dim-1, dim-1);
        }
    }
    else
    {
        Eigen::Transform<T, 2, Eigen::Affine> transform;
        transform = Eigen::Rotation2D<T> (x(0));
        transform.translation() = x.segment(1, 2);

        if(force2D)
        {
            mOut = Matrix::Identity(dim, dim);
            mOut.topLeftCorner(2, 2) = transform.matrix().topLeftCorner(2, 2);
            mOut.topRightCorner(2, 1) = transform.matrix().topRightCorner(2, 1);
        }
        else
        {
            mOut = transform.matrix();
        }
    }
    return mOut;
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::computeResidualError(ErrorElements mPts, const bool& force2D)
{
	const int dim = mPts.reading.features.rows();
	const int nbPts = mPts.reading.features.cols();

	// Adjust if the user forces 2D minimization on XY-plane
	int forcedDim = dim - 1;
	if(force2D && dim == 4)
	{
		mPts.reading.features.conservativeResize(3, Eigen::NoChange);
		mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
		mPts.reference.features.conservativeResize(3, Eigen::NoChange);
		mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
		forcedDim = dim - 2;
	}

	// Fetch normal vectors of the reference point cloud (with adjustment if needed)
	const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

	// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
	assert(normalRef.rows() > 0);

	const Matrix deltas = mPts.reading.features - mPts.reference.features;

	// dotProd = dot(deltas, normals) = d.n
	Matrix dotProd = Matrix::Zero(1, normalRef.cols());
	for(int i = 0; i < normalRef.rows(); i++)
	{
		dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
	}
	// residual = w*(d.n)²
	dotProd = (mPts.weights.row(0).array() * dotProd.array().square()).matrix();

	// return sum of the norm of each dot product
	return dotProd.sum();
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::getResidualError(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches,
	const Penalties& penalties,
	const TransformationParameters& T_refMean_iter,
    const TransformationParameters& T_prior) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches, penalties, T_refMean_iter, T_prior);

	return PointToPlaneErrorMinimizer::computeResidualError(mPts, force2D);
}

template<typename T>
T PointToPlaneErrorMinimizer<T>::getOverlap() const
{

	// Gather some information on what kind of point cloud we have
	const bool hasReadingNoise = this->lastErrorElements.reading.descriptorExists("simpleSensorNoise");
	const bool hasReferenceNoise = this->lastErrorElements.reference.descriptorExists("simpleSensorNoise");
	const bool hasReferenceDensity = this->lastErrorElements.reference.descriptorExists("densities");

	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();

	// basix safety check
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}

	Eigen::Array<T, 1, Eigen::Dynamic>  uncertainties(nbPoints);

	// optimal case
	if (hasReadingNoise && hasReferenceNoise && hasReferenceDensity)
	{
		// find median density

		Matrix densities = this->lastErrorElements.reference.getDescriptorViewByName("densities");
		vector<T> values(densities.data(), densities.data() + densities.size());

		// sort up to half the values
		nth_element(values.begin(), values.begin() + (values.size() * 0.5), values.end());

		// extract median value
		const T medianDensity = values[values.size() * 0.5];
		const T medianRadius = 1.0/pow(medianDensity, 1/3.0);

		uncertainties = (medianRadius +
						this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise").array() +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise").array());
	}
	else if(hasReadingNoise && hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise") +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReadingNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else
	{
		LOG_INFO_STREAM("PointToPlaneErrorMinimizer - warning, no sensor noise and density. Using best estimate given outlier rejection instead.");
		return this->getWeightedPointUsedRatio();
	}


	const Vector dists = (this->lastErrorElements.reading.features.topRows(dim-1) - this->lastErrorElements.reference.features.topRows(dim-1)).colwise().norm();


	// here we can only loop through a list of links, but we are interested in whether or not
	// a point has at least one valid match.
	int count = 0;
	int nbUniquePoint = 1;
	Vector lastValidPoint = this->lastErrorElements.reading.features.col(0) * 2.;
	for(int i=0; i < nbPoints; i++)
	{
		const Vector point = this->lastErrorElements.reading.features.col(i);

		if(lastValidPoint != point)
		{
			// NOTE: we tried with the projected distance over the normal vector before:
			// projectionDist = delta dotProduct n.normalized()
			// but this doesn't make sense 


			if(PointMatcherSupport::anyabs(dists(i, 0)) < (uncertainties(0,i)))
			{
				lastValidPoint = point;
				count++;
			}
		}

		// Count unique points
		if(i > 0)
		{
			if(point != this->lastErrorElements.reading.features.col(i-1))
				nbUniquePoint++;
		}

	}
	//cout << "count: " << count << ", nbUniquePoint: " << nbUniquePoint << ", this->lastErrorElements.nbRejectedPoints: " << this->lastErrorElements.nbRejectedPoints << endl;

	return (T)count/(T)(nbUniquePoint + this->lastErrorElements.nbRejectedPoints);
}

template struct PointToPlaneErrorMinimizer<float>;
template struct PointToPlaneErrorMinimizer<double>;
