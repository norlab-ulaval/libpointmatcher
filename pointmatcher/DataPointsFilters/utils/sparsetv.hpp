// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018

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

//Import "and", "or", "not" macros
#include <iso646.h>

//--ctor
template <typename T>
TensorVoting<T>::TensorVoting(T sigma_, std::size_t k_) : sigma{sigma_}, k{k_}
{
}
//--dtor
template <typename T>
TensorVoting<T>::~TensorVoting(){}

//--methods

///Voting methods
template <typename T>
void TensorVoting<T>::vote(const DP& pts)
{
	refine(pts);
}
	
template <typename T>
void TensorVoting<T>::encode(const DP& pts, Encoding encoding)
{
	const std::size_t nbPts = pts.getNbPoints();
	tensors.resize(nbPts, 1);
	
	switch(encoding)
	{
		case Encoding::ZERO:
		{
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i) 
				tensors(i) = Tensor::Zero();
			break;
		}
//------		
		case Encoding::UBALL:
		case Encoding::BALL:
		{
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
				tensors(i) = Tensor::Identity();
			break;
		}
		case Encoding::SBALL:
		{
			//Check if there is plates info
			if (not pts.descriptorExists("balls"))
				throw InvalidField("TensorVoting<T>::encode: Error, cannot find balls in descriptors.");

			const auto& balls_ = pts.getDescriptorViewByName("balls");
#pragma omp parallel for						
			for(std::size_t i = 0; i < nbPts; ++i)
				tensors(i) = Tensor::Identity() * balls_(0,i);

			break;
		}
//------
		case Encoding::UPLATE:
		{
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
				tensors(i) << 
					1., 0., 0., 
					0., 1., 0., 
					0., 0., 0.;
			break;
		}			
		case Encoding::PLATE:
		case Encoding::SPLATE:
		{
			//Check if there is plates info
			if (not pts.descriptorExists("plates"))
				throw InvalidField("TensorVoting<T>::encode: Error, cannot find plates in descriptors.");

			const auto& plates_ = pts.getDescriptorViewByName("plates");
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
			{
				const Vector3 n1 = plates_.col(i).segment(1,3);
				const Vector3 n2 = plates_.col(i).tail(3);
				
				tensors(i) = (encoding == Encoding::SPLATE ? plates_(0,i) : 1.) * (n1 * n1.transpose() + n2 * n2.transpose());
			}
			break;
		}
//------	
		case Encoding::USTICK:
		{
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
				tensors(i) << 
					1., 0., 0., 
					0., 0., 0., 
					0., 0., 0.;
			break;
		}				
		case Encoding::STICK:
		case Encoding::SSTICK:
		{
			//Check if there is normals info
			if (not pts.descriptorExists("sticks"))
				throw InvalidField("TensorVoting<T>::encode: Error, cannot find sticks in descriptors.");

			const auto& sticks_ = pts.getDescriptorViewByName("sticks");
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
			{
				const Vector3 n = sticks_.col(i).tail(3);		
				tensors(i) = (encoding == Encoding::SSTICK ? sticks_(0,i) : 1.) * (n * n.transpose());
			}
			break;
		}
//------
		case Encoding::AWARE_TENSOR:
		{
			//Check if there is normals info
			if (not pts.descriptorExists("sticks"))
				throw InvalidField("TensorVoting<T>::encode: Error, cannot find sticks in descriptors.");
			//Check if there is normals info
			if (not pts.descriptorExists("plates"))
				throw InvalidField("TensorVoting<T>::encode: Error, cannot find plates in descriptors.");

			const auto& sticks_ = pts.getDescriptorViewByName("sticks");
			const auto& plates_ = pts.getDescriptorViewByName("plates");
#pragma omp parallel for
			for(std::size_t i = 0; i < nbPts; ++i)
			{
				const Tensor S = sticks_.col(i).tail(3) * sticks_.col(i).tail(3).transpose();
				const Tensor P = plates_.col(i).segment(1,3) * plates_.col(i).segment(1,3).transpose() + plates_.col(i).tail(3) * plates_.col(i).tail(3).transpose();
		
				tensors(i) = (sticks_(0,i) / k) * S + (plates_(0,i) / k) * P;
			}
			break;
		}	
	}
}

template <typename T>
void TensorVoting<T>::disableBallComponent()
{
	const std::size_t nbPts = tensors.rows();
#pragma omp parallel for
	for(std::size_t i = 0; i < nbPts; ++i)
	{
		const Tensor S = sticks.col(i).tail(3) * sticks.col(i).tail(3).transpose();
		const Tensor P = plates.col(i).segment(1,3) * plates.col(i).segment(1,3).transpose() + plates.col(i).tail(3) * plates.col(i).tail(3).transpose();
		
		tensors(i) = (sticks(0,i) / k) * S + (plates(0,i) / k) * P;
	}
}

template <typename T>
void TensorVoting<T>::refine(const DP& pts)
{
	//pts = input token (x,y,z)
	//		|
	encode(pts); 
	//		|
	//tensors = unit ball tensors token (sparse)
	//		|
	ballVote(pts, true);
	//		|
	//tensors = generic 2nd order tensors refined tokens (sparse)
	//		|
	decompose();
	//		|
	//sparseStick = tensors (saliency, nx, ny, nz)
	//sparsePlate = tensors (saliency, tx, ty, tz)
	//sparseBall = tensors (saliency, 0, 0, 0)
	//		|
	toDescriptors();
}

/************ Ball Vote ********************************************************
 * The ball vote can be constructed by subtracting the direct product of 
 * the tangent vector from a full rank tensor with equal eigenvalues 
 * (i.e. the identity matrix). The resulting tensor is attenuated by the same
 * Gaussian weight according to the distance between the voter and the receiver
 ******************************************************************************/
template <typename T>
void TensorVoting<T>::ballVote(const DP& pts, bool doKnn)
{
	const std::size_t nbPts = pts.getNbPoints();
	
	const Tensor I = Tensor::Identity();
	
	if(doKnn) computeKnn(pts);
	
	for(std::size_t voter = 0; voter < nbPts; ++voter)
	{
		for(std::size_t j = 0; j < k ; j++)
		{
			Index votee = indices(j,voter);
		
			if(votee == NNS::InvalidIndex) break;
			if(votee == Index(voter)) continue;

			const Vector3 v = pts.features.col(votee).head(3) - pts.features.col(voter).head(3);
			const T normDist = v.norm()/sigma;
			
			if(normDist > 0. and normDist < 3.) //if not too far
			{
					const Tensor vv = v * v.transpose(); //outer product for projection in direction of voter
					const T normVv = vv.norm(); //frobenius norm
					if(normVv > 0.)
					{
						//accumulate vote a votee location with decay function weighting voter vote
						const Tensor acc = tensors(votee) + DecayFunction::sradial(normDist) * (I - vv / normVv);
						tensors(votee) = acc;
					}
			}	
		}
	}
}

	
template <typename T>
void TensorVoting<T>::stickVote(const DP& pts, bool doKnn)
{
	//Check if there is normals info
	if (not pts.descriptorExists("sticks"))
		throw InvalidField("TensorVoting<T>::stickVote: Error, cannot find sticks in descriptors.");

	const auto& sticks_ = pts.getDescriptorViewByName("sticks");
	
	const std::size_t nbPts = pts.getNbPoints();
		
	if(doKnn) computeKnn(pts);
	
	for(std::size_t voter = 0; voter < nbPts; ++voter)
	{
		Vector3 vn = sticks_.col(voter).tail(3).normalized(); //normal
		const Vector3 O  = pts.features.col(voter).head(3); //voter coord

		for(std::size_t j = 0; j < k ; j++)
		{
			Index votee = indices(j,voter);
		
			if(votee == NNS::InvalidIndex) break;
			if(votee == Index(voter)) continue;

			Vector3 v = pts.features.col(votee).head(3) - O; // v = P - O 
			
			const T normDist = v.norm()/sigma;
			
			if(normDist > 0. and normDist < 3.) //if not too far
			{
				v.normalize();
				
				const T vvn = v.dot(vn); // size of point
				if(vvn < 0.) vn *= -1.; //reorient normal

				const T theta = std::asin(vvn); //sepatation angle votee--voter along tangent in [- PI/2; PI/2]

				//cast vote only if smaller than 45deg
				if (std::fabs(theta) <= M_PI / 4. or std::fabs(theta) >= 3. * M_PI / 4.)
				{
					//get tangent vector to osculating circle
					Vector3 vt = vn.cross(v.cross(vn)).normalized();
	
					const T vvt = v.dot(vt); // size of point						
					if(vvt < 0.) vt *= -1.; //reorient tangent
	
					//most likely normal at P
					const Vector3 vc = vn * std::cos(2. * theta) - vt * std::sin(2. * theta); //vote cast								
		
					//accumulate vote a votee location with decay function weighting voter vote			
					const Tensor acc = tensors(votee) + sticks_(0, voter) * DecayFunction::eta(normDist * sigma, sigma, vvn) * (vc * vc.transpose());					
					tensors(votee) = acc;
				}
			}	
		}
	}
}

//FIXME: not sure of the implementation...
template <typename T>
void TensorVoting<T>::plateVote(const DP& pts, bool doKnn)
{		
	//Check if there is normals info
	if (not pts.descriptorExists("plates"))
		throw InvalidField("TensorVoting<T>::stickVote: Error, cannot find plates in descriptors.");

	const auto& plates_ = pts.getDescriptorViewByName("plates");
	
	const std::size_t nbPts = pts.getNbPoints();
		
	if(doKnn) computeKnn(pts);
		
	for(std::size_t voter = 0; voter < nbPts; ++voter)
	{
		Matrix U(3,2); U << plates_.col(voter).segment(1,3), plates_.col(voter).tail(3);
		const Matrix Ns = U*U.transpose(); //normalspace
		
		for(std::size_t d = 0; d <= 1 ; ++d)
		{
			Vector3 vn = Ns.col(d).normalized(); //vector basis in normal space	
			const Vector3 O  = pts.features.col(voter).head(3); //voter coord
			
			for(std::size_t j = 0; j < k ; j++)
			{
				Index votee = indices(j,voter);
		
				if(votee == NNS::InvalidIndex) break;
				if(votee == Index(voter)) continue;

				Vector3 v = pts.features.col(votee).head(3) - O; // v = P - O 
			
				const T normDist = v.norm()/sigma;
			
				if(normDist > 0. and normDist < 3.) //if not too far
				{
					v.normalize();
						
					const T vvn = v.dot(vn); // size of point
					if(vvn < 0.) vn *= -1.; //reorient normal
			
					const T theta = std::asin(vvn); //sepatation angle voter--votee
					
					//cast vote only if smaller than 45deg
					if (std::fabs(theta) <= M_PI / 4. or std::fabs(theta) >= 3. * M_PI / 4.)
					{	
						//get tangent vector to osculating circle
						Vector3 vt = vn.cross(v.cross(vn)).normalized();

						const T vvt = v.dot(vt); // size of point						
						if(vvt < 0.) vt *= -1.; //reorient tangent
					
						const Vector3 vc = vn * std::cos(2. * theta) - vt * std::sin(2. * theta); //vote cast
			
						//accumulate vote a votee location with decay function weighting voter vote			
						const Tensor acc = tensors(votee) + plates_(0, voter) * DecayFunction::eta(normDist*sigma, sigma, vvn) * (vc * vc.transpose());				
						tensors(votee) = acc;
					}
				}	
			}
		}
	}
}

/*******************************************************************************
 * See Eq (11) in:
 * 		T.-P. Wu, S.-K. Yeung, J. Jia, C.-K. Tang, and G. Medioni, 
 * 		“A Closed-Form Solution to Tensor Voting: Theory and Applications,” 2016.
 ******************************************************************************/
template <typename T>
void TensorVoting<T>::cfvote(const DP& pts, bool doKnn)
{
	const std::size_t nbPts = pts.getNbPoints();
		
	if(doKnn) computeKnn(pts);
	
	const Tensors K = tensors; //save old tensors values
	encode(pts, Encoding::ZERO); //all tensors are zero
	
#pragma omp parallel for
	for(std::size_t votee = 0; votee < nbPts; ++votee) //vote sites
	{
		const Vector3 x_i  = pts.features.col(votee).head(3);
		
		for(std::size_t j = 0; j < k ; j++) //voters
		{
			const Index voter = indices(j,votee); //get voter at site Xi
		
			if(voter == NNS::InvalidIndex) continue;		
			if(voter == Index(votee)) continue;
			
			const Vector3 x_j = pts.features.col(voter).head(3); 
			Vector3 r_ij = x_i - x_j;
			
			const T normDist = r_ij.norm() / sigma;
			
			if(normDist > 0. and normDist < 3.) //if not too far
			{
				r_ij.normalize();

				const Tensor rrt = r_ij * r_ij.transpose();
				const Tensor R_ij = (Tensor::Identity() - 2. * rrt);
				const Tensor Rp_ij  = (Tensor::Identity() - .5 * rrt) * R_ij;
				const T c_ij = DecayFunction::cij((x_i - x_j).norm(), sigma);
				
				//accumulate vote a voter location with decay function weighting voter vote	
				const Tensor S_ij = c_ij * R_ij * K(voter) * Rp_ij;
				
				const Tensor acc = tensors(votee) + S_ij;
				tensors(votee) = acc;
			}	
		}
	}
}

template <typename T>
void TensorVoting<T>::decompose()
{
	const std::size_t nbPts = tensors.rows();

	sparseStick.resize(nbPts);
	sparsePlate.resize(nbPts);
	sparseBall.resize(nbPts);
	
#pragma omp parallel for	
	for(std::size_t i = 0; i < nbPts; ++i)
	{
		Eigen::SelfAdjointEigenSolver<Tensor> solver(tensors(i));
		
		const Matrix33 eigenVe = solver.eigenvectors();
		const Vector3 eigenVa = solver.eigenvalues().array().abs();
		
		// lambda1 > lambda2 > lambda3 > 0
		int lambda1_idx; const T lambda1 = eigenVa.maxCoeff(&lambda1_idx);
		int lambda3_idx; const T lambda3 = eigenVa.minCoeff(&lambda3_idx);
		const int lambda2_idx = (0+1+2) - (lambda1_idx + lambda3_idx);
		const T lambda2 = eigenVa(lambda2_idx);
		
		const T norm = 1;   
		
		if(not (lambda1 >= lambda2 and lambda2 >= lambda3) or lambda2_idx > 2. or lambda2_idx < 0.)
		{
			sparseStick(i) 	<< 0.0001,0.,0.,0.;
			sparsePlate(i) 	<< 0.0001,0.,0.,0.;
			sparseBall(i) 	<< 0.0001,0.,0.,0.;
			
			//std::cerr << "Warning: eigen values not ordered ("<<eigenVa(0)<<", "<<eigenVa(1)<<", "<<eigenVa(2)<<")" << std::endl;
			continue;
		}
				
		// store relevant stick, plate, ball information:
		sparseStick(i)(0) = (lambda1 - lambda2) / norm; //
		sparseStick(i).tail(3) = eigenVe.col(lambda1_idx); //normal information
		
		sparsePlate(i)(0) = (lambda2 - lambda3) / norm; //
		sparsePlate(i).tail(3) = eigenVe.col(lambda3_idx); //tangent information
		
		sparseBall(i)(0) = lambda3 / norm; //
		sparseBall(i).tail(3) = eigenVe.col(lambda2_idx); //<< 0.,0.,0.; //no principal direction, but store lambda2 for convinience
	}
}

template <typename T>
void TensorVoting<T>::toDescriptors()
{
	const std::size_t nbPts = tensors.rows();
	 
	pointness = PM::Matrix::Zero(1, nbPts);
	curveness = PM::Matrix::Zero(1, nbPts);
	surfaceness = PM::Matrix::Zero(1, nbPts);
  
	normals		= PM::Matrix::Zero(3, nbPts);
	tangents	= PM::Matrix::Zero(3, nbPts);
  
	sticks = PM::Matrix::Zero(4, nbPts);
	plates = PM::Matrix::Zero(7, nbPts);
	balls  = PM::Matrix::Zero(1, nbPts);

#pragma omp parallel for  
	for(std::size_t i = 0; i < nbPts; i++)
	{
		surfaceness(i) = sparseStick(i)(0) / k;
		curveness(i) = sparsePlate(i)(0) / k;
		pointness(i) = sparseBall(i)(0) / k;

		normals.col(i) = sparseStick(i).tail(3);
		tangents.col(i) = sparsePlate(i).tail(3);
		
		sticks.col(i) = sparseStick(i); //s + e1
		
		plates(0,i) = sparsePlate(i)(0); //s
		plates.col(i).segment(1,3) = sparseStick(i).tail(3); //e1
		plates.col(i).tail(3) = sparseBall(i).tail(3); //e2
		
		balls(i) = sparseBall(i)(0); //s
	}
}

template <typename T>
void TensorVoting<T>::computeKnn(const DP& pts)
{
	const std::size_t nbPts = pts.getNbPoints();
	
	if(k >= nbPts) k = nbPts - 1;
	
	std::shared_ptr<NNS> knn(
		NNS::create(pts.features, pts.features.rows() - 1, 
			(k<30? NNS::SearchType::KDTREE_LINEAR_HEAP : NNS::SearchType::KDTREE_TREE_HEAP)	
		)
	);

	indices = IndexMatrix::Zero(k, nbPts);
	dist = Matrix::Zero(k, nbPts);

	knn->knn(pts.features, indices, dist, Index(k));
}
