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
#pragma once

#include "pointmatcher/PointMatcher.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <nabo/nabo.h>

/*!
 * \class sparsetv.h
 * \brief Tensor Voting framework for inference of structures
 *
 * \author Mathieu Labussiere (<mathieu dot labu at gmail dot com>)
 * \date 27/07/2018
 * \version 0.1
 *
 */
//Sparse Tensor Voting
template <typename T>
struct TensorVoting 
{
//--types
	using PM = PointMatcher<T>;
	using DP = typename PM::DataPoints;
	
	using InvalidField = typename DP::InvalidField;
	
	using SparseTensor = Eigen::Matrix<T, 4, 1>; // (saliency, ex,ey,ez)
	using Tensor = Eigen::Matrix<T, 3, 3>;
	
	using Field = Eigen::Matrix< Tensor, Eigen::Dynamic, 1>;
	using Tensors = Eigen::Matrix< Tensor, Eigen::Dynamic, 1>;
	using SparseTensors = Eigen::Matrix< SparseTensor, Eigen::Dynamic, 1>;
	
	using Vector3  = Eigen::Matrix<T, 3, 1>;
	using Matrix33  = Eigen::Matrix<T, 3, 3>;

	using Matrix = typename PM::Matrix;
	using Vector = typename PM::Vector;
	
	using NNS = Nabo::NearestNeighbourSearch<T>;
	using Index = typename NNS::Index;
	using IndexMatrix = typename NNS::IndexMatrix;
	
	enum Encoding : std::uint8_t { ZERO, UBALL, BALL, SBALL, UPLATE, PLATE, SPLATE, USTICK, STICK, SSTICK, AWARE_TENSOR};

public:
//--attributes
	const T sigma; //control the scale of the voting field
	std::size_t k; //number of neighbors

	Tensors tensors;
	
	SparseTensors sparseStick;
	SparseTensors sparsePlate;
	SparseTensors sparseBall;
	
public:	
//--resulting descriptors
	Matrix surfaceness; //surface saliency map (stick)
	Matrix curveness; //curve saliency map (plate)
	Matrix pointness; //junction saliency map (ball)
	
	Matrix normals; //normals (stick orientation)
	Matrix tangents; //tangents (plate orientation)
	
	Matrix sticks; //(s,n)
	Matrix plates; //(s,n1,n2)
	Matrix balls; // (s)
private:
//knn
	IndexMatrix indices;
	Matrix dist;

public:
//--ctor
	TensorVoting(T sigma_ = T(0.2), std::size_t k_ = 50);
//--dtor
	~TensorVoting();

//--methods

///Voting methods
	void vote(const DP& pts);
	void refine(const DP& pts);
	
	void encode(const DP& pts, Encoding encoding = Encoding::UBALL);
	
	void disableBallComponent();
	
	void ballVote(const DP& pts, bool doKnn = true);
	void stickVote(const DP& pts, bool doKnn = true);
	void plateVote(const DP& pts, bool doKnn = true);
	
	void cfvote(const DP& pts, bool doKnn = true);
	
	void decompose();
	
	void toDescriptors();

///Kernel methods
	struct DecayFunction
	{
		static inline T sradial(const T z)
		{
			if(z < 3.)
				return std::pow(z, 2) * std::pow(z - 3, 4) / 16.;
			else
				return 0.;
		}

		static inline T radial(const T z)
		{
			return std::exp(-std::pow(z, 2));
		}

		static inline T angular(const T theta)
		{
			return std::pow(std::cos(theta), 8);
		}
		
		/***************************************************************************
		 * See Eq (1) in :
		 * 		G. Guy and G. Medioni, “Inference of surfaces, 3D curves, and junctions 
		 * 		from sparse, noisy, 3D Data,” 1997.
		 **************************************************************************/
		static inline T diabolo(const T s, const T c, const T k, const T sigma)
		{
			return std::exp(-1. * std::pow(s, 2) / std::pow(sigma, 2) - c * std::pow(k, 2) );
		}
		
		/***************************************************************************
		 * See Eq (1) and Eq (2) in :
		 * 		T.-P. Wu, S.-K. Yeung, J. Jia, C.-K. Tang, and G. Medioni, 
		 * 		“A Closed-Form Solution to Tensor Voting: Theory and Applications,” 2016.
		 ***************************************************************************/
		static inline T eta(const T vv, const T sigma, const T vvn)
		{
			return cij(vv, sigma) * (1. - vvn * vvn);
		}
		
		static inline T cij(const T vv, const T sigma)
		{
			return std::exp(- std::pow(vv, 2) / sigma);
		}
	};
private:
	void computeKnn(const DP& pts);
};

#include "sparsetv.hpp"
