// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2011,
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

#ifndef __POINTMATCHER_TRANSFORMATIONCHECKERS_H
#define __POINTMATCHER_TRANSFORMATIONCHECKERS_H

struct CounterTransformationChecker: public TransformationChecker
{
	static const std::string description()
	{
		return "This checker stops the ICP loop after a certain number of iterations.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "maxIterationCount", "maximum number of iterations ", 100, 0 }
		};
	}
	
	const int maxIterationCount;
	
	CounterTransformationChecker(const Parameters& params = Parameters());
	virtual void init(const TransformationParameters& parameters, bool& iterate);
	virtual void check(const TransformationParameters& parameters, bool& iterate);
};

struct ErrorTransformationChecker: public TransformationChecker
{
	static const std::string description()
	{
		return "This checker stops the ICP loop when the average transformation errors are below thresholds.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "minDeltaRotErr", "threshold for rotation error (radian)", 0.001, 0., 2*M_PI },
			{ "minDeltaTransErr", "threshold for translation error", 0.001, 0.},
			{ "tail", "number of iterations over which to average error", 3, 0 }
		};
	}
	
	const T minDeltaRotErr;
	const T minDeltaTransErr;
	const unsigned int tail;

protected:
	QuaternionVector rotations;
	VectorVector translations;

public:
	ErrorTransformationChecker(const Parameters& params = Parameters());
	
	virtual void init(const TransformationParameters& parameters, bool& iterate);
	virtual void check(const TransformationParameters& parameters, bool& iterate);
};

struct BoundTransformationChecker: public TransformationChecker
{
	static const std::string description()
	{
		return "This checker stops the ICP loop with an exception when the transformation values exceed bounds.";
	}
	static const ParametersDoc availableParameters()
	{
		return {
			{ "maxRotationNorm", "rotation bound", 1., 0. },
			{ "maxTranslationNorm", "translation bound", 1., 0. }
		};
	}
		
	const T maxRotationNorm;
	const T maxTranslationNorm;
	
protected:
	Quaternion initialRotation;
	Vector initialTranslation;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BoundTransformationChecker(const Parameters& params = Parameters());
	virtual void init(const TransformationParameters& parameters, bool& iterate);
	virtual void check(const TransformationParameters& parameters, bool& iterate);
};

#endif // __POINTMATCHER_TRANSFORMATIONCHECKERS_H
