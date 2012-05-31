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

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <time.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace std;
using namespace Eigen;

const float floatRand(float val);


/**
  * Tool generating random poses. Useful for the executable "evaluate_dataset" 
  */
int main(int argc, char *argv[])
{
	if (!(argc == 9))
	{
		cerr << "Error in command line, usage " << argv[0] << " fileName nbPose  dx dy dz droll dpitch dyaw" << endl;
		cerr << "Translation component in meter and rotation radian." << endl;
		abort();
	}
	
	boost::mt19937 eng(time(0)); // See how to seed that
	typedef boost::normal_distribution<float> NormalsDist;

	srand(time(NULL));
	std::fstream outFile;
	outFile.open(argv[1], fstream::out);

	const int nbPose = atoi(argv[2]);
	const float dx = atof(argv[3]);
	const float dy = atof(argv[4]);
	const float dz = atof(argv[5]);
	const float droll = atof(argv[6]);
	const float dpitch = atof(argv[7]);
	const float dyaw = atof(argv[8]);

	typedef boost::variate_generator<boost::mt19937&, NormalsDist> RandGenerator;
	RandGenerator rand_normal_x(eng, NormalsDist(0.0, dx));
	RandGenerator rand_normal_roll(eng, NormalsDist(0.0, droll));

	RandGenerator rand_normal_y(eng, NormalsDist(0.0, dy));
	RandGenerator rand_normal_z(eng, NormalsDist(0.0, dz));
	RandGenerator rand_normal_pitch(eng, NormalsDist(0.0, dpitch));
	RandGenerator rand_normal_yaw(eng, NormalsDist(0.0, dyaw));


	//cout << nbPose << " " << dx << " " << dy << " " << dz << " " << droll << " " << dpitch << " " << dyaw << endl;

	for(int x=0; x<4; x++)
	{
		for(int y=0; y<4; y++)
		{
			outFile << "iT" << x << y << ", ";
		}
	}
	outFile << endl;

	for(int i = 0; i < nbPose; i++)
	{
		// convention roll, pitch, yaw (1,2,3)
		Matrix3f rot;
		rot = AngleAxisf(rand_normal_roll(), Vector3f::UnitX())
		    * AngleAxisf(rand_normal_yaw(), Vector3f::UnitY())
			* AngleAxisf(rand_normal_pitch(), Vector3f::UnitZ());
		const float x = rand_normal_x();
		const float y = rand_normal_y();
		const float z = rand_normal_z();
		//rot = AngleAxisf(rand_uniform(droll), Vector3f::UnitX())
		//    * AngleAxisf(rand_uniform(dyaw), Vector3f::UnitY())
		//	* AngleAxisf(rand_uniform(dpitch), Vector3f::UnitZ());
		//const float x = rand_uniform(dx);
		//const float y = rand_uniform(dy);
		//const float z = rand_uniform(dz);
		outFile << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << ", " << x << ", "
	          << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << ", " << y << ", "
		        << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << ", " << z << ", "
		        << 0        << ", " << 0        << ", " << 0        << ", " << 1;
		outFile << endl;
	}

	outFile.close();

	return 0;
}

const float rand_uniform(float val)
{
	return ((2*rand()-RAND_MAX) / (float)RAND_MAX)*val;
}
