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

#ifndef __POINTMATCHER_MATCHERS_H
#define __POINTMATCHER_MATCHERS_H

struct NullMatcher: public Matcher
{
	static const std::string description()
	{
		return "does nothing, return no matches";
	}
	
	virtual void init(const DataPoints& filteredReference, bool& iterate);
	virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference, bool& iterate);
};

struct KDTreeMatcher: public Matcher
{
	static const std::string description()
	{
		return "This matcher matches a point from the reading to its closest neighbors in the reference.";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "knn", "number of nearest neighbors to consider it the reference", "1", "1", "2147483647", &P::Comp<unsigned> },
			{ "epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T> },
			{ "searchType", "Nabo search type", "1", "0", "4", &P::Comp<unsigned> },
			{ "maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T>}
		});
	}
	
	const int knn;
	const T epsilon;
	const NNSearchType searchType;
	const T maxDist;

protected:
	NNS* featureNNS;

public:
	KDTreeMatcher(const Parameters& params = Parameters());
	virtual ~KDTreeMatcher();
	virtual void init(const DataPoints& filteredReference, bool& iterate);
	virtual Matches findClosests(const DataPoints& filteredReading, const DataPoints& filteredReference, bool& iterate);
};

#endif // __POINTMATCHER_MATCHERS_H
