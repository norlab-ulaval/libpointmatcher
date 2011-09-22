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

#ifndef __POINTMATCHER_OUTLIERFILTERS_H
#define __POINTMATCHER_OUTLIERFILTERS_H

struct NullFeatureOutlierFilter: public FeatureOutlierFilter
{
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct MaxDistOutlierFilter: public FeatureOutlierFilter
{
	const T maxDist;
	
	MaxDistOutlierFilter(const T maxDist);

	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct MedianDistOutlierFilter: public FeatureOutlierFilter 
{
	const T factor;
	
	MedianDistOutlierFilter(const T factor);
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};


/* Hard rejection threshold using quantile.
Based on:
	D Chetverikov, "The Trimmed Iterative Closest Point Algorithm" (2002)
*/
struct TrimmedDistOutlierFilter: public FeatureOutlierFilter
{
	const T ratio;
	
	TrimmedDistOutlierFilter(const T ratio);
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

/* Hard rejection threshold using quantile and variable ratio.
Based on:
	J. M. Phillips and al., "Outlier Robust ICP for Minimizing Fractional RMSD" (2007)
*/
struct VarTrimmedDistOutlierFilter: public FeatureOutlierFilter
{
	// default ratio
	T ratio_;
	// min ratio
	T min_;
	// max ratio
	T max_;
	// lambda (part of the term that balance the rmsd: 1/ratio^lambda)
	T lambda_;

	VarTrimmedDistOutlierFilter(const T r, const T min=0.05, const T max=0.99, const T lambda=0.95);
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	
	private:
	// return the optimized ratio
	T optimizeInlierRatio(const Matches& matches);
};

struct MinDistOutlierFilter: public FeatureOutlierFilter
{
	const T minDist;
	
	MinDistOutlierFilter(const T minDist); 
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

// ---------------------------------

struct NullDescriptorOutlierFilter: public DescriptorOutlierFilter
{
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

#endif // __POINTMATCHER_OUTLIERFILTERS_H
