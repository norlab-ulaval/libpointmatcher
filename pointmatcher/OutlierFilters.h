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
	static const std::string description()
	{
		return "does nothing";
	}
	
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct MaxDistOutlierFilter: public FeatureOutlierFilter
{
	static const std::string description()
	{
		return "This filter considers as outlier links whose norms are above a threshold.";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "maxDist", "threshold distance", "1", "0.0000001", "inf", &P::Comp<T>} 
		});
	}
	
	const T maxDist;
	
	MaxDistOutlierFilter(const Parameters& params = Parameters());
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct MinDistOutlierFilter: public FeatureOutlierFilter
{
	static const std::string description()
	{
		return "This filter considers as outlier links whose norms are below a threshold.";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "minDist", "threshold distance", "1", "0.0000001", "inf", &P::Comp<T>} 
		});
	}
	
	const T minDist;
	
	MinDistOutlierFilter(const Parameters& params = Parameters()); 
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct MedianDistOutlierFilter: public FeatureOutlierFilter 
{
	static const std::string description()
	{
		return "This filter considers as outlier links whose norms are above the median link norms times a factor.";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "factor", "points farther away factor * median will be considered outliers.", "3", "0.0000001", "inf", &P::Comp<T>}
		});
	}
	
	const T factor;
	
	MedianDistOutlierFilter(const Parameters& params = Parameters());
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct TrimmedDistOutlierFilter: public FeatureOutlierFilter
{
	static const std::string description()
	{
		return "Hard rejection threshold using quantile. This filter considers as inlier a certain percentage of the links with the smallest norms. Based on: D Chetverikov, \"The Trimmed Iterative Closest Point Algorithm\" (2002)";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "ratio", "percentage to keep", "0.75", "0.0000001", "0.9999999", &P::Comp<T>}
		});
	}
	
	const T ratio;
	
	TrimmedDistOutlierFilter(const Parameters& params = Parameters());
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

struct VarTrimmedDistOutlierFilter: public FeatureOutlierFilter
{
	static const std::string description()
	{
		return "Hard rejection threshold using quantile and variable ratio. Based on: J. M. Phillips and al., \"Outlier Robust ICP for Minimizing Fractional RMSD\" (2007)";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "minRatio", "min ratio", "0.05", "0.0000001", "1", &P::Comp<T>},
			{ "maxRatio", "max ratio", "0.99", "0.0000001", "1", &P::Comp<T>},
			{ "lambda", "lambda (part of the term that balance the rmsd: 1/ratio^lambda", "0.95" }
		});
	}

	const T minRatio;
	const T maxRatio;
	const T lambda;
	
	VarTrimmedDistOutlierFilter(const Parameters& params = Parameters());
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
	
private:
	// return the optimized ratio
	T optimizeInlierRatio(const Matches& matches);
};


// ---------------------------------

struct NullDescriptorOutlierFilter: public DescriptorOutlierFilter
{
	static const std::string description()
	{
		return "does nothing";
	}
	
	virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input, bool& iterate);
};

#endif // __POINTMATCHER_OUTLIERFILTERS_H
