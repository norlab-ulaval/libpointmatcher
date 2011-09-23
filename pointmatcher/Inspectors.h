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

#ifndef __POINTMATCHER_INSPECTORS_H
#define __POINTMATCHER_INSPECTORS_H

// Clearer name when no inspector is required
struct NullInspector: public Inspector
{
	static const std::string description()
	{
		return "does nothing";
	}
};

struct AbstractVTKInspector: public Inspector
{
protected:
	virtual std::ostream* openStream(const std::string& role) = 0;
	virtual std::ostream* openStream(const std::string& role, const size_t iterationCount) = 0;
	virtual void closeStream(std::ostream* stream) = 0;
	void dumpDataPoints(const DataPoints& data, std::ostream& stream);
	void dumpMeshNodes(const DataPoints& data, std::ostream& stream);
	void dumpDataLinks(const DataPoints& ref, const DataPoints& reading, 	const Matches& matches, const OutlierWeights& featureOutlierWeights, std::ostream& stream);
	
	std::ostream* streamIter;

public:
	AbstractVTKInspector(const ParametersDoc paramsDoc, const Parameters& params);
	virtual void init() {};
	virtual void dumpDataPoints(const DataPoints& cloud, const std::string& name);
	virtual void dumpMeshNodes(const DataPoints& cloud, const std::string& name);
	virtual void dumpIteration(const size_t iterationCount, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& featureOutlierWeights, const OutlierWeights& descriptorOutlierWeights, const TransformationCheckers& transformationCheckers);
	virtual void finish(const size_t iterationCount);

private:
	void buildGenericAttributeStream(std::ostream& stream, const std::string& attribute, const std::string& nameTag, const DataPoints& cloud, const int forcedDim);

	void buildScalarStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
	void buildScalarStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
	
	void buildNormalStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
	void buildNormalStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
	
	void buildVectorStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
	void buildVectorStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);
	
	void buildTensorStream(std::ostream& stream, const std::string& name, const DataPoints& ref, const DataPoints& reading);
	void buildTensorStream(std::ostream& stream, const std::string& name, const DataPoints& cloud);

	Matrix padWithZeros(const Matrix m, const int expectedRow, const int expectedCols); 
};

struct VTKFileInspector: public AbstractVTKInspector
{
	static const std::string description()
	{
		return "Dump the different steps into VTK files.";
	}
	static const ParametersDoc availableParameters()
	{
		return ParametersDoc({
			{ "baseFileName", "base file name for the VTK files ", "point-matcher-output" }
		});
	}
	
	const std::string baseFileName;
	
protected:
	virtual std::ostream* openStream(const std::string& role);
	virtual std::ostream* openStream(const std::string& role, const size_t iterationCount);
	virtual void closeStream(std::ostream* stream);
	
public:
	VTKFileInspector(const Parameters& params = Parameters());
	virtual void init();
	virtual void finish(const size_t iterationCount);
};

#endif // __POINTMATCHER_INSPECTORS_H
