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

#include "Parametrizable.h"
#include <boost/format.hpp>

namespace PointMatcherSupport
{
	using namespace std;
	/*
	struct __toto
	{
		__toto()
		{
			typedef PointMatcher<float>::Parametrizable P;
			P p(
				"test",
				"this is a test",
				{ { "val0", "test val0", 1}, { "val1", "test val1", "string" } },
				P::Parameters()
			);
			cout << p << endl;
		}
	};

	static __toto __toto_instance;*/

	template<typename S>
	Parametrizable::ParameterDoc::ParameterDoc(const std::string name, const std::string doc, const S defaultValue):
		name(name),
		doc(doc),
		defaultValue(boost::lexical_cast<string>(defaultValue))
	{}

	Parametrizable::Parametrizable(
		std::initializer_list<ParameterDoc> paramsDoc,
		const Parameters& params):
		parametersDoc(paramsDoc)
	{
		// fill current parameters from either values passed as argument, or default value
		for (auto it = parametersDoc.cbegin(); it != parametersDoc.cend(); ++it)
		{
			const string& paramName(it->name);
			Parameters::const_iterator paramIt(params.find(paramName));
			if (paramIt != params.end())
				parameters[paramName] = paramIt->second;
			else
				parameters[paramName] = it->defaultValue;
		}
	}

	void Parametrizable::dump(std::ostream& o) const
	{
		for (auto it = parametersDoc.cbegin(); it != parametersDoc.cend(); ++it)
			o << it->name << " (" << it->defaultValue << ") - " << it->doc << endl;
	}

	std::string Parametrizable::getParam(const std::string& name) const
	{
		Parameters::const_iterator paramIt(parameters.find(name));
		if (paramIt == parameters.end())
			throw Error((boost::format("Parameter %1 does not exist in object %2") % name % typeid(*this).name()).str());
		// TODO: use string distance to propose close one, copy/paste code from Aseba
		return paramIt->second;
	}
} // namespace PointMatcherSupport