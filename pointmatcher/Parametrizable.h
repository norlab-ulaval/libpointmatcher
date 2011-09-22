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

#ifndef __POINTMATCHER_PARAMETRIZABLE_H
#define __POINTMATCHER_PARAMETRIZABLE_H

#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <boost/lexical_cast.hpp>
#include <limits>

namespace PointMatcherSupport
{
	template<typename S>
	std::string toParam(const S& value)
	{
		return boost::lexical_cast<std::string>(value);
	}
	
	struct Parametrizable
	{
		struct InvalidParameter: std::runtime_error
		{
			InvalidParameter(const std::string& reason):runtime_error(reason) {}
		};
		
		//! return whether a is smaller than b
		typedef bool(*LexicalComparison)(std::string a, std::string b);
		
		template<typename S>
		static bool Comp(std::string a, std::string b)
		{
			return boost::lexical_cast<S>(a) < boost::lexical_cast<S>(b);
		}
		
		struct ParameterDoc
		{
			std::string name;
			std::string doc;
			std::string defaultValue;
			std::string minValue;
			std::string maxValue;
			LexicalComparison comp;
			
			/*
			This code is beautiful, this code is correct, this code does not work ;-(
			Blame gcc bug 9050 (http://gcc.gnu.org/bugzilla/show_bug.cgi?id=9050), shame
			on them forever and beyond. People being laaaazzzy adopters, I'm forced to use
			something that work on gcc 4.4.
			
			template<typename S>
			ParameterDoc(const std::string name, const std::string doc, const S defaultValue, const S minValue, const S maxValue = std::numeric_limits<S>::max());
			template<typename S>
			ParameterDoc(const std::string name, const std::string doc, const S defaultValue);
			*/
			ParameterDoc(const std::string name, const std::string doc, const std::string defaultValue, const std::string minValue, const std::string maxValue, LexicalComparison comp);
			ParameterDoc(const std::string name, const std::string doc, const std::string defaultValue);
			
			friend std::ostream& operator<< (std::ostream& o, const ParameterDoc& p);
		};
		
		struct ParametersDoc : public std::vector<ParameterDoc>
		{
			ParametersDoc(std::initializer_list<ParameterDoc> list):std::vector<ParameterDoc>(list){}
			ParametersDoc() {}
			friend std::ostream& operator<< (std::ostream& o, const ParametersDoc& p);
		};
		
		/*
		Again, not used because fo gcc bug 9050
		struct Parameter: public std::string
		{
			template<typename S>
			Parameter(const S value);
			Parameter(){}
		};
		*/
		typedef std::string Parameter;
		typedef std::map<std::string, Parameter> Parameters;
		
		const ParametersDoc parametersDoc;
		
		Parameters parameters;
		
		Parametrizable(){}
		Parametrizable(const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Parametrizable(){}
		
		std::string getParamValueString(const std::string& paramName) const;
		
		template<typename S>
		S get(const std::string& paramName) const { return boost::lexical_cast<S>(getParamValueString(paramName)); }
		
		friend std::ostream& operator<< (std::ostream& o, const Parametrizable& p) { o << p.parametersDoc; return o; }
	};
} // namespace PointMatcherSupport

#endif // __POINTMATCHER_PARAMETRIZABLE_H
