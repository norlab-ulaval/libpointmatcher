// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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
#include <set>
#include <string>
#include <boost/lexical_cast.hpp>
#include <limits>
#define BOOST_ASSIGN_MAX_PARAMS 6
#include <boost/assign/list_inserter.hpp>
#include <boost/algorithm/string.hpp>


namespace PointMatcherSupport
{	
	//! A lexical casting function that is an improvements over boost::lexical_cast that can handle "inf", "-inf", "Nan" for float and doubles
	template<typename Target>
	inline Target lexical_cast_scalar_to_string(const std::string& arg)
	{
		if (arg == "inf")
			return std::numeric_limits<Target>::infinity();
		else if (arg == "-inf")
			return -std::numeric_limits<Target>::infinity();
		else if (arg == "nan")
			return std::numeric_limits<Target>::quiet_NaN();
		else
			return boost::lexical_cast<Target>(arg);
	}

	//! Overloaded function for convenience
	template<typename Target>
	inline Target lexical_cast_scalar_to_string(const char*& arg)
	{
		return lexical_cast_scalar_to_string<Target>(std::string(arg));
	}

	
	//! General case of lexical cast, use boost
	template<typename Target, typename Source>
	inline Target lexical_cast(const Source& arg)
	{
		return boost::lexical_cast<Target>(arg);
	}

    //! Special case of lexical cast to std::vector<T>
    template<typename T>
    inline std::vector<T> lexical_cast_vector(const std::string& arg)
    {
        std::string cleanedInput = arg;
        if (cleanedInput.find('[') != 0)
        {
            throw std::runtime_error("Vector parameter '" + arg + "' must start with '['");
        }
        if (cleanedInput.find(']') != cleanedInput.size()-1)
        {
            throw std::runtime_error("Vector parameter '" + arg + "' must end with ']'");
        }
        cleanedInput.erase(std::remove(cleanedInput.begin(), cleanedInput.end(), '['), cleanedInput.end());
        cleanedInput.erase(std::remove(cleanedInput.begin(), cleanedInput.end(), ' '), cleanedInput.end());
        cleanedInput.erase(std::remove(cleanedInput.begin(), cleanedInput.end(), ']'), cleanedInput.end());

        // Split the string into a vector of strings using commas as the delimiter
        std::vector<std::string> tokens;
        boost::algorithm::split(tokens, cleanedInput, boost::algorithm::is_any_of(","));

        std::vector<T> result;
        result.reserve(tokens.size());
        for(const auto& token: tokens)
        {
            auto value = lexical_cast_scalar_to_string<T>(token);
            result.push_back(value);
        }
        return result;
    }
	//! Special case of lexical cast to float, use lexical_cast_scalar_to_string
	template<>
	inline float lexical_cast(const std::string& arg) { return lexical_cast_scalar_to_string<float>(arg); }
	//! Special case of lexical cast to float, use lexical_cast_scalar_to_string
	template<>
	inline double lexical_cast(const std::string& arg) { return lexical_cast_scalar_to_string<double>(arg); }
	
	//
	
	//! Return the string value using lexical_cast
	template<typename S>
	inline std::string toParam(const S& value)
	{
		return lexical_cast<std::string>(value);
	}

    //! Return the string value of a std::vector<T>
    template<typename T>
    inline std::string toParam(const std::vector<T>& input)
    {
        std::ostringstream oss;
        oss << "[";
        std::copy(input.begin(), input.end() - 1,
                  std::ostream_iterator<T>(oss, ", "));
        oss << input.back() << "]";
        return oss.str();
    }
	
	//! The superclass of classes that are constructed using generic parameters. This class provides the parameter storage and fetching mechanism
	struct Parametrizable
	{
		//! An exception thrown when one tries to fetch the value of an unexisting parameter
		struct InvalidParameter: std::runtime_error
		{
			InvalidParameter(const std::string& reason);
		};
		
		//! A function that returns whether a is smaller than b
		typedef bool(*LexicalComparison)(std::string a, std::string b);
		
		//! Return whether a < b, lexically casted to S
		template<typename S>
		static bool Comp(std::string a, std::string b)
		{
			return lexical_cast<S>(a) < lexical_cast<S>(b);
		}
		
		//! The documentation of a parameter
		struct ParameterDoc
		{
			std::string name; //!< name
			std::string doc; //!< short documentation
			std::string defaultValue; //!< default value
			std::string minValue; //!< if bounds are checked, minimum value
			std::string maxValue; //!< if bounds are checked, maxmimu value
			LexicalComparison comp; //!< pointer to comparison function
			
			/*
			This code is beautiful, this code is correct, this code does not work ;-(
			Blame gcc bug 9050 (http://gcc.gnu.org/bugzilla/show_bug.cgi?id=9050), shame
			on them forever and beyond. People being laaaazzzy adopters, I'm forced to use
			something that works on gcc 4.4.
			
			template<typename S>
			ParameterDoc(const std::string& name, const std::string& doc, const S defaultValue, const S minValue, const S maxValue = std::numeric_limits<S>::max());
			template<typename S>
			ParameterDoc(const std::string& name, const std::string& doc, const S defaultValue);
			*/
			ParameterDoc(const std::string& name, const std::string& doc, const std::string& defaultValue, const std::string& minValue, const std::string& maxValue, LexicalComparison comp);
			ParameterDoc(const std::string& name, const std::string& doc, const std::string& defaultValue);
			
			friend std::ostream& operator<< (std::ostream& o, const ParameterDoc& p);
		};
		
		//! The documentation of all parameters
		typedef std::vector<ParameterDoc> ParametersDoc;
		
		/*
		Again, not used because fo gcc bug 9050
		struct Parameter: public std::string
		{
			template<typename S>
			Parameter(const S value);
			Parameter(){}
		};
		*/
		typedef std::string Parameter; //!< alias
		typedef std::map<std::string, Parameter> Parameters; //!< Parameters stored as a map of string->string
		typedef std::set<std::string> ParametersUsed; //!< Parameters whose value has been read
		
		const std::string className; //!< name of the class
		const ParametersDoc parametersDoc; //!< documentation of parameters
		Parameters parameters; //!< parameters with their values encoded in string
		ParametersUsed parametersUsed; //!< parameters whose value has actually been read
		
		Parametrizable();
		Parametrizable(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Parametrizable();
		
		std::string getParamValueString(const std::string& paramName);
		
		//! Return the value of paramName, lexically-casted to S
		template<typename S>
		S get(const std::string& paramName) { return lexical_cast<S>(getParamValueString(paramName)); }

		//! Return the value of paramName, lexically-casted std::vector<S>
		template<typename T>
		inline std::vector<T> getVector(const std::string& paramName) {return lexical_cast_vector<T>(getParamValueString(paramName)); }

		friend std::ostream& operator<< (std::ostream& o, const Parametrizable& p);
	};
	std::ostream& operator<< (std::ostream& o, const Parametrizable::ParametersDoc& p);
} // namespace PointMatcherSupport

#endif // __POINTMATCHER_PARAMETRIZABLE_H
