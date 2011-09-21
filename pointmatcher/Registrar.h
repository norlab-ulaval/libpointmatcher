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

#ifndef __POINTMATCHER_REGISTRAR_H
#define __POINTMATCHER_REGISTRAR_H

#include "Parametrizable.h"
#include <boost/format.hpp>

namespace PointMatcherSupport
{
	template<typename Interface>
	struct Registrar
	{
	public:
		struct ClassDescriptor
		{
			virtual ~ClassDescriptor() {}
			virtual Interface* createInstance(const Parametrizable::Parameters& params) const = 0;
			virtual const std::string description() const = 0;
			virtual const Parametrizable::ParametersDoc availableParameters() const = 0;
		};
		
		template<typename C>
		struct GenericClassDescriptor: public ClassDescriptor
		{
			virtual Interface* createInstance(const Parametrizable::Parameters& params) const
			{
				return new C(params);
			}
			virtual const std::string description() const
			{
				return C::description();
			}
			virtual const Parametrizable::ParametersDoc availableParameters() const
			{
				return C::availableParameters();
			}
		};
		
		template<typename C>
		struct GenericClassDescriptorNoParam: public ClassDescriptor
		{
			virtual Interface* createInstance(const Parametrizable::Parameters& params) const
			{
				return new C();
			}
			virtual const std::string description() const
			{
				return C::description();
			}
			virtual const Parametrizable::ParametersDoc availableParameters() const
			{
				return {};
			}
		};
		
	protected:
		typedef std::map<std::string, ClassDescriptor*> DescriptorMap;
		DescriptorMap classes;
		
	public:
		//! Destructor, remove all classes descriptors 
		~Registrar()
		{
			for (auto it = classes.begin(); it != classes.end(); ++it)
				delete it->second;
		}
		//! Register a class by storing an instance of a descriptor helper class
		void reg(const std::string &name, ClassDescriptor* descriptor)
		{
			classes[name] = descriptor;
		}
		
		//! Return a descriptor following a name
		ClassDescriptor* getDescriptor(const std::string& name)
		{
			auto it = classes.find(name);
			if (it == classes.end())
			{
				std::cerr << "No element named " << name << " is registered. Known ones are:\n";
				dump(std::cerr);
				throw std::runtime_error((boost::format("Trying to instanciate unknown element %1 from registrar") % name).str());
			}
			return it->second;
		}
		
		//! Create an instance
		Interface* create(const std::string& name, const Parametrizable::Parameters& params)
		{
			return getDescriptor(name)->createInstance(params);
		}
		
		//! Get the description of a class
		std::string getDescription(const std::string& name)
		{
			return getDescriptor(name)->description();
		}
		
		//! Print the list of registered classes to stream
		void dump(std::ostream &stream)
		{
			for (auto it = classes.begin(); it != classes.end(); ++it)
				stream << "- " << it->first << "\n";
		}
		
		//! begin for const iterator over classes descriptions
		typename DescriptorMap::const_iterator begin() const
		{
			return classes.begin();
		}
		//! end for const iterator over classes descriptions
		typename DescriptorMap::const_iterator end() const
		{
			return classes.end();
		}
	};

	#define REG(name) name##Registrar
	#define DEF_REGISTRAR(name) PointMatcherSupport::Registrar< name > name##Registrar;
	#define ADD_TO_REGISTRAR(name, element) { \
		typedef typename PointMatcherSupport::Registrar< name >::template GenericClassDescriptor< element > Desc; \
		name##Registrar.reg(# element, new Desc() ); \
	}
	#define ADD_TO_REGISTRAR_NO_PARAM(name, element) { \
		typedef typename PointMatcherSupport::Registrar< name >::template GenericClassDescriptorNoParam< element > Desc; \
		name##Registrar.reg(# element, new Desc() ); \
	}
} // namespace PointMatcherSupport

#endif // __POINTMATCHER_REGISTRAR_H
