#include "PointMatcher.h"

#include <yaml-cpp/yaml.h>

namespace PointMatcherSupport
{
	void getNameParamsFromYAML(const YAML::Node& module, std::string& name, Parametrizable::Parameters& params)
	{
		if (module.size() != 1)
		{
			// parameter-less entry
			name = module.as<std::string>();
		}
		else
		{
			// get parameters
			YAML::const_iterator mapIt(module.begin());
			name = mapIt->first.as<std::string>();
			for(YAML::const_iterator paramIt = mapIt->second.begin(); paramIt != mapIt->second.end(); ++paramIt)
			{
				std::string key = paramIt->first.as<std::string>();
                if (paramIt->second.IsSequence())
                {
                    std::ostringstream oss;
                    oss << "[";
                    for(int i = 0; i < paramIt->second.size()-1; ++i)
                    {
                        oss << paramIt->second[i] << ", ";
                    }
                    oss << paramIt->second[paramIt->second.size()-1] << "]";
                    params[key] = oss.str();
                }
                else
                {
                    params[key] = paramIt->second.as<std::string>();
                }
			}
		}
	}
}
