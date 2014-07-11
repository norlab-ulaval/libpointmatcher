#include "yaml-cpp-pm/null.h"
#include "yaml-cpp-pm/node.h"

namespace YAML
{
	_Null Null;

	bool IsNull(const Node& node)
	{
		return node.Read(Null);
	}
}
