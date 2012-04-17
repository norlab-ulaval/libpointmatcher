#!/usr/bin/env python

import yaml
import os.path
import sys
import numpy as np


def main():

	if len(sys.argv) < 8 or len(sys.argv) > 9:
		sys.exit("\nUsage error: " + sys.argv[0] + " template.yaml sectionName moduleName paramName paramMin paramStep paramMax <int>\n")

	yamlFileName = sys.argv[1]
	yamlFile = open(sys.argv[1])
	document = yaml.load(yamlFile)
	yamlFile.close()
	
	sectionName = sys.argv[2]
	moduleName = sys.argv[3]
	paramName = sys.argv[4]
	paramMin = float(sys.argv[5])
	paramStep = float(sys.argv[6])
	paramMax = float(sys.argv[7])
	if len(sys.argv) == 9:
		dataType = sys.argv[8] # only support int or float
	else:
		dataType = "float"

	

	for p in np.arange(paramMin, paramMax, paramStep):
		if dataType == "int":
			p = int(p)
		else:
			p = float(p)
		for section in document:
			if section == sectionName:
				for module in document[sectionName]:
					if type(module) is dict:
						if module.has_key(moduleName):
							for param in module[moduleName]:
								if param == paramName:
									module[moduleName][paramName] = p
					else:
						if module == moduleName:
							for param in document[sectionName][moduleName]:
								if param == paramName:
									document[sectionName][moduleName][paramName] = p
		
		newFileName = os.path.splitext(yamlFileName)[0]
		newFileName = newFileName + "_" + paramName
		newFileName = newFileName + str(p).replace(".", "-") + ".yaml"
		stream = file(newFileName, 'w')
		yaml.dump(document, stream, default_flow_style=False)


if __name__=="__main__":
	main()

