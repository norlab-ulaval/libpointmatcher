#!/usr/bin/env python

import yaml
import os.path
import sys
import re
import csv
from numpy import *

from generate_experiment import *

def get_yaml_value(document, sectionName, moduleName, paramName):
	for section in document:
			if section == sectionName:
				for module in document[sectionName]:
					if type(module) is dict:
						if module.has_key(moduleName):
							for param in module[moduleName]:
								if param == paramName:
									return str(module[moduleName][paramName])
					else:
						if module == moduleName:
							for param in document[sectionName][moduleName]:
								if param == paramName:
									return str(document[sectionName][moduleName][paramName])
	return str()

def main():

	if len(sys.argv) != 6:
		sys.exit("\nUsage error: " + sys.argv[0] + " setupFileName.csv path/to/configFiles/ sectionName moduleName paramName\n")

	setupFileName = sys.argv[1]
	pathToConfig = sys.argv[2]
	sectionName = sys.argv[3]
	moduleName = sys.argv[4]
	paramName = sys.argv[5]
	csvOutName = 'configTable_XXX.csv'
	
	configFile = open(csvOutName, 'w')
	csvConfig = csv.writer(configFile, lineterminator='\n')
	header = sectionName+'-'+moduleName+'-'+paramName
	csvConfig.writerow([header])
	
	# Load csv file with experimental setup
	print "Loading ICP config files..."
	csvSetup = csv.DictReader(open(setupFileName), skipinitialspace=True)
	for row in csvSetup:
		yamlFile = open(pathToConfig+'/'+row['config'])
		document = yaml.load(yamlFile)
		yamlFile.close()
		value = get_yaml_value(document, sectionName, moduleName, paramName)
		csvConfig.writerow([value])

	print "Done!"
	print "Results in "+csvOutName

if __name__=="__main__":
	main()

