#!/usr/bin/env python

import yaml
import os.path
import sys
import re
import csv
from numpy import *
import matplotlib.pyplot as ptl
from scipy import stats

def get_T_matrix(row, name):
	T = matrix(eye(4))
	for x in range(0,4):
		for y in range(0,4):
			T[x,y] = float(row[name+str(x)+str(y)])
	return T

def extract_results_vs_param(name, resultsFileName, paramFileName):
	
	csvResults = csv.DictReader(open(resultsFileName), skipinitialspace=True)
	csvParam = csv.DictReader(open(paramFileName), skipinitialspace=True)
	
	valueByParam = dict()
	for resRow, paramRow in zip(csvResults, csvParam):
		value = float(resRow[name])
		#value = 1/(float(resRow[name])+0.00001)
		param = float(paramRow.values()[0])

		if valueByParam.has_key(param):
			valueByParam[param] = append(valueByParam[param], value)
		else:
			valueByParam[param] = array([value])

	x = list()
	y_med = list()
	y_lowerBound = list()
	y_higherBound = list()
	for valueGroup in sorted(valueByParam.iterkeys()):
		x.append(valueGroup)
		y_med.append(stats.scoreatpercentile(valueByParam[valueGroup],50))
		y_lowerBound.append(stats.scoreatpercentile(valueByParam[valueGroup],10))
		y_higherBound.append(stats.scoreatpercentile(valueByParam[valueGroup],90))
	
	return x, y_med, [y_lowerBound, y_higherBound]

def extract_results_vs_scanID(name, resultsFileName, setupFileName):
	
	csvResults = csv.DictReader(open(resultsFileName), skipinitialspace=True)
	csvSetup = csv.DictReader(open(setupFileName), skipinitialspace=True)
	
	valueByScanId = dict()
	for resRow, setupRow in zip(csvResults, csvSetup):
		value = float(resRow[name])
		readingName = setupRow['reading']
		readingId = int(filter(lambda x: x.isdigit(), readingName))

		if valueByScanId.has_key(readingId):
			valueByScanId[readingId] = append(valueByScanId[readingId], value)
		else:
			valueByScanId[readingId] = array([value])

	x = list()
	y_med = list()
	y_lowerBound = list()
	y_higherBound = list()
	for valueGroup in sorted(valueByScanId.iterkeys()):
		x.append(valueGroup)
		y_med.append(stats.scoreatpercentile(valueByScanId[valueGroup],50))
		y_lowerBound.append(stats.scoreatpercentile(valueByScanId[valueGroup],10))
		y_higherBound.append(stats.scoreatpercentile(valueByScanId[valueGroup],90))
	
	return x, y_med, [y_lowerBound, y_higherBound]

def main():
	
	if len(sys.argv) != 4:
		sys.exit("\nUsage error: " + sys.argv[0] + " resultsFileName.csv setupFileName.csv configTableFileName.csv\n")

	resultsFileName = sys.argv[1]
	setupFileName = sys.argv[2]
	paramFileName = sys.argv[3]

		
	ptl.figure()
	ptl.subplot(221)
	x, y, y_err = extract_results_vs_param('e_trans_mean', resultsFileName, paramFileName)
	ptl.plot(x, y, 'ko-')
	ptl.xlabel('Parameter')
	ptl.ylabel('Translation Error')
	
	ptl.subplot(222)
	x, y, y_err = extract_results_vs_param('e_rot_mean', resultsFileName, paramFileName)
	ptl.plot(x, y, 'ko-')
	ptl.xlabel('Parameter')
	ptl.ylabel('Rotation Error')

	ptl.subplot(223)
	x, y, y_err = extract_results_vs_param('ConvergenceDuration_mean', resultsFileName, paramFileName)
	ptl.plot(x, y, 'ko-')
	ptl.xlabel('Parameter')
	ptl.ylabel('Time')

	ptl.subplot(224)
	x, y, y_err = extract_results_vs_param('failed_mean', resultsFileName, paramFileName)
	ptl.plot(x, y, 'ko-')
	ptl.xlabel('Parameter')
	ptl.ylabel('Nb of failure')


	#x, y, y_err = extract_results_vs_param('ConvergenceDuration_mean', csvResults, csvParam)
	#x, y, y_err = extract_results_vs_param('e_rot_mean', csvResults, csvParam)

	#plot(x, y, 'ko-')

	#x, y, y_err = extract_results_vs_scanID('e_trans_mean', csvResults, csvSetup)
	#x, y, y_err = extract_results_vs_scanID('ConvergenceDuration_mean', csvResults, csvSetup)
	
	#plt.figure()
	#plt.errorbar(x, y, yerr=y_err, fmt='o') 
	ptl.show()

if __name__=="__main__":
	main()

