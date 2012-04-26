#!/usr/bin/env python

import yaml
import os.path
import sys
import re
import csv
from numpy import *
import matplotlib.pyplot as ptl
from scipy import stats
from generate_experiment import *

def extract_stats_per_xValue(xArray, yArray):
	
	valueByX = dict()
	for x, y in zip(xArray, yArray):
		if valueByX.has_key(x):
			valueByX[x] = append(valueByX[x], y)
		else:
			valueByX[x] = array([y])

	x = array([])
	y_med = array([])
	y_lb = array([]) # lower bound
	y_hb = array([]) # higher bound
	for valueGroup in sorted(valueByX.iterkeys()):
		x = append(x, valueGroup)
		y_med = append(y_med, stats.scoreatpercentile(valueByX[valueGroup],50))
		y_lb = append(y_lb, stats.scoreatpercentile(valueByX[valueGroup],10))
		y_hb = append(y_hb, stats.scoreatpercentile(valueByX[valueGroup],90))
	
	return x, y_med, [y_lb, y_hb]

def main():
	
	if len(sys.argv) < 3:
		sys.exit("\nUsage error: " + sys.argv[0] + " resolution resultsFileName1.csv resultsFileName2.csv ... \n")

	nbGraphPoints = float(sys.argv[1])

	styleList = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
	errorTransPlot = list()
	errorRotPlot = list()

	maxTransAccuracy = 0
	maxRotAccuracy = 0
	for i in range(2, len(sys.argv)):
		resultsFileName = sys.argv[i]
		
		csvResults = csv.DictReader(open(resultsFileName), skipinitialspace=True)
		
		errorTransList = array([])
		errorRotList = array([])
		for resRow in csvResults:
			err = float(resRow['e_trans_mean'])
			errorTransList = append(errorTransList, err)
			err = float(resRow['e_rot_mean'])
			errorRotList = append(errorRotList, err)
	
		errorTransPlot.append(errorTransList)
		errorRotPlot.append(errorRotList)
		maxTransAccuracy = max(maxTransAccuracy, errorTransList.max())
		maxRotAccuracy = max(maxRotAccuracy, errorRotList.max())

	ptl.figure()
	ax1 = ptl.subplot(121)
	ax2 = ptl.subplot(122)
	
	maxCount = len(errorTransList)
	accuracyTrans = linspace(0, maxTransAccuracy, num=nbGraphPoints)
	accuracyRot = linspace(0, maxRotAccuracy, num=nbGraphPoints)
	plotId = 0
	for errorTransList, errorRotList  in zip(errorTransPlot, errorRotPlot):
			
		recall_errT = array([])
		recall_errR = array([])
		for accuT, accuR in zip(accuracyTrans, accuracyRot):
			countT = 0
			countR = 0
			for et, er in zip(errorTransList, errorRotList):
				if et < accuT:
					countT = countT + 1
				if er < accuR:
					countR = countR + 1

			recall_errT = append(recall_errT,countT/float(maxCount))
			recall_errR = append(recall_errR,countR/float(maxCount))
	
	
		ax1.plot(accuracyTrans, recall_errT, styleList[plotId], label='arg'+str(plotId))
		ax2.plot(accuracyRot, recall_errR, styleList[plotId], label='arg'+str(plotId))
		plotId = plotId + 1

	ax1.legend(loc=4)
	ax1.set_xlabel('Accuracy threshold (m)')
	ax1.set_ylabel('Cumulative distribution')
	ax2.set_xlabel('Accuracy threshold (rad)')

	ptl.show()

if __name__=="__main__":
	main()

