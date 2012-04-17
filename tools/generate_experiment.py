#!/usr/bin/env python

import yaml
import os.path
import sys
import re
import csv
from numpy import *

def tryint(s):
		try:
			return int(s)
		except:
			return s

def alphanum_key(s):
		""" Turn a string into a list of string and number chunks.
				"z23a" -> ["z", 23, "a"]
		"""
		return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def add_T_header(l, name):
		for x in range(0,4):
			for y in range(0,4):
				l.append(name+str(x)+str(y))
		return l

def add_T_values(l, T):
		for x in range(0,4):
			for y in range(0,4):
				l.append("%.6f" % (T[x,y]))
		return l

def main():

	if len(sys.argv) < 7 or len(sys.argv) > 9:
		sys.exit("\nUsage error: " + sys.argv[0] + " /local/PointCloud/path namePattern /icp/Config/path namePattern groundTruthFileName.csv posePerturbationsFileName.csv <steps> <direction>\n")

	pointCloudPath = sys.argv[1]
	pointCloudPattern = sys.argv[2]
	icpConfigPath = sys.argv[3]
	icpConfigPattern = sys.argv[4]
	groundTruthFileName = sys.argv[5]
	posePerturbationsFileName = sys.argv[6]
	steps = 1
	if len(sys.argv) > 7:
		steps = int(sys.argv[7])
	direction = "forward"
	if len(sys.argv) > 8:
		direction = argv[8]

	# Handle a sorted list of files
	files = os.listdir(pointCloudPath)
	pointCloudFiles = [k for k in files if pointCloudPattern in k]
	pointCloudFiles.sort(key=alphanum_key)

	# Handle list of unsorted files
	files = os.listdir(icpConfigPath)
	icpConfigFiles = [k for k in files if icpConfigPattern in k]

	# Load csv file with ground truth
	csvGroungTruth = csv.DictReader(open(groundTruthFileName), skipinitialspace=True)
	truthTransformList = list()
	for row in csvGroungTruth:
		T = matrix(eye(4))
		for x in range(0,4):
			for y in range(0,4):
				T[x,y] = float(row['T'+str(x)+str(y)])
		truthTransformList.append(T)

	if len(pointCloudFiles) != len(truthTransformList):
		sys.exit('ERROR: number of point cloud file ('+len(pointCloudFiles)+') is different than number of ground truth transformations ('+len(truthTransformList)+')')

	# Load csv file with perturbation poses
	csvPerturbations = csv.DictReader(open(posePerturbationsFileName), skipinitialspace=True)
	perturbationList = list()
	for row in csvPerturbations:
		T = matrix(eye(4))
		for x in range(0,4):
			for y in range(0,4):
				T[x,y] = float(row['iT'+str(x)+str(y)])
		perturbationList.append(T)

	# Build the final csv file with all the combined information
	expFile = open('experiment_XXX.csv', 'w')
	csvExp = csv.writer(expFile, lineterminator='\n')
	header = ['reading', 'reference', 'config']
	header = add_T_header(header, 'iT')
	header = add_T_header(header, 'gT')
	csvExp.writerow(header)
	for i in range(0,len(pointCloudFiles)):
		for config in icpConfigFiles:
			for pT in perturbationList:
				readId = i+steps
				refId = i
				if readId < len(pointCloudFiles):
					rowText = [pointCloudFiles[readId], pointCloudFiles[refId], config]
					gT = linalg.inv(truthTransformList[refId]) * truthTransformList[readId]
					iT = pT * gT
					rowText = add_T_values(rowText, iT)
					rowText = add_T_values(rowText, gT)
					csvExp.writerow(rowText)


	print "\n" + str(len(pointCloudFiles)*len(icpConfigFiles)*len(perturbationList)) + " experiments generated in experiement_XXX.csv\n"
	expFile.close()

if __name__=="__main__":
	main()

