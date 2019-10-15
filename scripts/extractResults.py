#!/usr/bin/env python
# Takes in filename of a results csv, analyses the data
# and prints out results based on arguments

# Created: 15/10/19 - Andrew Bui
# Last Modified: 15/10/19 - Andrew Bui

import argparse
import csv

# Creates parser to read arguments
def create_parser():
	parser = argparse.ArgumentParser()

	# Specify arguments
	parser.add_argument('-f', '--filename')
	parser.add_argument('-s', '--summary',action='store_true')
	parser.add_argument('-d', '--detailed', action='store_true')

	return parser

# Outputs the number of onjects that were found during yolo run
def object_count(csvFile):

	objectsDetected = 0
	
	with csvFile:
	# Skip first row if there is a header
		has_header = csv.Sniffer().has_header(csvFile.read(1024))
		csvFile.seek(0)
		csvReader = csv.reader(csvFile)
		if had_header:
			next(csvReader)

		for line in csvReader:
			objectsDetected += int(line[1])
	
		print('Number of objects detected: {}\n'.format(objectsDetected))


# Outputs average probably and count of each object type
def detailed_breakdown(csvFile):
	allObjects = {}
	averageProbability = {}

	# read in csv file
	with csvFile:
		# Skip first row if there is a header
		has_header = csv.Sniffer().has_header(csvFile.read(1024))
		csvFile.seek(0)
		csvReader = csv.reader(csvFile)
		if had_header:
			next(csvReader)
		
		for line in csvReader:
			# Appends probability instance if object already exists
			if line[1] in objects:
				objects[line[1]].append(float(line[2]))
			# Otherwise add new object type
			else:
				objects[line[1]] = [float(sequence[2])]

	# Determine average and count for each object type
	for key, value in objects.items():
		average = round(sum(value)/len(value),4)
		averageProbability[key] = {
			"average": average,
			"count": len(value)
		}
	
	for key, value in sorted(averageProbability.items()):
		print(key, value)


if __name__ == "__main__":
	OUTBOUNDS_RESULTS = '../output_bounds/'
	
	# Create parser
	parser = create_parser()
	args = parser.parse_args()

	# Check that filename has been specified
	if args.filename:
		filename = OUTBOUNDS_RESULTS + args.filename

		# Check if file cann be read, exit if it cannot
		try:
			csvFile = open(filename, mode='r')
		except IOError:
			print('Could not read file:{}'.format(filename))
			exit(1)

		# Report based on type of csv is read
		if args.detailed:
			detailed_breakdown(csvFile)
		elif args.summary:
			object_count(csvFile)
	else:
		print('Filename not specified\n')
		exit(1)
