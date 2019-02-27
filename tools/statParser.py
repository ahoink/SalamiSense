#!/usr/bin/python3

import argparse
import csv
import copy


def parseRAM(stat):
	ramKey = stat.index("RAM")
	usage = stat[ramKey+1]
	usage = usage.split("/")
	current = usage[0]
	maxMem = usage[1][:-2]

	return current


def parseCPU(stat, coreList, dataList):
	cpuKey = stat.index("CPU")
	util = stat[cpuKey+1]
	util = util[1:-1]
	util = util.split(",")
	cores = [core.split("%@") for core in util]

	dataList.extend(cores)


def parseFreqUtil(stat, dev):
	devKey = stat.index(dev.upper())
	usage = stat[devKey+1]
	usage = usage.split("%@")
	util = usage[0]
	freq = usage[1]
	
	return util


def parseTemp(stat, dev):
	tempList = [x[:-1] for i, x in enumerate(stat) if (x[-1].lower() == "c") and ("@" in x)]
	for i in tempList:
		if i.split("@")[0].lower() == dev.lower():
			return i.split("@")[1]

	
def parsePower(stat, domain):
	dev = stat.index(domain.upper())
	power = stat[dev+1]
	power = power.split("/")
	current = power[0]
	avg = power[1]

	return current


def parseAllStats(stat, dataList, powDom, tempDom, coreList):
	parseCPU(stat, coreList, dataList)
	for dom in powDom:
		dataList.append(parsePower(stat, dom))
	
	for temp in tempDom:
		dataList.append(parseTemp(stat, temp))
	
	dataList.append(parseRAM(stat))
	dataList.append(parseFreqUtil(stat, "EMC_FREQ"))
	dataList.append(parseFreqUtil(stat, "GR3D_FREQ"))
			

def writeCsv(fileName, data):
	with open(fileName, "a+") as f:
		csvWriter = csv.writer(f, delimiter=",")
		wr = [i for j in data[:6] for i in j]
		wr += data[6:]
		csvWriter.writerow(wr)


def main():
	parser = argparse.ArgumentParser(description="Jetson CPU profiler parser")
	parser.add_argument("--time", help=".log file for CPU processing time")
	parser.add_argument("--stats", help=".log file for tegrastats")
	args = parser.parse_args()

	timeFile = args.time
	statFile = args.stats
	if timeFile and statFile:
		freqkHz = timeFile.split("_")[-2]
		csvFile = "tools/cpufreq_logs/cpufreq_" + freqkHz + ".csv"
		
		numCores = 6
		cpuCores = ["CPU"+str(i) for i in range(numCores)]
		cpuHdr = []
		for i in range(numCores):
			tmp = ["CPU%d %%" % i, "CPU%d freq" % i]
			cpuHdr.extend(tmp)

		powDomains = ["VDD_IN","VDD_CPU","VDD_GPU","VDD_SOC","VDD_WIFI","VDD_DDR"]
		tempDomains = ["BCPU","MCPU","GPU","PLL","Tboard","Tdiode","PMIC","thermal"]
		periph = ["RAM", "EMC_FREQ", "GR3D_FREQ"]
		
		times = []
		print("opening timing log...", end=" ")
		with open(timeFile, 'r') as t:
			times.append(t.readline())

		print("done")

		csvHeader = cpuHdr + powDomains + tempDomains + periph
			
		with open(csvFile, "w") as f:
			csvWriter = csv.writer(f, delimiter=",")
			csvWriter.writerow(csvHeader)

		print("opening tegrastats log...", end=" ")
		s = open(statFile, 'r')
		for line in s:
			data = []
			line = line[:-1].split(" ")
			parseAllStats(line, data, powDomains, tempDomains, cpuCores)
			writeCsv(csvFile, data)
		
		s.close()
		print("done")
			
	else:
		parser.print_help()

if __name__ == '__main__':
	main()
