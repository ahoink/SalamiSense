import numpy as np
import copy

# Start "Private" stuff, don't call these plz
# get mean and std of each col
def getMeanStd(data):
	ret = []
	for i in range(len(data[0]) - 1):
		col = [row[i] for row in data]
		mean = np.mean(col)
		std = np.std(col)
		ret.append([mean, std])
	return ret

# Standardizes the data in a matrix (except the last column)
def standardize(data, meanStd, bonus):
	for i in range(len(data[0]) - bonus):
		col = [row[i] for row in data]
		for row in data:
			row[i] -= meanStd[i][0]
			row[i] /= meanStd[i][1]

# Perform linear regression on data
def linRegress(data):
	X = [r[:-1] for r in data]
	Y = [[r[-1]] for r in data]
	for r in X:
		r.insert(0, 1)
	X = np.array(X)
	Y = np.array(Y)
	theta = np.matmul(X.T, X)
	theta = np.linalg.inv(theta)
	theta = np.matmul(theta, X.T)
	theta = np.matmul(theta, Y)
	return theta

# calculates root mean squared error
def RMSE(expect, actual):
	rmse = 0
	for i in range(len(expect)):
		rmse += (expect[i][0] - actual[i][0])**2
	rmse /= len(expect)
	return rmse**0.5
# end "Private"


# Load matrix from csv
def loadCSV(path):
	X = []
	with open(path) as csv:
		data = csv.readlines()
		for row in data:
			srow = row.split(",")
			X.append([float(i) for i in srow])
		return X

# get model from test data matrix
def getModel(data):
	meanStd = getMeanStd(data)
	standardize(data, meanStd, 1)
	theta = linRegress(data)
	return theta, meanStd

# save the model to path as csv
def saveModel(theta, meanStd, path):
	csv = []
	csv.append([i for row in meanStd for i in row])
	csv.append([i for row in theta for i in row])
	f = open(path, 'w')
	for r in csv:
		f.write(','.join([str(x) for x in r]) + '\n')
	f.close()

# load model from csv
def loadModel(path):
	csv = loadCSV(path)
	meanStd = []
	theta = [[v] for v in csv[1]]

	for i in range(0, len(csv[0]), 2):
		meanStd.append([csv[0][i], csv[0][i+1]])
	return theta, meanStd

# run a vector through model
def runThroughModel(vector, theta, meanStd):
	vector = vector[:]
	m = []
	m.append(vector)
	standardize(m, meanStd, 0)
	for r in m:
		r.insert(0, 1)

	result = np.matmul(m, theta)
	return result[0][0]





