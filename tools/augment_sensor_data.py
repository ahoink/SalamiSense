#! /usr/bin/python3

import numpy as np
import copy
import random

# Load matrix from csv
def loadCSV(path) :
    X = []
    with open(path) as csv:
        data = csv.readlines()
        for row in data:
            srow = row.split(",")
            X.append([float(i) for i in srow])
        return X


def scale_occupancy(data, factor) :
    for i in range(len(data)) :
        data[i][0] = data[i][0] * factor

    return data

def rand_noise(data) :
    random.seed(0)
    for i in range(len(data)) :
        rand = random.randint(-2, 1)
        data[i][0] += rand
        if data[i][0] < 0 :
            data[i][0] = 0

    return data

def save_model(data, path) :
    f = open(path, 'w')
    for r in data :
        f.write(','.join([str(x) for x in r]) + '\n')
    f.close()


def main() :
    data = loadCSV("training_set.csv")
    data = scale_occupancy(data,2)
    data = rand_noise(data)
    save_model(data, "noisy_sensor_training_set.csv")

main()



        

