import cv2
import numpy as np
import time
import threading
import json
import pickle

import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
import pysrc.Gouda as Gouda
from pysrc.SalamiUtils import NodeData, NewConnHandler, BoxThread
import pysrc.VectorUtils as vu
import pysrc.Peeps as Peeps
import pysrc.Regression as Reg

import keras
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import tensorflow as tf

from PIL import Image, ImageTk

keyPress = None
showConf = False
gui = None
nindex = -1
clients = []
connHandler = None
serv = None

def get_session():
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    return tf.Session(config=config)


def getBoxStats(dimg, box):
	depthSum = 0
	depthSqSum = 0
	n = (box[3] - box[1]) * (box[2] - box[0])
	for y in range(box[1], box[3]):
		for x in range(box[0], box[2]):
			depthSum += dimg[y][x]
			depthSqSum += dimg[y][x]**2
	
	mean = depthSum / n
	var = depthSqSum / n - mean**2
	return np.sqrt(var)

def controlHVAC(config, crowded, tempr):
	# Room Occupied
	if crowded > 0:
		# Set to Cooling
		if config["Mode"] == "Cool":
			if tempr > config["Sensors"]["MaxOccupTemp"]:
				return ("AC ON FULL BLAST")
			elif tempr > config["Sensors"]["TargetOccupTemp"]:
				if crowded > 80:
					return ("AC ON HIGH")
				elif crowded > 40:
					return ("AC ON MEDIUM")
				else:
					return ("AC ON LOW")
			else:
				return ("AC OFF")
		# Set to Heating
		elif config["Mode"] == "Heat":
			if tempr < config["Sensors"]["MinOccupTemp"]:
				return ("HEAT ON FULL BLAST")
			elif tempr < config["Sensors"]["TargetOccupTemp"]:
				if crowded > 80:
					return ("HEAT ON LOW")
				elif crowded > 40:
					return ("HEAT ON MEDIUM")
				else:
					return ("HEAT ON HIGH")
			else:
				return ("HEAT OFF")
	# Room Vacant
	else:
		# Set to Cooling
		if config["Mode"] == "Cool":
			if tempr > config["Sensors"]["MaxVacantTemp"]:
				return ("AC ON HIGH")
			elif tempr > config["Sensors"]["TargetVacantTemp"]:
				return ("AC ON LOW")
			else:
				return ("AC OFF")
		# Set to Heating
		elif config["Mode"] == "Heat":
			if tempr < config["Sensors"]["MinVacantTemp"]:
				return ("HEAT ON HIGH")
			elif tempr < config["Sensors"]["TargetVacantTemp"]:
				return ("HEAT ON LOW")
			else:
				return ("HEAT OFF")


# --- GUI FUNCTIONS --- #
def exitButtonPress():
	global keyPress
	keyPress = ord('q')

def confButtonPress():
	global showConf
	global gui
	if showConf:
		showConf = False
		gui.setButtonText("confidence", "Confidence\nOff")
	else:
		showConf = True
		gui.setButtonText("confidence", "Confidence\nOn")

def newNodeButton():
	global connHandler
	global serv
	connHandler.newClient(serv)

def selNodeButton():
	global nindex
	nindex = (nindex + 1) % len(clients)

def delNodeButton():
	global clients
	global nindex
	global serv
	if nindex >= 0:
		serv.dropClient(clients[nindex].getClient())
		clients.pop(nindex)
		if clients:
			nindex = (nindex - 1) % len(clients)
		else:
			nindex = -1

Calibrating = False
CalIndex = 0
def calibrator(x, fx=0, fy=0):
	global Calibrating
	global CalIndex
	global clients
	global nindex
	if not Calibrating:
		if x is None:
			nindex = 0
			Calibrating = True
	else:
		if x is not None and len(x) == 3:
			print("Calibration point %d:" % (CalIndex))
			print("%f,%f,%f" % (x[0],x[1],x[2]))
			clients[nindex].setRefPoint(CalIndex, x[0], x[1], x[2], fx, fy)
		nindex = (nindex + 1) % len(clients)
		if nindex == 0:
			CalIndex += 1
		if CalIndex >= 3:
			Calibrating = False
			CalIndex = 0
			for c in clients:
				refs = c.getRefPointList()
				basis = vu.GetNewBasis(refs[0], refs[1], refs[2])
				c.setBasis(basis)

def calButton():
	calibrator(None)

def clickEvent(event):
	global clients
	global nindex
	v = clients[nindex].getVertex()
	ind = int(event.y * 640 + event.x)
	if len(v) > ind:
		calibrator(v[ind], event.x, event.y)
	else:
		calibrator(None)

def initGUI():
	wheel = Gouda.Wheel()
	wheel.defaultConfig()

	wheel.addFrame("nonode", 250, 240)
	wheel.setFrameText("nonode", "No node connected", "red")

	wheel.addFrame("vid", 0, 0)
	wheel.setFrameClickEvent("vid", clickEvent)
	wheel.addFrame("depth", 480, 360)

	wheel.addFrame("title", 645, 10)
	wheel.setFrameText("title", "Hello!", "red")

	wheel.addFrame("threshQ", 645, 125)
	wheel.setFrameText("threshQ", "Threshold (%):", "blue")
	wheel.addUserEntry("thresh", 670, 150, 3, None)

	wheel.addFrame("pdistQ", 755, 125)
	wheel.setFrameText("pdistQ", "Sensitivity:", "blue")
	wheel.addUserEntry("pdist", 770, 150, 3, None)

	wheel.addFrame("tempQ", 645, 200)
	wheel.setFrameText("tempQ", "Target Temp (ºF):")
	wheel.addUserEntry("temp", 670, 220, 3, None)

	wheel.addFrame("peopleT", 645, 260)
	wheel.setFrameText("peopleT", "People Count:")
	wheel.addFrame("people", 670, 280)
	wheel.setFrameText("people", "idk")

	wheel.addFrame("tempT", 645, 310)
	wheel.setFrameText("tempT", "Temperature")
	wheel.addFrame("temp", 670, 330)
	wheel.setFrameText("temp", "idk")

	wheel.addFrame("co2T", 745, 310)
	wheel.setFrameText("co2T", "CO2")
	wheel.addFrame("co2", 770, 330)
	wheel.setFrameText("co2", "idk")

	wheel.addFrame("humidT", 645, 360)
	wheel.setFrameText("humidT", "Humidity")
	wheel.addFrame("humid", 670, 380)
	wheel.setFrameText("humid", "idk")

	wheel.addFrame("tvocT", 745, 360)
	wheel.setFrameText("tvocT", "TVOC")
	wheel.addFrame("tvoc", 770, 380)
	wheel.setFrameText("tvoc", "idk")

	wheel.addFrame("HVAC", 0, 480)

	wheel.addFrame("poopy", 1000, 360)
	wheel.setFrameText("poopy", "( '_>') Don't look here")

	#wheel.addFrame("sensors", 200, 200)
	#wheel.setFrameText("sensors", "")

	wheel.addFrame("nodeno", 0, 56)

	wheel.addButton("quit", "exit", 675, 430, exitButtonPress)
	wheel.addButton("confidence", "Confidence\nOff", 650, 50, confButtonPress)

	wheel.addButton("newnode", "New  Node", 0, 0, newNodeButton)
	wheel.addButton("selnode", ">", 0, 28, selNodeButton)
	wheel.addButton("delnode", "x", 606, 0, delNodeButton)
	wheel.addButton("calibrate", "", 0, 450, calButton)
	return wheel




def main():
	global keyPress
	global gui
	global showConf
	global clients
	global nindex
	global connHandler
	global serv

	print("Reading in config file")
	config = None
	try:
		with open("config.json", 'r') as f:
			config = json.loads(f.read())
	except:
		print("Unable to open config file")
		exit()
	print("Loaded config file\n", json.dumps(config))

	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	connHandler = NewConnHandler()
	
	# For averaging framerate
	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	frames = 0

	# adjust this to point to your downloaded/trained model
	model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	#model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference10.h5'

	print("loading CNN model")
	# load retinanet model
	bThread = BoxThread(model_path)
	bThread.start()

	theta, meanStd = Reg.loadModel("noisy_sensor_model.csv")

	gui = initGUI()
	thresh = 0
	pdist = 0

	while keyPress != ord('q'):
		connHandler.tick()
		# New client connection established
		if connHandler.getState() == connHandler.STATE_FINISHED:
			newClient = connHandler.getClient()
			if newClient is not None:
				cids = [citer.getID() for citer in clients]
				c = NodeData()
				c.setClient(newClient)
				for i in range(1000):
					if not i in cids:
						c.setID(i)
						break
				clients.append(c)
				nindex = len(clients) - 1
				bThread.addClient(c)
		statusStr, statusColor = connHandler.getStatusStr()
		gui.setButtonText("newnode", statusStr, statusColor)
		gui.update()

		gui.setFrameText("nodeno", "%d/%d" % (nindex + 1, len(clients)))
		if not clients:
			gui.setFrameImage("vid", [])
			gui.setFrameImage("depth", [])
			continue

		# Receive color and depth data from each client
		for c in clients:
			data = serv.receiveData(c.getClient(), Roni.TYPE_RGB)
			depth = serv.receiveData(c.getClient(), Roni.TYPE_DEPTH)
			sense = serv.receiveData(c.getClient(), Roni.TYPE_EDGE)
			vfrac = 0
			vertStr = None
			for i in range(8):
				vertStr = serv.receiveData(c.getClient(), Roni.TYPE_3D_0 + i)
				if vertStr is not None and vertStr:
					vertStr = pickle.loads(vertStr)
					vfrac = i
					break

			# Decode data to images and sensor readings
			if data is not None and data:
				img = Coppa.decodeColorFrame(data)
				c.setColor(img.copy())
			if depth is not None and depth:
				depImg = Coppa.decodeDepthFrame(depth)
				c.setDepth(depImg.copy())
			if sense is not None and sense:
				sense = pickle.loads(sense)
				if len(sense) > 3:
					c.setCO2(sense[0])
					c.setTVOC(sense[1])
					c.setTemp(sense[2])
					c.setHumidity(sense[3])
			if vertStr is not None and len(vertStr) > 0:
				c.setVertex(vertStr, vfrac)

		# Skip first 10 frames, just in case
		frames += 1
		if frames < 10: continue

		newThresh = gui.getEntryValue("thresh")
		try:
			thresh = int(newThresh)
		except:
			pass

		newPDist = gui.getEntryValue("pdist")
		try:
			pdist = float(newPDist)
		except:
			pass

		newTrgtTemp = gui.getEntryValue("temp")
		try:
			config["Sensors"]["TargetOccupTemp"] = int(newTrgtTemp)
		except:
			pass

		# get copies of color and depth frames
		img = clients[nindex].getColor().copy()
		depImg = clients[nindex].getDepth().copy()
		if len(img) < 1 or  len(depImg) < 1:
			continue

		fVert = clients[nindex].getVertex()

		# Display calibration points
		refXY = clients[nindex].getRefXY()
		for pt in refXY:
			cv2.circle(img, (pt[0], pt[1]), 4, (0, 255, 255), 1)
			img[pt[1]][pt[0]] = [0, 255, 255]
		
		# Find valid boxes and add to or update people list
		people = []
		for c in clients:
			boxes, scores, labels = c.getResults()
			if len(boxes) > 0:
				for box, score, label in zip(boxes[0], scores[0], labels[0]):
					# scores are sorted so we can break at first low score found
					if score < (thresh / 100):
						break
					Peeps.AddToPeople(c, box, people, pdist, score)
							
		totalPeeps = len(people)
		frameBoxes, scores, isDupe = Peeps.GetIDBoxes(nindex, people)
		framePeeps = len(frameBoxes)

		# Visualize People found in frame
		for i in range(framePeeps):
			# color based on if appears in multiple frames
			color = (0, 0, 255)
			if isDupe[i]: color = (0, 255, 0)
		
			b = frameBoxes[i].astype(int)
			draw_box(img, b, color=color)
			
			# draw center point of box
			x = int((frameBoxes[i][0] + frameBoxes[i][2]) / 2)
			y = int((frameBoxes[i][1] + frameBoxes[i][3]) / 2)
			cv2.circle(img, (x,y), 3, (255, 255, 0), -1)
			
			if showConf:
				draw_caption(img, b, "%0.0f%%" % (float(scores[i]) * 100))


		# Calculate frame rate and display on image
		t1 = time.time()
		last10[it] = 1.0 / (t1 - t0)
		it = (it + 1) % 10
		fps = np.average(last10)
		t0 = t1
		cv2.putText(img, "stream FPS: %.2f" % fps, (0, 30),
		cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
	
		# Sensor readings
		fCO2 = clients[nindex].getCO2()
		fTVOC = clients[nindex].getTVOC()
		fTemp = clients[nindex].getTemp() 
		fHum = clients[nindex].getHumidity()
	
		# Regression
		# [people, co2, tvoc, tempr, humid]
		regInput = [totalPeeps, fCO2, fTVOC, 70, fHum]
		res = Reg.runThroughModel(regInput, theta, meanStd)

		# Do HVAC stuff
		ctrl = controlHVAC(config, res, 70) + ("(%.2f)" % res)
		gui.setFrameText("HVAC", ctrl)

		# Display sensor readings on GUI
		gui.setFrameText("people", "%d/%d (frame/total)" % (framePeeps, totalPeeps))
		gui.setFrameText("temp", "%.2f ºF" % fTemp)
		gui.setFrameText("humid", "%.2f%%" % fHum)
		gui.setFrameText("co2", "%.0f ppm" % fCO2)
		gui.setFrameText("tvoc", "%.0f ppb" % fTVOC)

		# Display color and depth images (depth picture-in-picture)
		imRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		depRGB = cv2.cvtColor(depImg, cv2.COLOR_BGR2RGB)
		guiImg = ImageTk.PhotoImage(Image.fromarray(imRGB))
		depImg = ImageTk.PhotoImage(Image.fromarray(cv2.resize(depRGB, (160, 120))))

		gui.setFrameImage("vid", guiImg)
		gui.setFrameImage("depth", depImg)


	# Close server and display window 
	bThread.stop()
	bThread.join()
	serv.close()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
