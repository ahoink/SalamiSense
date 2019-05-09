import cv2
import numpy as np
import time
import threading
import pyrealsense2 as rs
import json
import pickle

import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
import pysrc.Gouda as Gouda
from pysrc.SalamiUtils import NodeData, NewConnHandler, BoxThread

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

def controlHVAC(config, pplCount):
	# Room Occupied
	if pplCount > 0:
		# Set to Cooling
		if config["Mode"] == "Cool":
			if tempr > config["MaxOccupTemp"]:
				print("AC ON FULL BLAST")
			elif tempr > config["TargetOccupTemp"]:
				if pplCount > 8:
					print("AC ON HIGH")
				elif pplCount > 4:
					print("AC ON MEDIUM")
				else:
					print("AC ON LOW")
			else:
				print("AC OFF")
		# Set to Heating
		elif config["Mode"] == "Heat":
			if tempr < config["MinOccupTemp"]:
				print("HEAT ON FULL BLAST")
			elif tempr < config["TargetOccupTemp"]:
				if pplCount > 8:
					print("HEAT ON LOW")
				elif pplCount > 4:
					print("HEAT ON MEDIUM")
				else:
					print("HEAT ON HIGH")
			else:
				print("HEAT OFF")
	# Room Vacant
	else:
		# Set to Cooling
		if config["Mode"] == "Cool":
			if tempr > config["MaxVacantTemp"]:
				print("AC ON HIGH")
			elif tempr > config["TargetVacamtTemp"]:
				print("AC ON LOW FLOW")
			else:
				print("AC OFF")
		# Set to Heating
		elif config["Mode"] == "Heat":
			if tempr < config["MinVacantTemp"]:
				print("HEAT ON HIGH")
			elif tempr < config["TargetVacantTemp"]:
				print("HEAT ON LOW")
			else:
				print("HEAT OFF")


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

def initGUI():
	wheel = Gouda.Wheel()
	wheel.defaultConfig()

	wheel.addFrame("nonode", 250, 240)
	wheel.setFrameText("nonode", "No node connected", "red")

	wheel.addFrame("vid", 0, 0)
	wheel.addFrame("depth", 480, 360)

	wheel.addFrame("title", 645, 10)
	wheel.setFrameText("title", "Hello!", "red")

	wheel.addFrame("threshQ", 645, 125)
	wheel.setFrameText("threshQ", "Threshold (%):", "blue")
	wheel.addUserEntry("thresh", 670, 150, 3, None)

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
	#connHandler.newClient(serv)
	#while(connHandler.getState() != connHandler.STATE_FINISHED):
	#	connHandler.tick()
	#cl = connHandler.getClient()
	
	# For averaging framerate
	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	frames = 0

	# Senpais
	tempr = 0
	humid = 0
	co2 = 0


	# adjust this to point to your downloaded/trained model
	model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	#model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference10.h5'

	print("loading model")
	# load retinanet model
	bThread = BoxThread(model_path)
	bThread.start()

	gui = initGUI()
	thresh = 0

	while keyPress != ord('q'):
		connHandler.tick()
		if connHandler.getState() == connHandler.STATE_FINISHED:
			newClient = connHandler.getClient()
			if newClient is not None:
				c = NodeData()
				c.setClient(newClient)
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

		# Recieve color and depth data from each client
		for c in clients:
			data = serv.receiveData(c.getClient(), Roni.TYPE_RGB)
			depth = serv.receiveData(c.getClient(), Roni.TYPE_DEPTH)
			sense = serv.receiveData(c.getClient(), Roni.TYPE_EDGE)

			# Decode data to image
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

		# Skip first 10 frames, just in case
		frames += 1
		if frames < 10: continue

		newThresh = gui.getEntryValue("thresh")
		try:
			thresh = int(newThresh)
		except:
			pass

		img = clients[nindex].getColor().copy()
		depImg = clients[nindex].getDepth().copy()
		if len(img) < 1 or  len(depImg) < 1:
			continue

		# Count number of valid boxes and people, visualize boxes
		bc = 0
		totalPeep = 0
		for c in clients:
			boxes, scores, labels = c.getResults()
			if len(boxes) > 0:
				for box, score, label in zip(boxes[0], scores[0], labels[0]):
					# scores are sorted so we can break
					if score < (thresh / 100):
						break
					totalPeep += 1
					if c is clients[nindex]:
						bc += 1
						color = label_color(label)
				
						b = box.astype(int)
						draw_box(img, b, color=color)
						if showConf:
							draw_caption(img, b, "%0.0f%%" % (float(score) * 100))

		# Do HVAC stuff
		#controlHVAC(config, totalPeep)

		# Calculate frame rate and display on image
		t1 = time.time()
		last10[it] = 1.0 / (t1 - t0)
		it = (it + 1) % 10
		fps = np.average(last10)
		t0 = t1
		cv2.putText(img, "stream FPS: %.2f" % fps, (0, 30),
		cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
	
		fCO2 = clients[nindex].getCO2()
		fTVOC = clients[nindex].getTVOC()
		fTemp = clients[nindex].getTemp()
		fHum = clients[nindex].getHumidity()
		gui.setFrameText("people", "%d/%d (frame/total)" % (bc, totalPeep))
		#gui.setFrameText("sensors", "CO2: %f\nTVOC: %f\nTemp: %f\nHum: %f" % (fCO2,\
		#	fTVOC, fTemp, fHum))
		gui.setFrameText("temp", "%.2f ºF" % fTemp)
		gui.setFrameText("humid", "%.2f%%" % fHum)
		gui.setFrameText("co2", "%.0f ppm" % fCO2)
		gui.setFrameText("tvoc", "%.0f ppb" % fTVOC)
	
		imRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		depRGB = cv2.cvtColor(depImg, cv2.COLOR_BGR2RGB)
		guiImg = ImageTk.PhotoImage(Image.fromarray(imRGB))
		depImg = ImageTk.PhotoImage(Image.fromarray(cv2.resize(depRGB, (160, 120))))

		gui.setFrameImage("vid", guiImg)
		gui.setFrameImage("depth", depImg)

		# Display


	# Close server and display window 
	bThread.stop()
	bThread.join()
	serv.close()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
