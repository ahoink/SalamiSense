import cv2
import numpy as np
import time
import threading
import pyrealsense2 as rs

import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
import pysrc.Gouda as Gouda
from pysrc.SalamiUtils import NodeData, NewConnHandler

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

class BoxThread(threading.Thread):
	def __init__(self, modelPath):
		super(BoxThread, self).__init__()
		self.modelPath = modelPath
		self.model = None
		self.boxes = []
		self.scores = []
		self.labels = []
		self.data = None
		self.go = True

	def setData(self, data):
		self.data = data.copy()

	def run(self):
		# set the modified tf session as backend in keras
		keras.backend.tensorflow_backend.set_session(get_session())
		self.model = models.load_model(self.modelPath, backbone_name='resnet50')
		print("Model Loaded")
		while self.go:
			if self.data is not None:
				#t0 = time.time()
				imgcpy = self.data.copy()
				img = preprocess_image(imgcpy)

				boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(img, axis=0))
				self.boxes = boxes.copy()
				self.scores = scores.copy()
				self.labels = labels.copy()
				#t1 = time.time()
				#print("compute time: %f" % (t1-t0))
	
	def getBoxes(self):
		return self.boxes, self.scores, self.labels

	def stop(self):
		self.go = False


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

def initGUI():
	wheel = Gouda.Wheel()
	wheel.defaultConfig()
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
	wheel.setFrameText("peopleT", "People in Frame:")
	wheel.addFrame("people", 670, 280)
	wheel.setFrameText("people", "idk")

	wheel.addFrame("tempT", 645, 310)
	wheel.setFrameText("tempT", "Temperature")
	wheel.addFrame("temp", 670, 330)
	wheel.setFrameText("temp", "idk")

	wheel.addFrame("humidT", 645, 360)
	wheel.setFrameText("humidT", "Humidity")
	wheel.addFrame("humid", 670, 380)
	wheel.setFrameText("humid", "idk")

	wheel.addButton("quit", "exit", 675, 430, exitButtonPress)
	wheel.addButton("confidence", "Confidence\nOff", 650, 50, confButtonPress)

	wheel.addButton("newnode", "New  Node", 0, 0, newNodeButton)
	wheel.addButton("selnode", ">", 0, 24, selNodeButton)
	return wheel


def main():
	global keyPress
	global gui
	global showConf
	global clients
	global nindex
	global connHandler
	global serv
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	connHandler = NewConnHandler()
	#connHandler.newClient(serv)
	#while(connHandler.getState() != connHandler.STATE_FINISHED):
	#	connHandler.tick()
	#cl = connHandler.getClient()
	
	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	frames = 0


	# adjust this to point to your downloaded/trained model
	model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	#model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference10.h5'

	print("loading model")
	# load retinanet model
	bThread = BoxThread(model_path)
	bThread.start()

	# load label to names mapping for visualization purposes
	labels_to_names = {0: 'person'}
	colorizer = rs.colorizer()

	gui = initGUI()
	thresh = 0

	while keyPress != ord('q'):
		connHandler.tick()
		if connHandler.getState() == connHandler.STATE_FINISHED:
			clients.append(connHandler.getClient())
			nindex += 1
		gui.setButtonText("newnode", connHandler.getStatusStr())
		gui.update()

		if not clients:
			continue

		data = []
		depth = []
		while not data or not depth:
			if not data:
				data = serv.receiveData(clients[nindex], Roni.TYPE_RGB)
			if not depth:
				depth = serv.receiveData(clients[nindex], Roni.TYPE_DEPTH)

		frames += 1
		if frames < 10: continue

		# get image
		img = Coppa.decodeColorFrame(data)
		depImg = Coppa.decodeDepthFrame(depth)
		if depImg is None: continue

		newThresh = gui.getEntryValue("thresh")
		try:
			thresh = int(newThresh)
		except:
			pass

		bThread.setData(img)
		boxes, scores, labels = bThread.getBoxes()
		bc = 0
		if len(boxes) > 0:
			for box, score, label in zip(boxes[0], scores[0], labels[0]):
				# scores are sorted so we can break
				if score < (thresh / 100):
					break
				bc += 1
				color = label_color(label)
		
				b = box.astype(int)
				draw_box(img, b, color=color)
				if showConf:
					draw_caption(img, b, "%s" % score)

		# Calculate frame rate and display on image
		t1 = time.time()
		last10[it] = 1.0 / (t1 - t0)
		it = (it + 1) % 10
		fps = np.average(last10)
		t0 = t1
		cv2.putText(img, "stream FPS: %.2f" % fps, (0, 30),
		cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
	
		gui.setFrameText("people", "%d" % bc)
	
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
