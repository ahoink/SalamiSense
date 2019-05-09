import threading
import pysrc.Roni as Roni
import keras
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import tensorflow as tf
import numpy as np
import time


def get_session():
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    return tf.Session(config=config)

class NodeData(object):
	def __init__(self):
		self.client = None

		self.boxes = []
		self.scores = []
		self.labels = []
		self.color = []
		self.depth = []

		self.people = 0
		self.temp = 0
		self.hum = 0
		self.co2 = 0
		self.tvoc = 0

	def setClient(self, client):
		self.client = client
	def getClient(self):
		return self.client

	def getResults(self):
		return self.boxes, self.scores, self.labels
		
	def setBoxes(self, boxes):
		self.boxes = boxes
	def getBoxes(self):
		return self.boxes
	
	def setScores(self, scores):
		self.scores = scores
	def getScores(self):
		return self.scores
	
	def setLabels(self, labels):
		self.labels = labels
	def getLabels(self):
		return self.labels
	
	def setColor(self, color):
		self.color = color
	def getColor(self):
		return self.color

	def setDepth(self, depth):
		self.depth = depth
	def getDepth(self):
		return self.depth

	def setPeople(self, people):
		self.people = people
	def getPeople(self):
		return self.people
	
	def setTemp(self, temp):
		self.temp = temp
	def getTemp(self):
		return self.temp

	def setHumidity(self, humidity):
		self.hum = humidity
	def getHumidity(self):
		return self.hum

	def setCO2(self, co2):
		self.co2 = co2
	def getCO2(self):
		return self.co2

	def setTVOC(self, tvoc):
		self.tvoc = tvoc
	def getTVOC(self):
		return self.tvoc

class BoxThread(threading.Thread):
	def __init__(self, modelPath):
		super(BoxThread, self).__init__()
		self.modelPath = modelPath
		self.model = None
		self.go = True
		self.clients = []

	def addClient(self, client):
		self.clients.append(client)

	def setData(self, data):
		self.data = data.copy()

	def run(self):
		# set the modified tf session as backend in keras
		keras.backend.tensorflow_backend.set_session(get_session())
		self.model = models.load_model(self.modelPath, backbone_name='resnet50')
		print("Model Loaded")
		while self.go:
			for c in self.clients:
				if c.getColor() is not None:
					try:
						#t0 = time.time()
						imgcpy = c.getColor().copy()
						img = preprocess_image(imgcpy)

						boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(img, axis=0))
						c.setBoxes(boxes.copy())
						c.setScores(scores.copy())
						c.setLabels(labels.copy())
						#t1 = time.time()
						#print("compute time: %f" % (t1-t0))
					except:
						pass
	
	def stop(self):
		self.go = False



class ConnThread(threading.Thread):
	def __init__(self, roniHost):
		super(ConnThread, self).__init__()
		self.host = roniHost
		self.running = False
		self.client = None

	def getClient(self):
		c = self.client
		self.client = None
		return c

	def run(self):
		self.running = True
		self.client = self.host.getClient()
		self.running = False

class NewConnHandler(object):
	STATE_IDLE = 0
	STATE_RUNNING = 1
	STATE_FINISHED = 2
	STATE_FINISHED_WAIT = 3
	STATE_TIMEOUT = 4

	def __init__(self):
		self.state = self.STATE_IDLE
		self.client = None
		self.thrd = None
		self.timeout = 0

	def tick(self):
		if self.state == self.STATE_RUNNING:
			if self.thrd is not None and not self.thrd.running:
				self.client = self.thrd.getClient()
				self.thrd.join()
				self.thrd = None
				if self.client is None:
					self.timeout = time.time()
					self.state = self.STATE_TIMEOUT
				else:
					self.state = self.STATE_FINISHED
		elif self.state == self.STATE_TIMEOUT:
			if (time.time() - self.timeout) > 2:
				self.state = self.STATE_IDLE
		elif self.state == self.STATE_FINISHED_WAIT:
			if (time.time() - self.timeout) > 1:
				self.state = self.STATE_IDLE

	def newClient(self, roniHost):
		if self.state != self.STATE_IDLE:
			return False
		self.thrd = ConnThread(roniHost)
		self.thrd.start()
		self.state = self.STATE_RUNNING
		return True
	
	def getClient(self):
		if self.state == self.STATE_FINISHED:
			self.state = self.STATE_FINISHED_WAIT
			self.timeout = time.time()
			c = self.client
			self.client = None
			return c
		return None

	def getState(self):
		return self.state

	def getStatusStr(self):
		if self.state == self.STATE_IDLE:
			return "New  Node", "black"
		elif self.state == self.STATE_RUNNING:
			return "Searching", "#2010B0"
		elif self.state == self.STATE_FINISHED or \
			 self.state == self.STATE_FINISHED_WAIT:
			return "Connected", "#104010"
		elif self.state == self.STATE_TIMEOUT:
			return "Timed Out", "red"
		else:
			return ""


