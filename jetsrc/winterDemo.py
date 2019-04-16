import cv2
import numpy as np
import time
import threading

import pysrc.Roni as Roni
import pysrc.Coppa as Coppa

import keras
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import tensorflow as tf

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



def main():
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	cl = serv.getClient()
	cv2.namedWindow('RoniStream', cv2.WINDOW_AUTOSIZE)
	
	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	frames = 0
	keyPress = None


	# adjust this to point to your downloaded/trained model
	model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	#model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference10.h5'

	print("loading model")
	# load retinanet model
	bThread = BoxThread(model_path)
	bThread.start()

	# load label to names mapping for visualization purposes
	labels_to_names = {0: 'person'}

	while keyPress != ord('q'):
		data = serv.receiveData(cl, Roni.TYPE_RGB)
		
		if len(data) > 0:
			frames += 1
			if frames < 50: continue

			# get image
			img = Coppa.decodeColorFrame(data)

			bThread.setData(img)
			boxes, scores, labels = bThread.getBoxes()
			bc = 0
			if len(boxes) > 0:
				for box, score, label in zip(boxes[0], scores[0], labels[0]):
					# scores are sorted so we can break
					if score < 0.6:
						break
					
					bc += 1
					color = label_color(label)
			
					b = box.astype(int)
					draw_box(img, b, color=color)
					draw_caption(img, b, "%s" % score)

			# Calculate frame rate and display on image
			t1 = time.time()
			last10[it] = 1.0 / (t1 - t0)
			it = (it + 1) % 10
			fps = np.average(last10)
			t0 = t1
			cv2.putText(img, "stream FPS: %.2f" % fps, (0, 30),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
		
			cv2.putText(img, "People: %d" % bc, (0, 60),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
			
			# Display
			#img = cv2.resize(img, (1280, 960))
			cv2.imshow('RoniStream', img)
			keyPress = cv2.waitKey(1)


	# Close server and display window 
	bThread.stop()
	bThread.join()
	serv.close()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
