import cv2
import numpy as np
import time

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

	# set the modified tf session as backend in keras
	keras.backend.tensorflow_backend.set_session(get_session())

	# adjust this to point to your downloaded/trained model
	#model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference.h5'
	
	print("loading model")
	# load retinanet model
	model = models.load_model(model_path, backbone_name='resnet50')
	
	# load label to names mapping for visualization purposes
	labels_to_names = {0: 'person'}

	while keyPress != ord('q'):
		data = serv.receiveData(cl, Roni.TYPE_RGB)
		
		if len(data) > 0:
			frames += 1
			if frames < 50: continue

			# get image
			img = Coppa.decodeColorFrame(data)

			img_copy = img.copy()
			img = preprocess_image(img)

			boxes, scores, labels = model.predict_on_batch(np.expand_dims(img, axis=0))

			for box, score, label in zip(boxes[0], scores[0], labels[0]):
				# scores are sorted so we can break
				if score < 0.6:
					break
			
				color = label_color(label)
		
				b = box.astype(int)
				draw_box(img_copy, b, color=color)
				draw_caption(img_copy, b, "%s" % score)

			# Calculate frame rate and display on image
			t1 = time.time()
			last10[it] = 1.0 / (t1 - t0)
			it = (it + 1) % 10
			fps = np.average(last10)
			t0 = t1
			cv2.putText(img_copy, "FPS: %.2f" % fps, (0, 30),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
		
			# Display
			cv2.imshow('RoniStream', img_copy)

			keyPress = cv2.waitKey(1)


	# Close server and display window 
	serv.close()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
