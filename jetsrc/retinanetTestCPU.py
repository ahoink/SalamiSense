import cv2
import numpy as np
import time

import keras
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import tensorflow as tf

def get_session():
    config = tf.ConfigProto(device_count={"GPU":0})
    config.gpu_options.allow_growth = True
    return tf.Session(config=config)

def main():
	
	img = cv2.imread("tools/avery_test_Color.png", 1)

	# set the modified tf session as backend in keras
	keras.backend.tensorflow_backend.set_session(get_session())

	# adjust this to point to your downloaded/trained model
	model_path = '/home/nvidia/Documents/SalamiSense/snapshots_1/resnet50_csv_inference.h5'
	#model_path = '/home/nvidia/Documents/SalamiSense/resnet50_csv_inference.h5'
	
	# load retinanet model
	model = models.load_model(model_path, backbone_name='resnet50')
	
	# load label to names mapping for visualization purposes
	labels_to_names = {0: 'person'}

	for i in range(16):
		img_copy = img.copy()
		img_copy = preprocess_image(img_copy)
		
		t0 = time.time()
		
		boxes, scores, labels = model.predict_on_batch(np.expand_dims(img_copy, axis=0))

		t1 = time.time()
		elapsed = (t1 - t0)
		if i > 0: print(elapsed)


if __name__ == '__main__':
	main()
