import base64
import numpy as np
import cv2 as cv

def encodeColorFrame(colorFrame):
	color = np.asanyarray(colorFrame.get_data())
	ret, buf = cv.imencode('.jpg', color)
	jpgTxt = base64.b64encode(buf)
	return jpgTxt

def decodeColorFrame(colorTxt):
	imgData = base64.b64decode(colorTxt)
	imgData = np.frombuffer(imgData, dtype=np.uint8)
	img = cv.imdecode(imgData, flags=1)
	return img
