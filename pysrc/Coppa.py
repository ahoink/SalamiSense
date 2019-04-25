import base64
import numpy as np
import cv2

def encodeColorFrame(colorFrame, dtype=np.uint8):
	color = np.asanyarray(colorFrame.get_data(), dtype=dtype)
	ret, buf = cv2.imencode('.jpg', color)
	jpgTxt = base64.b64encode(buf)
	return jpgTxt

def decodeColorFrame(colorTxt, dtype=np.uint8):
	imgData = base64.b64decode(colorTxt)
	imgData = np.frombuffer(imgData, dtype=dtype)
	img = cv2.imdecode(imgData, flags=1)
	return img

def encodeDepthFrame(depthFrame, dtype=np.uint16):
	df = depthFrame.get_data()
	depth = np.asanyarray(df, dtype=dtype)
	depColorMap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03),cv2.COLORMAP_JET)
	ret, buf = cv2.imencode('.jpg', depColorMap)
	jpgTxt = base64.b64encode(buf)
	return jpgTxt

def decodeDepthFrame(depthTxt, dtype=np.uint8):
	imgData = base64.b64decode(depthTxt)
	try:
		imgData = np.frombuffer(imgData, dtype=dtype)
	except:
		return None
	img = cv2.imdecode(imgData, flags=1)
	return img

