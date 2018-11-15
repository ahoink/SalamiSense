from pysrc.Roni import RoniClient
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import base64
import cv2 as cv
import time

DEBUG = True

def main():
	print("Enabling streams and starting camera...")
	pipeline = rs.pipeline()

	# Enable color and depth streams
	config = rs.config()
	config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
	config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

	# Start streams
	profile = pipeline.start(config)

	# Depth sensor info
	depthSensor = profile.get_device().first_depth_sensor()
	depthScale = depthSensor.get_depth_scale()

	# Enable stream alignment to align depth to color
	alignStream = rs.stream.color
	align = rs.align(alignStream)

	print("Initializing Client")
	client = RoniClient()
	print("Connecting")
	client.connect()

	while True:
		# Get next set of frames from stream
		frames = pipeline.wait_for_frames()

		# Align depth frame to color frame
		alignedFrames = align.process(frames)

		# Get aligned depth and color frames
		depthFrame = alignedFrames.get_depth_frame()
		colorFrame = alignedFrames.get_color_frame()

		if not colorFrame:
			continue

		color = np.asanyarray(colorFrame.get_data())
		depth = np.asanyarray(depthFrame.get_data())

		if len(color) == 0:
			continue

		ret, buf = cv.imencode('.jpg', color)
		#cv.imwrite('poo.jpg', buf)
		jpgTxt = base64.b64encode(buf)

		print(len(jpgTxt))
		client.sendData(jpgTxt)


	serv.close()

if __name__ == '__main__':
	main()
