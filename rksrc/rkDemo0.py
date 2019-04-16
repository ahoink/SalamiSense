import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
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
	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

	# Start streams
	profile = pipeline.start(config)

	# Depth sensor info
	depthSensor = profile.get_device().first_depth_sensor()
	depthScale = depthSensor.get_depth_scale()

	# Enable stream alignment to align depth to color
	alignStream = rs.stream.color
	align = rs.align(alignStream)

	print("Initializing Client")
	client = Roni.RoniClient()
	print("Connecting...")
	client.connect()
	print("streaming...")

	while True:
		# Get next set of frames from stream
		frames = pipeline.wait_for_frames()

		# Align depth frame to color frame
		alignedFrames = align.process(frames)

		# Get aligned depth and color frames
		depthFrame = alignedFrames.get_depth_frame()
		colorFrame = alignedFrames.get_color_frame()

		#print(len(jpgTxt))
		depthTxt = Coppa.encodeDepthFrame(depthFrame)
		colorTxt = Coppa.encodeColorFrame(colorFrame)
		
		#if client.sendData(depthTxt) == False and client.sendData(colorTxt) == False:
		#	break
		client.sendData(depthTxt, Roni.TYPE_DEPTH)
		client.sendData(colorTxt, Roni.TYPE_RGB)

	client.close()

if __name__ == '__main__':
	main()
