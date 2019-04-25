import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import base64
import cv2 as cv
import time
import pickle

DEBUG = True

def main():
	print("Enabling streams and starting camera...")
	pipeline = rs.pipeline()
	pc = rs.pointcloud()

	# Enable color and depth streams
	config = rs.config()
	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start streams
	profile = pipeline.start(config)

	# Depth sensor info
	depthSensor = profile.get_device().first_depth_sensor()
	
	depthProfile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
	#epthSensor.set_option(rs.option.visual_preset, 0)
	depthScale = depthSensor.get_depth_scale()
	
	# Enable stream alignment to align depth to color
	#alignStream = rs.stream.color
	#align = rs.align(alignStream)

	print("Initializing Client")
	client = Roni.RoniClient()
	print("Connecting...")
	client.connect()
	print("streaming...")

	while True:
		# Get next set of frames from stream
		frames = pipeline.wait_for_frames()
		
		depthFrame = frames.get_depth_frame()
		colorFrame = frames.get_color_frame()
		
		# Align depth frame to color frame
		pc.map_to(colorFrame)
		#alignedFrames = align.process(frames)
		
		pts = pc.calculate(depthFrame)
		v = pts.get_vertices()
		verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
		
		vertPick = pickle.dumps(verts, protocol=pickle.HIGHEST_PROTOCOL)
		
		# Get aligned depth and color frames
		#depthFrame = alignedFrames.get_depth_frame()
		#colorFrame = alignedFrames.get_color_frame()
		
		depthTxt = Coppa.encodeDepthFrame(depthFrame)
		colorTxt = Coppa.encodeColorFrame(colorFrame)
		
		client.sendData(depthTxt, Roni.TYPE_DEPTH)
		#client.sendData(vertPick, Roni.TYPE_3D)
		if not client.sendData(colorTxt, Roni.TYPE_RGB):
			break
		

	client.close()

if __name__ == '__main__':
	main()
