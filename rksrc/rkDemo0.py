import pysrc.Roni as Roni
import pysrc.Coppa as Coppa
import pysrc.Sopressata as Sopressata
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import cv2 as cv
import time
import pickle

DEBUG = True

def main():
	print("Enabling streams and starting camera...")
	pipeline = rs.pipeline()
	pc = rs.pointcloud()

	uartPort = "/dev/ttyS3"
	uartBaud = 115200
	rtscts = True

	cape = Sopressata.Cape(uartPort, uartBaud, rtscts)

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
	cape.openPort()
	print("UART connected to cape:", uartPort)
	print("streaming...")
	
	itr = 0
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
		step = int(len(verts) / 8) #640x480 / 8 = 38,400
		vertSend = verts[itr*step : step*(itr+1)]
		vertPick = pickle.dumps(vertSend, protocol=pickle.HIGHEST_PROTOCOL)

		# Get aligned depth and color frames
		#depthFrame = alignedFrames.get_depth_frame()
		#colorFrame = alignedFrames.get_color_frame()
		
		# Read sensor data from Cape UART
		rawRead = cape.readSerial()
		if rawRead:
			co2, tvoc, temp, humid = cape.getAllSensorData(rawRead)
			sensors = [co2, tvoc, temp, humid]
			senseDump = pickle.dumps(sensors, protocol=pickle.HIGHEST_PROTOCOL)
			client.sendData(senseDump, Roni.TYPE_EDGE)

		depthTxt = Coppa.encodeDepthFrame(depthFrame)
		colorTxt = Coppa.encodeColorFrame(colorFrame)
		
		client.sendData(depthTxt, Roni.TYPE_DEPTH)
		client.sendData(vertPick, (Roni.TYPE_3D_0 + (itr % 8)))
		if not client.sendData(colorTxt, Roni.TYPE_RGB):
			break
		
		itr = (itr+1) % 8

	client.close()
	cape.closePort()

if __name__ == '__main__':
	main()
