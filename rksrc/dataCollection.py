import pyrealsense2 as rs
import numpy as np
import math
import time
import cv2

DEBUG = True

print("Enabling streams and starting camera...")
pipeline = rs.pipeline()

# Enable color and depth streams
config = rs.config()
#config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streams
profile = pipeline.start(config)

# Depth sensor info
#depthSensor = profile.get_device().first_depth_sensor()
#depthScale = depthSensor.get_depth_scale()

# Enable stream alignment to align depth to color
alignStream = rs.stream.color
align = rs.align(alignStream)

print("Begin receiving frames from camera...")
skip = 30
it = 0
numImgs = 1000
slep = 2
saveDir = "/home/rock/Projects/SalamiSense/data/"

while (it - skip) < numImgs:
	# Get next set of frames from stream
	frames = pipeline.wait_for_frames()

	# Align depth frame to color frame
	#alignedFrames = align.process(frames)

	# Get aligned depth and color frames
	#depthFrame = alignedFrames.get_depth_frame()
	#colorFrame = alignedFrames.get_color_frame()
	colorFrame = frames.get_color_frame()
	it += 1
	if it < skip:
		continue
	#print("Calculating point cloud")
	#pc = rs.pointcloud()
	#points3D = pc.calculate(depthFrame)
	#vertices = points3D.get_vertices()
	#vertices = np.asanyarray(vertices)
	color = np.asanyarray(colorFrame.get_data())

	cv2.imwrite("%simg_%04d.png" % (saveDir, (it - skip)), color)
	print("%d/%d" % (it - skip + 1, numImgs))

	time.sleep(slep)
	
# Stop streams
pipeline.stop()
