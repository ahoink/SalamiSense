import pyrealsense2 as rs
import numpy as np
from PIL import Image

DEBUG = True

print("Enabling streams and starting camera...")
pipeline = rs.pipeline()

# Enable color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)

# Start streams
profile = pipeline.start(config)

# Depth sensor info
depthSensor = profile.get_device().first_depth_sensor()
depthScale = depthSensor.get_depth_scale()

# Enable stream alignment to align depth to color
alignStream = rs.stream.color
align = rs.align(alignStream)

print("Begin receiving frames from camera...")
while True:
	# Get next set of frames from stream
	frames = pipeline.wait_for_frames()

	# Align depth frame to color frame
	alignedFrames = align.process(frames)

	# Get aligned depth and color frames
	depthFrame = alignedFrames.get_depth_frame()
	colorFrame = alignedFrames.get_color_frame()

	color = np.asanyarray(colorFrame.get_data())
	depth = np.asanyarray(depthFrame.get_data())	
	
	colorImg = Image.fromarray(color, "RGB")
	colorImg.save("color_test.png")

	break
	
# Stop streams
pipeline.stop()
