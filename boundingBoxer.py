import cv2
import numpy as np
import os

def mouseEvent(event, x, y, flags, params):
	global boxes
	global held

	if event == cv2.EVENT_LBUTTONDOWN:
		if not held:
			boxes.append([x, y, -1, -1])
			held = True
	elif event == cv2.EVENT_MOUSEMOVE:
		if held:
			boxes[-1][2] = x
			boxes[-1][3] = y
	elif event == cv2.EVENT_LBUTTONUP:
		held = False

cv2.namedWindow("test")
cv2.setMouseCallback("test", mouseEvent)

directory = "images"
images = os.listdir(directory)
validExt = [".png", ".jpg", ".jpeg"]
output = open("%s/results.csv" % directory, 'a')

for filename in images:
	
	# Check if file is valid image file
	valid = False
	for ext in validExt:
		if ext in filename:
			valid = True
	if not valid:
		continue

	# Open image and make a copy
	img = cv2.imread("%s/%s" % (directory, filename), 1)
	copyImg = np.copy(img)

	boxes = []
	held = False
	key = None

	while key != ord('q'):
		img = np.copy(copyImg)
		
		# undo/remove last bounding box
		if key == ord('u'):
			if boxes:
				boxes.pop()
		# save bounding box coords to file and continue to next image
		elif key == ord('n'):
			for box in boxes:
				output.write("%s,%d,%d,%d,%d,%s\n" %\
				(filename,box[0],box[1],box[2],box[3],"person"))
			break

		# Draw bounding box on image
		for box in boxes:
			if 0 <= box[0] and 0 <= box[1] and\
				0 <= box[2] and 0 <= box[3]:
				cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (0, 0, 0), 2)
				cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (0, 255, 64), 1)

		cv2.imshow("test", img)
		key = cv2.waitKey(1)


output.close()
cv2.destroyAllWindows()
