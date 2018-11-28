import cv2
import numpy as np
import time

import pysrc.Roni as Roni
import pysrc.Coppa as Coppa


def main():
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	cl = serv.getClient()
	cv2.namedWindow('RoniStream', cv2.WINDOW_AUTOSIZE)

	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	frames = 0
	framerates = []

	keyPressed = None
	while keyPressed != ord('q'):
		data = serv.receiveData(cl, Roni.TYPE_RGB)
	
		if len(data) > 0:
			frames += 1
			
			# get image, copy, and edge detect
			img = Coppa.decodeColorFrame(data)
			orig = np.copy(img)

			w = img.shape[1]
			h = img.shape[0]

			#edge detect
			edge = cv2.Canny(img, 100, 400)
			for i,row in enumerate(edge):
				for j,px in enumerate(row):
					if px:
						img[i][j] = [0, 255, 64]

			# Calculate framerate and display on image
			t1 = time.time()
			last10[it] = 1.0 / (t1 - t0)
			it = (it + 1) % 10
			fps = np.average(last10)
			#if 1010 > frames >= 10: framerates.append(fps)
			t0 = t1
			cv2.putText(img, "FPS: %.2f" % fps, (0, 30),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
		
			# Combine original, edge dectect, and overlay images
			edge3 = np.empty_like(img)
			edge3 = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
			img = cv2.copyMakeBorder(img, 0, 0, int(w/2), int(w/2),
			cv2.BORDER_CONSTANT, (0, 0, 0))
			combined = np.hstack((orig, edge3))
			combined = np.vstack((combined, img))

			# Display
			cv2.imshow('RoniStream', combined)
			keyPressed = cv2.waitKey(1)

	#with open("last1000frames_cpu.csv", 'w') as fi:
	#	for n in framerates:
	#		fi.write("%f," % n)
	
	# Close server and display window
	serv.close()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
