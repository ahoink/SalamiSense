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
	while True:
		data = serv.receiveData(cl, Roni.TYPE_RGB)
	
		if len(data) > 0:
			#imgData = base64.b64decode(data)
			#img = np.frombuffer(imgData, dtype=np.uint8)
			#somethingElse = cv2.imdecode(img, flags=1)
			#cv2.imshow('RoniStream', somethingElse)
			img = Coppa.decodeColorFrame(data)
			
			#edge detect
			edge = cv2.Canny(img, 100, 400)
			for i,row in enumerate(edge):
				for j,px in enumerate(row):
					if px:
						img[i][j] = [0, 0, 0]

			t1 = time.time()
			last10[it] = 1.0 / (t1 - t0)
			it = (it + 1) % 10
			fps = np.average(last10)
			t0 = t1
			cv2.putText(img, "FPS: %.2f" % fps, (0, 30),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
			
			cv2.imshow('RoniStream', img)
			cv2.waitKey(1)


if __name__ == '__main__':
	main()
