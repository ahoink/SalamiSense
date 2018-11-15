import cv2
import sys
import numpy as np
from PIL import Image
import base64
import pysrc.Roni as Roni
import pysrc.Coppa as Coppa

def main():
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	cl = serv.getClient()
	cv2.namedWindow('RoniStream', cv2.WINDOW_AUTOSIZE)

	while True:
		data = serv.receiveData(cl, Roni.TYPE_RGB)
		if len(data) > 0:
			#imgData = base64.b64decode(data)
			#img = np.frombuffer(imgData, dtype=np.uint8)
			#somethingElse = cv2.imdecode(img, flags=1)
			#cv2.imshow('RoniStream', somethingElse)
			img = Coppa.decodeColorFrame(data)
			
			#edge detect
			edge = cv2.Canny(img, 200, 300)
			for i,row in enumerate(edge):
				for j,px in enumerate(row):
					if px:
						img[i][j] = [255, 255, 255]

			cv2.imshow('RoniStream', img)
			cv2.waitKey(1)


if __name__ == '__main__':
	main()
