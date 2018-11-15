import cv2
import sys
import numpy as np
from PIL import Image
import base64

sys.path.insert(0,"../pysrc")
import Roni

def main():
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	cl = serv.getClient()
	cv2.namedWindow('RoniStream', cv2.WINDOW_AUTOSIZE)

	while True:
		data = serv.receiveData(cl, Roni.TYPE_RGB)
		if len(data) > 0:
			imgData = base64.b64decode(data)
			img = np.frombuffer(imgData, dtype=np.uint8)
			somethingElse = cv2.imdecode(img, flags=1)
			cv2.imshow('RoniStream', somethingElse)
			cv2.waitKey(1)


if __name__ == '__main__':
	main()
