import cv2
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
			img = Coppa.decodeColorFrame(data)
			cv2.imshow('RoniStream', img)
			cv2.waitKey(1)


if __name__ == '__main__':
	main()
