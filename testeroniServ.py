from Roni import RoniServer

def main():
	print("Initializing server")
	serv = RoniServer()
	serv.listen()
	print("Waiting for connection")
	c = serv.getClient()
	opt = 1
	while opt != 0:
		print("Options")
		print(" 0: Quit")
		print(" 1: Get Data")
		opt = input("> ")

		try:
			opt = int(opt)
		except:
			print("Don't do that")
			continue

		if opt == 1:
			data = serv.receiveData(c)
			print(repr(data))
	serv.close()

if __name__ == '__main__':
	main()
