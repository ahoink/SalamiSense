from Roni import RoniServer

def main():
	print("Initializing server")
	serv = RoniServer()
	serv.listen()
	c = []#serv.getClient()
	opt = 1
	while opt != 0:
		print("Options")
		print(" 0: Quit")
		print(" 1: Get Data")
		print(" 2: New Connection")
		opt = input("> ")

		try:
			opt = int(opt)
		except:
			print("Don't do that")
			continue

		if opt == 1:
			n = input("Client #: ")
			try:
				n = int(n)
				data = serv.receiveData(c[n])
				print(repr(data))
			except:
				print("Don't do that")

		if opt == 2:
			print("Waiting for connection")
			cl = serv.getClient()
			c.append(cl)

	serv.close()

if __name__ == '__main__':
	main()
