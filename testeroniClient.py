from Roni import RoniClient

def main():
	print("Initializing Client")
	client = RoniClient()
	print("Connecting")
	client.connect()
	print("Connected, go ahead and type something you jabroni")
	data = "Hey there Buddy"
	while data != "/quit":
		data = input()
		client.sendData(data.encode('utf8'))
	client.close()

if __name__ == '__main__':
	main()
