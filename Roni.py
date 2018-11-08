import socket

HOST = '127.0.0.1'
PORT = 8888

class RoniServer:
	def __init__(self, host=HOST, port=PORT):
		#self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.bind((host, port))
		self.connections = []

	def listen(self):
		self.sock.listen()

	def getClient(self):
		conn, addr = self.sock.accept()
		self.connections.append(conn)
		return conn

	def receiveData(self, conn, size=1024):
		return conn.recv(size)

	def close(self):
		for conn in self.connections:
			conn.close()
		self.sock.close()

class RoniClient:
	def __init__(self, host=HOST, port=PORT):
		#self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.host = host
		self.port = port

	def connect(self):
		self.sock.connect((self.host, self.port))

	def sendData(self, data):
		self.sock.sendall(data)

	def close(self):
		self.sock.close()

def main():
	print("Don't do that")

if __name__ == '__main__':
	main()	
