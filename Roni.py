import socket
import threading

HOST = '127.0.0.1'
PORT = 8888

class RoniRecvThread(threading.Thread):
	def __init__(self, conn):
		super(RoniRecvThread, self).__init__()
		self.conn = conn
		self.go = True
		self.data = []

	def run(self):
		while self.go:
			data = self.conn.recv(1024)
			if len(data) > 0:
				self.data = data
		self.conn.close()

	def getData(self):
		data = self.data
		self.data = []
		return data

	def stop(self):
		self.go = False

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
		recvThread = RoniRecvThread(conn)
		self.connections.append(recvThread)
		recvThread.start()
		return recvThread

	def receiveData(self, conn, size=1024):
		return conn.getData()

	def close(self):
		for conn in self.connections:
			conn.stop()
			conn.join()
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
