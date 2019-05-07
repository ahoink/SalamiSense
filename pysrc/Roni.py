import socket
import threading
import struct

# Default Network Info
#HOST = '10.250.125.102' # wifi
HOST = '129.25.33.136' # usual eth0
#HOST = "10.248.100.51" #demo eth0
#HOST='192.168.1.126'
PORT = 8888

# Data Types
TYPE_RGB =      0
TYPE_DEPTH =    1
TYPE_EDGE =     2
TYPE_3D =		3

class RoniRecvThread(threading.Thread):
	def __init__(self, conn):
		super(RoniRecvThread, self).__init__()
		self.conn = conn
		self.go = True
		self.data = [[],[],[],[]]

	def run(self):
		while self.go:
			st = self.recvAll(8)
			if not st:
				continue
			st = struct.unpack('>II', st)
			msgLen = st[0]
			dType = st[1]
			data = self.recvAll(msgLen)
			if data:
				self.data[dType] = data
		self.conn.close()

	def recvAll(self, n):
		data = b''
		while len(data) < n:
			pack = self.conn.recv(n - len(data))
			if not pack:
				return None
			data += pack
		return data

	def getData(self, dType):
		data = self.data[dType]
		self.data[dType] = []
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

	def receiveData(self, conn, dType=0):
		return conn.getData(dType)

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

	def sendData(self, data, dType=0):
		try:
			msg = struct.pack('>II', len(data), dType) + data
			self.sock.sendall(msg)
			return True
		except:
			return False

	def close(self):
		self.sock.close()

def main():
	print("Don't do that")

if __name__ == '__main__':
	main()	
