import threading
import pysrc.Roni as Roni

class NodeData(object):
	def __init__(self):
		self.frame = None
		self.people = 0
		self.temp = 0
		self.hum = 0
		self.co2 = 0
	
	def setFrame(self, frame):
		self.frame = frame
	def getFrame(self):
		return self.frame

	def setPeople(self, people):
		self.people = people
	def getPeople(self):
		return self.people
	
	def setTemp(self, temp):
		self.temp = temp
	def getTemp(self):
		return self.temp

	def setHumidity(self, humidity):
		self.hum = humidity
	def getHumidity(self):
		return self.humidity

	def setCO2(self, co2):
		self.co2 = co2
	def getCO2(self):
		return self.co2

class ConnThread(threading.Thread):
	def __init__(self, roniHost):
		super(ConnThread, self).__init__()
		self.host = roniHost
		self.running = False
		self.client = None

	def getClient(self):
		c = self.client
		self.client = None
		return c

	def run(self):
		self.running = True
		self.client = self.host.getClient()
		self.running = False

class NewConnHandler(object):
	STATE_IDLE = 0
	STATE_RUNNING = 1
	STATE_FINISHED = 2

	def __init__(self):
		self.state = self.STATE_IDLE
		self.client = None
		self.thrd = None

	def tick(self):
		if self.state == self.STATE_RUNNING:
			if self.thrd is not None and not self.thrd.running:
				self.client = self.thrd.getClient()
				self.thrd.join()
				self.thrd = None
				self.state = self.STATE_FINISHED

	def newClient(self, roniHost):
		if self.state != self.STATE_IDLE:
			return False
		self.thrd = ConnThread(roniHost)
		self.thrd.start()
		self.state = self.STATE_RUNNING
		return True
	
	def getClient(self):
		if self.state == self.STATE_FINISHED:
			self.state = self.STATE_IDLE
			c = self.client
			self.client = None
			return c
		return None

	def getState(self):
		return self.state

	def getStatusStr(self):
		if self.state == self.STATE_IDLE:
			return "New  Node"
		elif self.state == self.STATE_RUNNING:
			return "Searching"
		elif self.state == self.STATE_FINISHED:
			return "Connected"



















