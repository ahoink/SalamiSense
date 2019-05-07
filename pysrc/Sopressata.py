# Serial interface module to Cape sensors
# STM32F4 UART2, RK3399 /dev/ttyS3
# CO2, TVOC, Temp, Humid

import serial

idxCO2 = 		0
idxTVOC = 		1
idxTemp =		2
idxHumidity = 	3

class Cape(object):
	def __init__(self, device, baudrate, rtscts=True):
		self.device = device
		self.baudrate = baudrate
		self.rtscts = rtscts
		self.port = None
	
	def openPort(self):
		self.port = serial.Serial(port=self.device, \
								baudrate=self.baudrate, \
								bytesize=8, \
								parity='N', \
								stopbits=1, \
								rtscts=self.rtscts, \
								timeout=0) # timeout=0 for non-blocking reads	


	def closePort(self):
		self.port.close()
	

	def readSerial(self):
		if self.port is None:
			print("serial port not opened, now opening...")
			self.openPort()
		
		line = self.port.readline()
		data = str(line.decode())
		data = self.stripBytes(data)
		if data is None:
			return None

		# return sensor readings after "sensors"
		return data.split(',')[1:]


	def stripBytes(self, string):
		sensorStart = string.find("sensors")
		if sensorStart < 0:
			return None
		
		readings = string.replace('\x00','')
		readings = readings.rstrip("\r\n")
		sensorString = readings[sensorStart:]
		
		return sensorString


	def getCO2(self, data):
		return float(data[idxCO2])


	def getTVOC(self, data):
		return float(data[idxTVOC])


	def getTemp(self, data):
		return float(data[idxTemp])


	def getHumidity(self, data):
		return float(data[idxHumidity])


	def getAllSensorData(self, data):
		co2 = self.getCO2(data)
		tvoc = self.getTVOC(data)
		temp = self.getTemp(data)
		humid = self.getHumidity(data)

		return co2, tvoc, temp, humid


