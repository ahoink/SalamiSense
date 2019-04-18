import tkinter as tk

class Wheel(tk.Frame):
	def __init__(self):
		self.master = tk.Tk()
		tk.Frame.__init__(self, self.master)
		self.buttonDict = {}
		self.frameDict = {}
		self.entryDict = {}
		self.color = None
		#self.pack(fill=tk.BOTH, expand=1)

	def setSize(self, sizeStr):
		self.master.geometry(sizeStr)
		self.pack(fill=tk.BOTH, expand=1)

	def setTitle(self, title):
		self.master.title(title)

	def update(self):
		self.master.update()

	def mainloop(self):
		self.master.mainloop()

	def setBackground(self, color):
		self.configure(bg=color)
		self.color = color

	def addButton(self, identifier, text, dx, dy, command):
		if identifier in self.buttonDict:
			self.buttonDict[identifier].destroy()
		self.buttonDict[identifier] = tk.Button(self, text=text, command=command, bg=self.color)
		self.buttonDict[identifier].place(x=dx, y=dy)

	def addUserEntry(self, identifier, dx, dy, width, command=None):
		if identifier in self.entryDict:
			self.entryDict[identifier].destroy()
		self.entryDict[identifier] = tk.Entry(self, width=width, validate="key", validatecommand=command)
		self.entryDict[identifier].place(x=dx, y=dy)

	def getEntryValue(self, identifier):
		if identifier in self.entryDict:
			return self.entryDict[identifier].get()
		return None

	def addFrame(self, identifier, dx, dy):
		if identifier in self.frameDict:
			self.frameDict[identifier].destroy()
		self.frameDict[identifier] = tk.Label(self, bg = self.color)
		self.frameDict[identifier].place(x=dx, y=dy)

	def setFrameImage(self, identifier, image):
		if identifier in self.frameDict:
			self.frameDict[identifier].configure(image=image)
			self.frameDict[identifier].image = image

	def setFrameText(self, identifier, text, color=None):
		if identifier in self.frameDict:
			self.frameDict[identifier].configure(text=text, foreground=color)
			#self.frameDict[identifier].pack()

	def defaultConfig(self):
		self.setTitle("Salami Sense")
		self.setSize("800x480")
		self.setBackground("grey")

