import pysrc.Gouda as gd
from PIL import Image, ImageTk

gui = None

def hurtFunc():
    i = Image.open("M.jpg")
    render = ImageTk.PhotoImage(i)
    gui.setFrameImage(0, render)

gui = gd.Wheel()
gui.defaultConfig()

i = Image.open("J.png")
i = i.resize((640, 480))
render = ImageTk.PhotoImage(i)

gui.addFrame(0, 0, 0)
gui.setFrameImage(0, render)

gui.addFrame(1, 645, 10)
gui.setFrameText(1, "Hello!", "red")

gui.addFrame(2, 645, 125)
gui.setFrameText(2, "Threshold (%):", "blue")
gui.addUserEntry(0, 670, 150, 3, None)

gui.addFrame(7, 645, 200)
gui.setFrameText(7, "Target Temp (ºF):")
gui.addUserEntry(1, 670, 220, 3, None)

gui.addFrame(3, 645, 260)
gui.setFrameText(3, "People in Frame:")
gui.addFrame(4, 670, 280)
gui.setFrameText(4, "idk")

gui.addFrame(5, 645, 310)
gui.setFrameText(5, "Temperature")
gui.addFrame(6, 670, 330)
gui.setFrameText(6, "idk")

gui.addFrame(8, 645, 360)
gui.setFrameText(8, "Humidity")
gui.addFrame(9, 670, 380)
gui.setFrameText(9, "idk")

thresh = None
targ = None
while 1:
    newTh = gui.getEntryValue(0)
    if not newTh == thresh:
        try:
            thresh = int(newTh)
            gui.setFrameText(4, str(thresh))
            gui.setFrameText(9, str(thresh) + " Humids")
        except:
            pass
    newTar = gui.getEntryValue(1)
    if not newTar == targ:
        try:
            targ = int(newTar)
            gui.setFrameText(6, str(targ) + " ºF")
        except:
            pass
    gui.update()









