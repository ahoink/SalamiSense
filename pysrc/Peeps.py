import pysrc.VectorUtils as vu

class Person(object):
	def __init__(self, center, cid, box, score):
		self.center = center
		self.boxesD = {cid: box}
		self.scoresD = {cid: score}
		self.appearsInD = {cid: True}
		self.dup = False

	def appearsIn(self, cid):
		if cid in self.appearsInD:
			return self.appearsInD[cid]
		self.appearsInD[cid] = False
		return False

def AddToPeople(c, box, people, sens, score):
	xyz = vu.GetPersonVector(c, box)
	cid = c.getID()

	for p in people:
		if p.appearsIn(cid):
			continue
		d = vu.GetVectorDist(xyz, p.center)
		if  d < sens:
			p.boxesD[cid] = box
			p.scoresD[cid] = score
			p.appearsInD[cid] = True
			p.dup = True
			return
	pers = Person(xyz, cid, box, score)
	people.append(pers)

def GetIDBoxes(cid, people):
	cPeeps = [p for p in people if p.appearsIn(cid)]
	b = [p.boxesD[cid] for p in cPeeps]
	s = [p.scoresD[cid] for p in cPeeps]
	d = [p.dup for p in cPeeps]
	return b, s, d
