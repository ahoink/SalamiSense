import numpy as np

# Get new basis matrix from vectors A, B, C
def GetNewBasis(A, B, C):
	try:
		V1 = np.subtract(B, A)
		V1 = V1 / np.linalg.norm(V1)
		V2 = np.cross(V1, (np.subtract(C, A)))
		V2 = V2 / np.linalg.norm(V2)
		V3 = np.cross(V1, V2)
		basis = np.transpose([V1, V2, V3])
		return basis
	except:
		return [[0,0,0],[0,0,0],[0,0,0]]

# Convert vector v to basis vector b with origin o
def ChangeBasis(v, b, o):
	try:
		bi = np.linalg.inv(b)
		vp = np.subtract(v, o)
		newv = np.matmul(bi, np.transpose(vp))
		return np.transpose(newv)
	except:
		return [0,0,0]

def GetPersonVector(client, box):
	verts = client.getVertex()
	x = int((box[0] + box[2]) / 2)
	y = int((box[3] + box[1]) / 2)
	pt = [0, 0, 0]
	if len(verts) == (640*480):
		pt = verts[x + y * 640]
	basis = client.getBasis()
	origin = client.getOrigin()
	return ChangeBasis(pt, basis, origin)
	
def GetVectorDist(v1, v2):
	return np.linalg.norm(np.subtract(v2, v1))



