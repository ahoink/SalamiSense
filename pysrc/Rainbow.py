R = 255
G = 0
B = 0

def tick(c = 1):
	global R
	global G
	global B

	for _ in range(c):
		if (R == 255) and (G < 255):
			if B > 0:
				B = B-5
			else:
				G = G+5
		elif (G == 255) and (B < 255):
			if R > 0:
				R = R-5
			else:
				B = B+5
		elif (B == 255) and (R < 255):
			if G > 0:
				G = G-5
			else:
				R = R+5
	return (R, G, B)

