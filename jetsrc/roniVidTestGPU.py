import cv2
import numpy as np
import time

import pysrc.Guanciale as GC
import pysrc.Roni as Roni
import pysrc.Coppa as Coppa


funcStr =\
"""
__global__ void func(char* img, char* edge, int height, int width)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	int idx = (x + y * width);
	if (edge[idx])
	{
		int idx3 = idx * 3;
		img[idx3] = 0;
		img[idx3 + 1] = 255;
		img[idx3 + 2] = 50;
	}
}
"""

def main():
	print("Initializing vid server")
	serv = Roni.RoniServer()
	serv.listen()
	cl = serv.getClient()
	cv2.namedWindow('RoniStream', cv2.WINDOW_AUTOSIZE)

	mod = GC.makeGpuFun("func", funcStr)
	t0 = time.time()
	t1 = 0
	last10 = [0]*10
	it = 0
	while True:
		data = serv.receiveData(cl, Roni.TYPE_RGB)
		if len(data) > 0:
			# get image, copy, and edge detect
			img = Coppa.decodeColorFrame(data)
			orig = np.copy(img)
			edge = cv2.Canny(img, 100, 400)

			w = img.shape[1]
			h = img.shape[0]

			# Copy data to GPU
			imgGPU = GC.gpuMemcpy(img, img.nbytes)
			edgeGPU = GC.gpuMemcpy(edge, edge.nbytes)

			# Launch kernel
			grid = (int((w + 15) / 16), int((h + 15) / 16), 1)
			GC.execute(mod, imgGPU, edgeGPU, 
			np.int32(h), np.int32(w),grid=grid)
			
			# Copy memory back from GPU
			GC.gpuMemget(img, imgGPU)

			# Calculate frame rate and display on image
			t1 = time.time()
			last10[it] = 1.0 / (t1 - t0)
			it = (it + 1) % 10
			fps = np.average(last10)
			t0 = t1
			cv2.putText(img, "FPS: %.2f" % fps, (0, 30),
			cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255))
		
			# Combine original, edge detect, and overlay
			edge3 = np.empty_like(img)
			edge3 = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)
			img = cv2.copyMakeBorder(img, 0, 0, int(w/2), int(w/2), cv2.BORDER_CONSTANT, (0, 0,
			0))
			combined = np.hstack((orig, edge3))
			combined = np.vstack((combined, img))

			# Display
			cv2.imshow('RoniStream', combined)
			cv2.waitKey(1)


if __name__ == '__main__':
	main()
