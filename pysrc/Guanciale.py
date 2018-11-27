import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

GRID = (int((640 + 15) / 16), int((480 + 15) / 16), 1)
BLOCK = (16, 16, 1)

def makeGpuFun(funcName, funcStr):
	mod = SourceModule(funcStr)
	func = mod.get_function(funcName)
	return func

def gpuMemcpy(data, size):
	gpuMem = cuda.mem_alloc(size)
	cuda.memcpy_htod(gpuMem, data)
	return gpuMem

def gpuMemget(dest, gpuMem):
	cuda.memcpy_dtoh(dest, gpuMem)

def gpuMemfree(gpuMem):
	#TODO
	#cuda.mem_free
	return

def execute(fun, *args, grid=GRID, block=BLOCK):
	fun(*args, grid=grid, block=block)

