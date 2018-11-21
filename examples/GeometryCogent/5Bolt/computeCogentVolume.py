import exodus
import PlatoServices
##import ctypes
##from numpy import *

# boilerplate that dynamically loads the mpi library required by Plato.Analyze
#ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

inMesh = exodus.ExodusDB()
inMesh.read("background.gen");
numOptDofs = inMesh.numNodes
xinit = [1.0 for i in range(numOptDofs)]

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

services.importData("Topology", "SCALAR_FIELD", xinit)
services.compute("Compute Cogent Volume");
volume = services.exportData("Volume", "SCALAR")
volume_grad = services.exportData("Volume Gradient", "SCALAR_FIELD")

print "volume: ", volume

# open exodus file for output and configure
outMesh = exodus.ExodusDB()
outMesh.read("background.gen");
numTimes = 1
outMesh.nodeVarNames = ["Volgrad"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)
outMesh.nodeVars = [[volume_grad]]
outMesh.varTimes = [1.0]
outMesh.write("volume_grad.exo")
