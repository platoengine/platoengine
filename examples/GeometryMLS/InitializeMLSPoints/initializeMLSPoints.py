import exodus
import PlatoServices
import ctypes
from numpy import *

# boilerplate that dynamically loads the mpi library required by Plato.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

services.compute("Initialize MLS Point Values");
p = services.exportData("MLS Point Values", "SCALAR")

services.importData("MLS Point Values", "SCALAR", p)
services.compute("Compute Nodal Field");
vals = services.exportData("MLS Field Values", "SCALAR_FIELD")

# open exodus file for output and configure
outMesh = exodus.ExodusDB()
outMesh.read("square_tet_bc.gen");
numTimes = 1
outMesh.nodeVarNames = ["MLS_Field"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)
outMesh.nodeVars = [[vals]]
outMesh.varTimes = [1.0]
outMesh.write("MLS_Field.exo")
