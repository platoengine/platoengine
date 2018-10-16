import exodus
import PlatoServices
import ctypes
from numpy import *

# boilerplate that dynamically loads the mpi library required by Plato.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

pc = [ \
[0.2, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.0,  0.0,  0.0,  0.0,  0.0 ], \
[1.0, 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  0.2,  0.0,  0.0,  0.0 ], \
[1.0, 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  1.0,  1.0,  0.25, 0.0,  0.0 ], \
[1.0, 1.0,  0.0,  0.0,  0.0,  1.0,  1.0,  0.0,  1.0,  1.0,  0.25, 0.15], \
[0.0, 1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0,  0.0,  1.0,  1.0,  0.35], \
[0.0, 0.0,  1.0,  1.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  1.0 ], \
[0.0, 0.0,  1.0,  1.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  1.0 ], \
[0.0, 1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0,  0.0,  1.0,  1.0,  0.35], \
[1.0, 1.0,  0.0,  0.0,  0.0,  1.0,  1.0,  0.0,  1.0,  1.0,  0.25, 0.15], \
[1.0, 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  1.0,  1.0,  0.25, 0.0,  0.0 ], \
[1.0, 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  0.2,  0.0,  0.0,  0.0 ], \
[0.2, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.0,  0.0,  0.0,  0.0,  0.0 ] ]
xi = 12
yi = 12
zi = 3
bval = 0.35
p = [0.0 for i in range(xi*yi*zi)]
for i in range(xi):
  for j in range(yi):
    p[i*yi*zi+j*zi+0] = bval
    p[i*yi*zi+j*zi+1] = pc[j][i]
    p[i*yi*zi+j*zi+2] = bval

pfloat = [float(pval) for pval in p]

services.importData("MLS Point Values", "SCALAR", pfloat)
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
