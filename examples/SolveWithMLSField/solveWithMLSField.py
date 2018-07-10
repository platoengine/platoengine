#import nlopt
import exodus
import Plato
import PlatoServices
import ctypes
from numpy import *
import random

# boilerplate that dynamically loads the mpi library required by Plato.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global Analyze instance
analyze = Plato.Analyze("square_tet_bc.xml", "alexaApp.xml", "3D square")
analyze.initialize();

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

# create a fake mitchell structure
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

# import point values into MLS and compute nodal field
services.importData("MLS Point Values", "SCALAR", pfloat)
services.compute("Compute Nodal Field");
vals = services.exportData("MLS Field Values", "SCALAR_FIELD")

# import nodal field into Analyze and solve
analyze.importData("Topology", "SCALAR_FIELD", vals)
analyze.compute("Compute Displacement Solution")
solx = analyze.exportData("Solution X", "SCALAR_FIELD")
soly = analyze.exportData("Solution Y", "SCALAR_FIELD")
solz = analyze.exportData("Solution Z", "SCALAR_FIELD")

# open exodus file for output and configure
outMesh = exodus.ExodusDB()
outMesh.read("square_tet_bc.gen");
numTimes = 1
outMesh.nodeVarNames = ["MLS_Field", "solution_x", "solution_y", "solution_z"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)
outMesh.nodeVars = [[vals,solx,soly,solz]]
outMesh.varTimes = [1.0]
outMesh.write("MLS_Field.exo")

