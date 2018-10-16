import exodus
import Plato
import PlatoServices
import ctypes
from numpy import *
import random

# boilerplate that dynamically loads the mpi library required by Plato.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global Analyze instance
analyze = Plato.Analyze("square_tri_bc.xml", "alexaApp.xml", "2D square")
analyze.initialize();

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

# create a fake mitchell structure
pMatrix = [ \
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
pList = [0.0 for i in range(xi*yi)]
for i in range(xi):
  for j in range(yi):
    pList[i*yi+j] = pMatrix[j][i]

# import point values into MLS and compute nodal field
services.importData("MLS Point Values", "SCALAR", pList)
services.compute("Compute Nodal Field");
vals = services.exportData("MLS Field Values", "SCALAR_FIELD")

# import nodal field into Analyze and compute objective value and gradient
analyze.importData("Topology", "SCALAR_FIELD", vals)
analyze.compute("Compute Objective")
solx = analyze.exportData("Solution X", "SCALAR_FIELD")
soly = analyze.exportData("Solution Y", "SCALAR_FIELD")
obj_grad  = analyze.exportData("Objective Gradient", "SCALAR_FIELD")
obj_value = analyze.exportData("Objective Value", "SCALAR")

# import objective gradient into MLS and map to MLS point gradients
services.importData("MLS Field Values", "SCALAR_FIELD", obj_grad)
services.compute("Map Field to Points");
mls_obj_grad = services.exportData("Mapped MLS Point Values", "SCALAR")

# import point values into MLS and compute nodal field
services.importData("MLS Point Values", "SCALAR", mls_obj_grad)
services.compute("Compute Nodal Field");
re_vals = services.exportData("MLS Field Values", "SCALAR_FIELD")

# open exodus file for output and configure
outMesh = exodus.ExodusDB()
outMesh.read("square_tri_bc.gen");
numTimes = 1
outMesh.nodeVarNames = ["MLS_Field", "obj_grad", "mapped_obj_grad", "solution_x", "solution_y"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)
outMesh.nodeVars = [[vals,obj_grad,re_vals,solx,soly]]
outMesh.varTimes = [1.0]
outMesh.write("MLS_Field.exo")

