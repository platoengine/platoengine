import nlopt
import exodus
import PlatoPython
import PlatoServices
import ctypes
from numpy import *

# boilerplate that dynamically loads the mpi library required by PlatoPython.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global Analyze instance
appFileName = "analyzeApp.xml"
defaultInputFile = "lbracket_2d.xml"
analyze = PlatoPython.Analyze(defaultInputFile, appFileName, "2D Lbracket")
analyze.initialize();

# create global PlatoServices instance
services = PlatoServices.Services("platoMain.xml", "platoApp.xml", "services")
services.initialize();

# initialize MLS points 
services.compute("Initialize MLS Point Values");
pinit = services.exportData("MLS Point Values", "SCALAR")

# initialize control
targetFraction = 0.5
numOptDofs = len(pinit)

# compute nodal field from MLS points
services.importData("MLS Point Values", "SCALAR", pinit)
services.compute("Compute Nodal Field");
nodeVals = services.exportData("MLS Field Values", "SCALAR_FIELD")

# compute reference volume
numNodes = len(nodeVals)
x = [1.0 for i in range(numNodes)]
analyze.importData("Topology", "SCALAR_FIELD", x)
analyze.compute("Compute Constraint Value")
refValue = analyze.exportData("Constraint Value", "SCALAR")

# open exodus file for output and configure
inputMeshName = "lbracket_2d.gen"
outMesh = exodus.ExodusDB()
outMesh.read(inputMeshName)
outMesh.nodeVarNames = ["topology"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)

outMesh.nodeVars = [[nodeVals]]
outMesh.varTimes = [1.0]


#define objective
def f(x, grad):

  # compute nodal field from MLS points
  services.importData("MLS Point Values", "SCALAR", x.tolist())
  services.compute("Compute Nodal Field");
  nodeVals = services.exportData("MLS Field Values", "SCALAR_FIELD")

  # import current design into Analyze instance
  analyze.importData("Topology", "SCALAR_FIELD", nodeVals)

  if grad.size > 0:

    outMesh.nodeVars.append([nodeVals])
    prevStep = outMesh.varTimes[len(outMesh.varTimes)-1]
    outMesh.varTimes.append(prevStep+1)

    value = 0.0
    for i in range(grad.size):
      grad[i] = 0.0

    # compute cell solutions
    analyze.compute("Compute Objective")
  
    # export output data from Analyze instance
    obj_grad  = analyze.exportData("Objective Gradient", "SCALAR_FIELD")
    value = analyze.exportData("Objective Value", "SCALAR")
  
    # import objective gradient into MLS and map to MLS point gradients
    services.importData("MLS Field Values", "SCALAR_FIELD", obj_grad)
    services.compute("Map Field to Points");
    grad[:] = services.exportData("Mapped MLS Point Values", "SCALAR")

    print " objective value: ", value
    return value

  else:

    value = 0.0

    # compute cell solutions
    analyze.compute("Compute Objective Value")
  
    # export output data from Analyze instance
    value = analyze.exportData("Objective Value", "SCALAR")
  
    print " objective value: ", value
    return value




#define constraint
def g(x, grad):

  # compute nodal field from MLS points
  services.importData("MLS Point Values", "SCALAR", x.tolist())
  services.compute("Compute Nodal Field");
  nodeVals = services.exportData("MLS Field Values", "SCALAR_FIELD")

  # import current design into Analyze instance
  analyze.importData("Topology", "SCALAR_FIELD", nodeVals)

  if grad.size > 0:

    analyze.compute("Compute Constraint")
    con_grad = analyze.exportData("Constraint Gradient", "SCALAR_FIELD")
    value = analyze.exportData("Constraint Value", "SCALAR")

    # import constraint gradient into MLS and map to MLS point gradients
    services.importData("MLS Field Values", "SCALAR_FIELD", con_grad)
    services.compute("Map Field to Points");
    grad[:] = services.exportData("Mapped MLS Point Values", "SCALAR")

    val = value/refValue - targetFraction
    print " constraint value: ", val
    return val

  else:
  
    analyze.compute("Compute Constraint Value")
    value = analyze.exportData("Constraint Value", "SCALAR")
    
    val = value/refValue - targetFraction
    print " constraint value: ", val
    return val




# create nlopt instance
opt = nlopt.opt(nlopt.LD_MMA, numOptDofs)

opt.set_min_objective(f)
opt.add_inequality_constraint(g, 1e-6)
opt.set_lower_bounds(-0.299)
opt.set_upper_bounds(0.299)
opt.set_xtol_rel(1e-4)
opt.set_ftol_rel(1e-3)
opt.set_maxeval(10)

xopt = opt.optimize(pinit)
minf = opt.last_optimum_value()

print "optimum at: ", minf
fauxgrad = array([])
print "eval f() at min point: ", f(xopt,fauxgrad)
print "eval g() at min point: ", g(xopt,fauxgrad)

services.importData("MLS Point Values", "SCALAR", xopt.tolist())
services.compute("Compute Nodal Field");
nodeVals = services.exportData("MLS Field Values", "SCALAR_FIELD")
    
outMesh.nodeVars.append([nodeVals])
prevStep = outMesh.varTimes[len(outMesh.varTimes)-1]
outMesh.varTimes.append(prevStep+1)

# write output
outputMeshName   = "Mitchell2D.exo"
outMesh.write(outputMeshName)


# finalize mpi and kokkos.  Not essential, but you'll get warnings otherwise.
analyze.finalize()

