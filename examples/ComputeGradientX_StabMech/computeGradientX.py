import exodus
import PlatoPython

# boilerplate that dynamically loads the mpi library required by PlatoPython.Analyze
import ctypes
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# create global Analyze instance
analyze = PlatoPython.Analyze("InternalEnergyGradX_input.xml", "InternalEnergyGradX_appfile.xml", "3D bolted bracket")
analyze.initialize();

outMesh = exodus.ExodusDB()
outMesh.read("bolted_bracket.exo");
numNodes = outMesh.numNodes;

# compute gradientX
vals = [1.0 for i in range(numNodes)];
analyze.importData("Topology", "SCALAR_FIELD", vals)
analyze.compute("Compute ObjectiveX")
gradx = analyze.exportData("GradientX@0", "SCALAR_FIELD")
grady = analyze.exportData("GradientX@1", "SCALAR_FIELD")
gradz = analyze.exportData("GradientX@2", "SCALAR_FIELD")

# open exodus file for output and configure
numTimes = 1
outMesh.nodeVarNames = ["gradientx_x", "gradientx_y", "gradientx_z"]
outMesh.numNodeVars = len(outMesh.nodeVarNames)
outMesh.nodeVars = [[gradx,grady,gradz]]
outMesh.varTimes = [1.0]
outMesh.write("InternalEnergyGradX.exo")
