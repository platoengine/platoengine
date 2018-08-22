import exodus
import math

mesh = exodus.ExodusDB()
mesh.read("InternalEnergyGradX.exo")
gradx = mesh.getNodeData(0, "gradientx_x")

x0 = 2.5;

endNodes = []
for iNode in range(mesh.numNodes):
  if abs(mesh.coordinates[0][iNode] - x0) < 1e-8:
    endNodes.append(iNode)

val = 0.0
for inode in endNodes:
  val += gradx[inode]

print "sum on end: ", val
