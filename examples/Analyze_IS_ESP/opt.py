import os
import sys
import exodus
import PlatoServices
import PlatoPython
import ctypes
from contextlib import contextmanager
from numpy import *

##############################################################################
## define functionality that redirects console output to a file
##############################################################################
@contextmanager
def redirected(to=os.devnull):
    '''
    import os

    with redirected(to=filename):
        print("from Python")
        os.system("echo non-Python applications are also supported")
    '''
    fdout = sys.stdout.fileno()
    fderr = sys.stderr.fileno()

    def _redirect_stdout(to):
        sys.stdout.close()                  # + implicit flush()
        os.dup2(to.fileno(), fdout)         # fd writes to 'to' file
        sys.stdout = os.fdopen(fdout, 'w')  # Python writes to fd

    def _redirect_stderr(to):
        sys.stderr.close()                  # + implicit flush()
        os.dup2(to.fileno(), fderr)         # fd writes to 'to' file
        sys.stderr = os.fdopen(fderr, 'w')  # Python writes to fd

    with os.fdopen(os.dup(fdout), 'w') as old_stdout, os.fdopen(os.dup(fderr), 'w') as old_stderr:
        with open(to, 'w') as file:
            _redirect_stdout(to=file)
            _redirect_stderr(to=file)
        try:
            yield # allow code to be run with the redirected stdout
        finally:
            _redirect_stdout(to=old_stdout) # restore stdout.
                                            # buffering and flags such as
                                            # CLOEXEC may be different
            _redirect_stderr(to=old_stderr) # restore stderr.
                                            # buffering and flags such as
                                            # CLOEXEC may be different


# boilerplate that dynamically loads the mpi library required by PlatoPython.Analyze
ctypes.CDLL("libmpi.so",mode=ctypes.RTLD_GLOBAL)

# PA and PE require that the mesh exists, so:
os.system("plato-cli geometry esp --input brick.csm --output-model brick_opt.csm --output-mesh brick.exo --tesselation brick.eto")

# create global PlatoServices instance
engineAppFileName = "plato_operations.xml"
engineInputFile = "platomain.xml"
engine = PlatoServices.Services(engineInputFile, engineAppFileName, "services")
engine.initialize()

# create global Analyze instance
analyzeAppFileName = "plato_analyze_operations_1.xml"
analyzeInputFile = "plato_analyze_input_deck_1.xml"

with redirected():
  analyze = PlatoPython.Analyze(analyzeInputFile, analyzeAppFileName, "Inherent Strain")
  analyze.initialize();

# create global PlatoServices instance
engineAppFileName = "plato_operations.xml"
engineInputFile = "platomain.xml"
engine = PlatoServices.Services(engineInputFile, engineAppFileName, "services")
engine.initialize();

#define objective
def sol(x):

  with redirected():
    engine.importData("Parameters", "SCALAR", x)
    engine.compute("Update Geometry on Change")

  # import current design into Analyze instance
    analyze.importData("Parameters", "SCALAR", x)
    analyze.compute("Reinitialize on Change")

  # compute cell solutions
    analyze.compute("Compute Displacement Solution")
  
  # export output data from Analyze instance
    solx = analyze.exportData("Solution X", "SCALAR_FIELD")
    soly = analyze.exportData("Solution Y", "SCALAR_FIELD")
    solz = analyze.exportData("Solution Z", "SCALAR_FIELD")
  
    return [solx, soly, solz]


#define objective
def f(x):

  with redirected():
    engine.importData("Parameters", "SCALAR", x)
    engine.compute("Update Geometry on Change")

    # import current design into Analyze instance
    analyze.importData("Parameters", "SCALAR", x)
    analyze.compute("Reinitialize on Change")

    # compute cell solutions
    analyze.compute("Compute Objective Value")
  
    # export output data from Analyze instance
    value = analyze.exportData("Objective Value", "SCALAR")
  
    return value


#define objective
def dfdx(x, grad):

  with redirected():
    engine.importData("Parameters", "SCALAR", x)
    engine.compute("Update Geometry on Change")

    # import current design into Analyze instance
    analyze.importData("Parameters", "SCALAR", x)
    analyze.compute("Reinitialize on Change")

    # compute cell solutions
    analyze.compute("Compute Objective")
  
    # export output data from Analyze instance
    analyzeGrad = analyze.exportData("Objective Gradient", "SCALAR", len(grad))
    value = analyze.exportData("Objective Value", "SCALAR")
  
    for iVal in range(len(analyzeGrad)):
      grad[iVal] = analyzeGrad[iVal]
  
    return value


xinit = [1.0, 1.0, 1.0, 0.25]
gradP = [0.0 for i in range(len(xinit))]

val = dfdx(xinit, gradP)
dvdx = array(gradP)
x = array(xinit)

print "objective is: ", val
print "objective gradient is: ", gradP

stepSize = 0.01
numSteps = 1

vals = []

for iStep in range(numSteps):
  gnorm = linalg.norm(dvdx,2)
  dx = stepSize/gnorm*dvdx
  x = x + dx
  dv = dot(dvdx,dx)
  predictedVal = val + dv
  val = dfdx(x.tolist(), gradP)
  dvdx = array(gradP)
  print "x: ", x.tolist()
  print "gradP: ", gradP
  vals.append([(iStep+1)*stepSize, val, predictedVal])

print vals

# finalize mpi and kokkos.  Not essential, but you'll get warnings otherwise.
analyze.finalize()
engine.finalize()

diff = vals[0][1] - vals[0][2]
tsum = vals[0][1] + vals[0][2]
error = math.fabs(diff/tsum)
print "error: ", error
if error > 1.0e-3:
  print "failed"
  sys.exit(1)
else:
  print "passed"
  sys.exit(0)
